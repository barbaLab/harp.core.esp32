#include <harp_synchronizer.h>
#include <hal/uart_periph.h>
#include <hal/uart_ll.h>
#include <soc/uart_struct.h>

#include <cstring>

namespace {

constexpr uint32_t kSyncRxRelevantInterruptMask =
    (1u << 0) | // RX FIFO full
    (1u << 4) | // RX FIFO overflow
    (1u << 8);  // RX FIFO timeout

constexpr uint32_t kSyncExpectedLastByteStartLagUs = 120;
constexpr uint32_t kSyncLastByteStartWindowMinUs = 40;
constexpr uint32_t kSyncLastByteStartWindowMaxUs = 500;
constexpr uint32_t kSyncPacketPeriodUs = 1'000'000;

constexpr uint8_t kSyncPacketHeader0 = 0xAA;
constexpr uint8_t kSyncPacketHeader1 = 0xAF;

constexpr uint8_t kSyncPacketLength = 6;

static volatile uart_dev_t& uart_dev(uart_port_t uart_id)
{
    switch (uart_id)
    {
        case UART_NUM_0:
            return UART0;
        case UART_NUM_1:
            return UART1;
        case UART_NUM_2:
            return UART2;
        default:
            return UART0;
    }
}

static inline uint32_t abs_u32_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static inline uint64_t lift_us32_to_recent_us64(uint64_t now_us64, uint32_t ts_us32)
{
    uint64_t lifted = (now_us64 & 0xFFFFFFFF00000000ULL) | uint64_t(ts_us32);
    if (lifted > now_us64 && (lifted - now_us64) > 0x80000000ULL)
        lifted -= 0x100000000ULL;
    else if (lifted < now_us64 && (now_us64 - lifted) > 0x80000000ULL)
        lifted += 0x100000000ULL;
    return lifted;
}

} // namespace

HarpSynchronizer::HarpSynchronizer(uart_port_t uart_id, uint8_t uart_rx_pin,
                                   int uart_tx_pin)
:intr_handle_{nullptr}, uart_id_{uart_id}, state_{RECEIVE_HEADER_0},
 packet_index_{0}, new_timestamp_{false}, offset_us_64_{0},
 has_synced_{false}, sync_data_{0, 0, 0, 0},
 edge_capture_us32_{0}, edge_capture_head_{0}
{
    if (self == nullptr)
        self = this;
    uart_rx_pin_ = static_cast<gpio_num_t>(uart_rx_pin);
    uart_tx_pin_ = uart_tx_pin;
    tx_schedule_timer_ = nullptr;
    tx_generation_enabled_ = false;
    clock_gen_mode_ = false;
    clock_rep_mode_ = false;

    // Configure UART for Harp sync input.
    uart_config_t cfg = {};
    cfg.baud_rate  = HARP_SYNC_BAUDRATE;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    uart_param_config(uart_id_, &cfg);
    // Map RX pin; TX is optional, RTS/CTS not used.
    uart_set_pin(uart_id_, uart_tx_pin_, static_cast<int>(uart_rx_pin_),
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Configure GPIO edge capture on the same physical sync input used by UART.
    gpio_config_t gpio_cfg = {};
    gpio_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(uart_rx_pin_));
    gpio_cfg.mode = GPIO_MODE_INPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&gpio_cfg);

    if (!gpio_isr_service_installed_)
    {
        esp_err_t isr_install_result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (isr_install_result == ESP_OK || isr_install_result == ESP_ERR_INVALID_STATE)
            gpio_isr_service_installed_ = true;
    }
    if (gpio_isr_service_installed_)
        gpio_isr_handler_add(uart_rx_pin_, rx_edge_capture_isr, this);

    volatile uart_dev_t& dev = uart_dev(uart_id_);
    dev.int_ena.val = 0;
    dev.int_clr.val = kSyncRxRelevantInterruptMask;
    dev.conf0.rxfifo_rst = 1;
    dev.conf0.rxfifo_rst = 0;
    dev.conf1.rxfifo_full_thrhd = 1;
    dev.conf1.rx_tout_en = 1;
    dev.mem_conf.rx_tout_thrhd = 2;
    dev.int_ena.val = kSyncRxRelevantInterruptMask;

    // Attach a dedicated IRAM ISR at a fixed priority so the timestamp path
    // reads the hardware FIFO directly instead of waiting on the driver buffer.
    esp_intr_alloc(uart_periph_signal[uart_id_].irq,
                   ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
                   uart_rx_callback_isr, NULL,
                   &intr_handle_);

    if (uart_tx_pin_ != UART_PIN_NO_CHANGE)
    {
        esp_timer_create_args_t tx_timer_args = {};
        tx_timer_args.callback = &tx_schedule_timer_isr;
        tx_timer_args.arg = this;
        tx_timer_args.name = "harp_sync_tx";
#if defined(CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD) && CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD
        tx_timer_args.dispatch_method = ESP_TIMER_ISR;
#endif
        if (esp_timer_create(&tx_timer_args, &tx_schedule_timer_) == ESP_OK)
        {
            tx_generation_enabled_ = true;
            update_tx_scheduler_state();
        }
    }
}

HarpSynchronizer::~HarpSynchronizer()
{
    if (tx_schedule_timer_)
    {
        esp_timer_stop(tx_schedule_timer_);
        esp_timer_delete(tx_schedule_timer_);
        tx_schedule_timer_ = nullptr;
    }

    gpio_isr_handler_remove(uart_rx_pin_);
    if (intr_handle_) esp_intr_free(intr_handle_);
    self = nullptr;
}

HarpSynchronizer& HarpSynchronizer::init(uart_port_t uart_id, uint8_t uart_rx_pin,
                                         int uart_tx_pin)
{
    static HarpSynchronizer synchronizer(uart_id, uart_rx_pin, uart_tx_pin);
    return synchronizer;
}

// ESP-IDF ISR wrapper — called with arg = NULL; delegates to shared logic.
void IRAM_ATTR HarpSynchronizer::uart_rx_callback_isr(void* /*arg*/)
{
    uart_rx_callback();
}

void IRAM_ATTR HarpSynchronizer::rx_edge_capture_isr(void* arg)
{
    HarpSynchronizer* sync = static_cast<HarpSynchronizer*>(arg);
    uint8_t head = sync->edge_capture_head_;
    sync->edge_capture_us32_[head] = static_cast<uint32_t>(esp_timer_get_time());
    sync->edge_capture_head_ = static_cast<uint8_t>((head + 1U) % EDGE_CAPTURE_BUFFER_SIZE);
}

void IRAM_ATTR HarpSynchronizer::tx_schedule_timer_isr(void* arg)
{
    HarpSynchronizer* sync = static_cast<HarpSynchronizer*>(arg);
    if (!sync->tx_generation_enabled_ || !sync->clock_gen_mode_)
        return;

    const uint32_t current_sec = static_cast<uint32_t>(sync->time_us_64() / 1'000'000ULL);
    sync->emit_sync_packet_from_second(current_sec);

    sync->schedule_next_tx_packet();
}

void HarpSynchronizer::set_clock_modes(bool clk_rep, bool clk_gen)
{
    clock_rep_mode_ = clk_rep && supports_clock_output();
    clock_gen_mode_ = clk_gen && supports_clock_output();
    update_tx_scheduler_state();
}

void HarpSynchronizer::update_tx_scheduler_state()
{
    if (!tx_generation_enabled_ || !tx_schedule_timer_)
        return;

    if (clock_gen_mode_)
        schedule_next_tx_packet();
    else
        esp_timer_stop(tx_schedule_timer_);
}

void HarpSynchronizer::schedule_next_tx_packet()
{
    if (!tx_schedule_timer_ || !tx_generation_enabled_ || !clock_gen_mode_)
        return;

    const uint64_t now_harp_us = time_us_64();
    uint64_t next_boundary_harp_us = (now_harp_us / kSyncPacketPeriodUs + 1ULL) * kSyncPacketPeriodUs;
    uint64_t packet_start_harp_us = next_boundary_harp_us - (HARP_SYNC_OFFSET_US + (5ULL * HARP_SYNC_BYTE_TIME_US));
    if (packet_start_harp_us <= now_harp_us)
        packet_start_harp_us += kSyncPacketPeriodUs;

    const uint64_t packet_start_system_us = harp_to_system_us_64(packet_start_harp_us);
    const uint64_t now_system_us = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t delay_us =
        (packet_start_system_us > now_system_us) ?
            (packet_start_system_us - now_system_us) : 1ULL;

    esp_timer_stop(tx_schedule_timer_);
    esp_timer_start_once(tx_schedule_timer_, delay_us);
}

void HarpSynchronizer::emit_sync_packet_from_second(uint32_t second_value)
{
    if (!supports_clock_output())
        return;

    volatile uart_dev_t& dev = uart_dev(uart_id_);
    uart_dev_t* uart_hw = const_cast<uart_dev_t*>(&dev);
    if (uart_ll_get_txfifo_len(uart_hw) < kSyncPacketLength)
        return;

    uint8_t packet[kSyncPacketLength] = {
        kSyncPacketHeader0,
        kSyncPacketHeader1,
        static_cast<uint8_t>(second_value & 0xFFu),
        static_cast<uint8_t>((second_value >> 8) & 0xFFu),
        static_cast<uint8_t>((second_value >> 16) & 0xFFu),
        static_cast<uint8_t>((second_value >> 24) & 0xFFu),
    };

    uart_ll_write_txfifo(uart_hw, packet, kSyncPacketLength);
}

uint64_t HarpSynchronizer::select_last_byte_start_time_us(uint64_t now_us) const
{
    const uint32_t now32 = static_cast<uint32_t>(now_us);
    const uint8_t head_snapshot = edge_capture_head_;

    bool found = false;
    uint32_t best_ts32 = 0;
    uint32_t best_score = UINT32_MAX;

    for (size_t i = 0; i < EDGE_CAPTURE_BUFFER_SIZE; ++i)
    {
        const uint8_t idx = static_cast<uint8_t>((head_snapshot + EDGE_CAPTURE_BUFFER_SIZE - 1 - i) % EDGE_CAPTURE_BUFFER_SIZE);
        const uint32_t ts32 = edge_capture_us32_[idx];
        if (ts32 == 0)
            continue;

        const uint32_t age = now32 - ts32;
        if (age < kSyncLastByteStartWindowMinUs || age > kSyncLastByteStartWindowMaxUs)
            continue;

        const uint32_t score = abs_u32_diff(age, kSyncExpectedLastByteStartLagUs);
        if (!found || score < best_score)
        {
            found = true;
            best_score = score;
            best_ts32 = ts32;
        }
    }

    if (!found)
        return now_us;

    return lift_us32_to_recent_us64(now_us, best_ts32);
}

// ---------------------------------------------------------------------------
// Harp packet state machine (runs inside the UART RX ISR)
// ---------------------------------------------------------------------------
void HarpSynchronizer::uart_rx_callback()
{
    volatile uart_dev_t& dev = uart_dev(self->uart_id_);

    if (self->clock_gen_mode_)
    {
        // Spec behavior: when generating sync, ignore incoming clock messages.
        while (dev.status.rxfifo_cnt > 0)
            (void)dev.fifo.rxfifo_rd_byte;
        dev.int_clr.val = kSyncRxRelevantInterruptMask;
        return;
    }

    const uint32_t pending_interrupts = dev.int_st.val & kSyncRxRelevantInterruptMask;
    if (pending_interrupts == 0)
        return;

    if ((pending_interrupts & (1u << 4)) != 0)
    {
        dev.conf0.rxfifo_rst = 1;
        dev.conf0.rxfifo_rst = 0;
        self->state_ = RECEIVE_HEADER_0;
        self->packet_index_ = 0;
        self->new_timestamp_ = false;
        dev.int_clr.val = (1u << 4);
        return;
    }

    SyncState next_state_{self->state_};
    uint8_t new_byte = 0;

    while ((dev.status.rxfifo_cnt > 0) && !self->new_timestamp_)
    {
        new_byte = static_cast<uint8_t>(dev.fifo.rxfifo_rd_byte & 0xFFu);
#ifdef DEBUG
        //printf("state: %d | byte: 0x%x\r\n", self->state_, new_byte);
#endif
        switch (self->state_)
        {
            case RECEIVE_HEADER_0:
                if (new_byte == 0xAA)
                    next_state_ = RECEIVE_HEADER_1;
                break;
            case RECEIVE_HEADER_1:
                if (new_byte == 0xAF)
                    next_state_ = RECEIVE_TIMESTAMP;
                else
                    next_state_ = RECEIVE_HEADER_0;
                break;
            case RECEIVE_TIMESTAMP:
            {
                uint8_t packet_index = self->packet_index_;
                self->sync_data_[packet_index] = new_byte;
                packet_index++;
                self->packet_index_ = packet_index;
                if (packet_index == 4)
                {
                    next_state_ = RECEIVE_HEADER_0;
                    self->packet_index_ = 0;
                    self->new_timestamp_ = true;
                }
                else
                    next_state_ = RECEIVE_TIMESTAMP;
                break;
            }
        }
        self->state_ = next_state_;
    }

    dev.int_clr.val = pending_interrupts;

    if (!self->new_timestamp_)
        return;

    // Apply new timestamp: interpret 4-byte little-endian uint32_t,
    // add 1[s] per protocol spec (sequence encodes the previous second).
    uint32_t sec = 0;
    memcpy(&sec, (const void*)self->sync_data_, sizeof(sec));
    sec += 1;
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t last_byte_start_us = self->select_last_byte_start_time_us(now_us);

    // Convert from the captured last-byte start instant to Harp time by
    // accounting for one full UART byte duration plus the protocol offset.
    uint64_t curr_harp_us =
        uint64_t(sec) * 1'000'000ULL - (HARP_SYNC_OFFSET_US + HARP_SYNC_BYTE_TIME_US);

    self->offset_us_64_ = last_byte_start_us - curr_harp_us;

    self->has_synced_ = true;

    if (self->clock_rep_mode_ && self->supports_clock_output())
        self->emit_sync_packet_from_second(sec - 1);

    self->new_timestamp_ = false;
#ifdef DEBUG
    //printf("harp time: %llu [us] | offset: %lld\r\n", curr_harp_us, self->offset_us_64_);
#endif
}
