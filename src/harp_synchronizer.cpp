#include <harp_synchronizer.h>

HarpSynchronizer::HarpSynchronizer(uart_port_t uart_id, uint8_t uart_rx_pin)
:uart_id_{uart_id}, packet_index_{0}, sync_data_{0, 0, 0, 0},
 state_{RECEIVE_HEADER_0}, new_timestamp_{false}, offset_us_64_{0},
 has_synced_{false}, intr_handle_{nullptr}
{
    if (self == nullptr)
        self = this;
    // Configure UART for Harp sync input.
    uart_config_t cfg = {
        .baud_rate  = HARP_SYNC_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(uart_id_, &cfg);
    // Map RX pin; TX/RTS/CTS not used.
    uart_set_pin(uart_id_, UART_PIN_NO_CHANGE, (int)uart_rx_pin,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Install driver with minimal RX buffer; no TX buffer, no event queue.
    uart_driver_install(uart_id_, 256, 0, 0, NULL, 0);
    // Enable RX FIFO-full interrupt at threshold of 1 byte.
    uart_set_rx_full_threshold(uart_id_, 1);
    uart_enable_rx_intr(uart_id_);
    // Attach IRAM-safe ISR via esp_intr_alloc.
    esp_intr_alloc(uart_periph_signal[uart_id_].irq,
                   ESP_INTR_FLAG_IRAM,
                   uart_rx_callback_isr, NULL,
                   &intr_handle_);
}

HarpSynchronizer::~HarpSynchronizer()
{
    if (intr_handle_) esp_intr_free(intr_handle_);
    uart_driver_delete(uart_id_);
    self = nullptr;
}

HarpSynchronizer& HarpSynchronizer::init(uart_port_t uart_id, uint8_t uart_rx_pin)
{
    static HarpSynchronizer synchronizer(uart_id, uart_rx_pin);
    return synchronizer;
}

// ESP-IDF ISR wrapper — called with arg = NULL; delegates to shared logic.
void IRAM_ATTR HarpSynchronizer::uart_rx_callback_isr(void* /*arg*/)
{
    uart_rx_callback();
}

// ---------------------------------------------------------------------------
// Harp packet state machine (runs inside the UART RX ISR)
// ---------------------------------------------------------------------------
void HarpSynchronizer::uart_rx_callback()
{
    SyncState next_state_{self->state_};
    uint8_t new_byte;

    // Helpers to abstract byte availability and byte reading from the
    // ESP-IDF UART driver without blocking.
    auto byte_available = [&]() -> bool {
        size_t len = 0;
        uart_get_buffered_data_len(self->uart_id_, &len);
        return len > 0;
    };

    auto read_byte = [&]() -> uint8_t {
        uint8_t b = 0;
        uart_read_bytes(self->uart_id_, &b, 1, 0);
        return b;
    };

    while (byte_available() && !self->new_timestamp_)
    {
        new_byte = read_byte();
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
                self->sync_data_[self->packet_index_++] = new_byte;
                if (self->packet_index_ == 4)
                {
                    next_state_ = RECEIVE_HEADER_0;
                    self->packet_index_ = 0;
                    self->new_timestamp_ = true;
                }
                else
                    next_state_ = RECEIVE_TIMESTAMP;
                break;
        }
        self->state_ = next_state_;
    }

    if (!self->new_timestamp_)
        return;

    // Apply new timestamp: interpret 4-byte little-endian uint32_t,
    // add 1[s] per protocol spec (sequence encodes the previous second).
    uint32_t sec = *((uint32_t*)(self->sync_data_)) + 1;
    uint64_t curr_harp_us = uint64_t(sec) * 1'000'000ULL - HARP_SYNC_OFFSET_US;

    self->offset_us_64_ = (uint64_t)esp_timer_get_time() - curr_harp_us;

    self->has_synced_ = true;
    self->new_timestamp_ = false;
#ifdef DEBUG
    //printf("harp time: %llu [us] | offset: %lld\r\n", curr_harp_us, self->offset_us_64_);
#endif
}
