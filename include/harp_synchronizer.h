#ifndef HARP_SYNCHRONIZER_H
#define HARP_SYNCHRONIZER_H
#include <stdint.h>

// ESP32-S3 HAL dependencies.
#include <driver/uart.h>       // uart_port_t, uart_config_t, uart_*
#include <driver/gpio.h>       // gpio_num_t
#include <esp_intr_alloc.h>    // esp_intr_alloc, intr_handle_t
#include <esp_timer.h>         // esp_timer_get_time()

#ifdef DEBUG
#include <cstdio> // for printf
#endif

#define HARP_SYNC_BAUDRATE (100'000UL)
#define HARP_SYNC_DATA_BITS (8)
#define HARP_SYNC_STOP_BITS (1)
#define HARP_SYNC_BYTE_TIME_US (100) // 8N1 @ 100 kbps

#define HARP_SYNC_OFFSET_US (672) // Receiver-side timing model follows the
                                  // published Harp sync example: the next
                                  // whole second starts 672 us after the last
                                  // packet byte is received.

// Synchronizer that updates timekeeping according to specific UART input.
// Singleton. Targets ESP32-S3 exclusively.
class HarpSynchronizer
{
public:
    enum SyncState
    {
        RECEIVE_HEADER_0,
        RECEIVE_HEADER_1,
        RECEIVE_TIMESTAMP
    };

private:
    HarpSynchronizer(uart_port_t uart_id, uint8_t uart_rx_pin,
                     int uart_tx_pin = UART_PIN_NO_CHANGE);
    ~HarpSynchronizer();
public:
    HarpSynchronizer() = delete;
    HarpSynchronizer(HarpSynchronizer& other) = delete;
    void operator=(const HarpSynchronizer& other) = delete;

/**
 * \brief init the HarpSynchronizer singleton and return a reference to it.
 */
     static HarpSynchronizer& init(uart_port_t uart_id, uint8_t uart_rx_pin,
                                             int uart_tx_pin = UART_PIN_NO_CHANGE);

    static HarpSynchronizer& instance(){return *self;}

    static inline uint64_t system_to_harp_us_64(uint64_t system_time_us)
    {return system_time_us - self->offset_us_64_;}

    /**
     * \brief Set the Harp clock offset so that harp_time = system_time - offset.
     * Called by the synchronizer ISR when a new timestamp packet arrives.
     */
    static inline void set_harp_time_us_64(uint64_t harp_time_us)
    {self->offset_us_64_ = (uint64_t)esp_timer_get_time() - harp_time_us;}

    static inline uint64_t time_us_64()
    {return system_to_harp_us_64((uint64_t)esp_timer_get_time());}

    static inline uint32_t time_us_32()
    {return uint32_t(time_us_64());}

    static inline uint64_t harp_to_system_us_64(uint64_t harp_time_us)
    {return harp_time_us + self->offset_us_64_;}

    static inline uint32_t harp_to_system_us_32(uint64_t harp_time_us)
    {return uint32_t(harp_to_system_us_64(harp_time_us));}

    static inline bool is_synced()
    {
        if (self == nullptr)
            return false;
        return self->has_synced_ || self->clock_gen_mode_;
    }

    bool supports_clock_output() const
    {return uart_tx_pin_ != UART_PIN_NO_CHANGE;}

    bool generator_enabled() const
    {return clock_gen_mode_;}

    bool repeater_enabled() const
    {return clock_rep_mode_;}

    void set_clock_modes(bool clk_rep, bool clk_gen);

private:
    static inline HarpSynchronizer* self = nullptr;
    static inline bool gpio_isr_service_installed_ = false;

    static void uart_rx_callback();
    // ISR wrapper with the signature required by esp_intr_alloc.
    static void IRAM_ATTR uart_rx_callback_isr(void* arg);
    static void IRAM_ATTR rx_edge_capture_isr(void* arg);
    static void IRAM_ATTR tx_schedule_timer_isr(void* arg);

    void schedule_next_tx_packet();
    void update_tx_scheduler_state();
    void emit_sync_packet_from_second(uint32_t second_value);
    uint64_t select_last_byte_start_time_us(uint64_t now_us) const;

    intr_handle_t intr_handle_; // handle returned by esp_intr_alloc; freed in destructor
    uart_port_t   uart_id_;     // UART peripheral index (e.g. UART_NUM_1)
    gpio_num_t    uart_rx_pin_;
    int           uart_tx_pin_;
    esp_timer_handle_t tx_schedule_timer_;
    volatile bool tx_generation_enabled_;
    volatile bool clock_gen_mode_;
    volatile bool clock_rep_mode_;

    volatile SyncState state_;
    volatile uint8_t packet_index_;
    volatile bool new_timestamp_;
    volatile uint64_t offset_us_64_;
    volatile bool has_synced_;
    alignas(uint32_t) volatile uint8_t sync_data_[4];

    static constexpr size_t EDGE_CAPTURE_BUFFER_SIZE = 32;
    volatile uint32_t edge_capture_us32_[EDGE_CAPTURE_BUFFER_SIZE];
    volatile uint8_t edge_capture_head_;

    friend class HarpCore;
};

#endif // HARP_SYNCHRONIZER_H
