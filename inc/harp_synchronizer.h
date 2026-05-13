#ifndef HARP_SYNCHRONIZER_H
#define HARP_SYNCHRONIZER_H
#include <stdint.h>

// ESP32-S3 HAL dependencies.
#include <driver/uart.h>       // uart_port_t, uart_config_t, uart_*
#include <esp_intr_alloc.h>    // esp_intr_alloc, intr_handle_t
#include <esp_timer.h>         // esp_timer_get_time()

#ifdef DEBUG
#include <cstdio> // for printf
#endif

#define HARP_SYNC_BAUDRATE (100'000UL)
#define HARP_SYNC_DATA_BITS (8)
#define HARP_SYNC_STOP_BITS (1)

#define HARP_SYNC_OFFSET_US (672 - 90) // time (in [us]) from the **end** of
                                       // the last packet byte and the time
                                       // specified in that packet.

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
    HarpSynchronizer(uart_port_t uart_id, uint8_t uart_rx_pin);
    ~HarpSynchronizer();
public:
    HarpSynchronizer() = delete;
    HarpSynchronizer(HarpSynchronizer& other) = delete;
    void operator=(const HarpSynchronizer& other) = delete;

/**
 * \brief init the HarpSynchronizer singleton and return a reference to it.
 */
    static HarpSynchronizer& init(uart_port_t uart_id, uint8_t uart_rx_pin);

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
    {return self->has_synced_;}

private:
    static inline HarpSynchronizer* self = nullptr;

    static void uart_rx_callback();
    // ISR wrapper with the signature required by esp_intr_alloc.
    static void IRAM_ATTR uart_rx_callback_isr(void* arg);

    intr_handle_t intr_handle_; // handle returned by esp_intr_alloc; freed in destructor
    uart_port_t   uart_id_;     // UART peripheral index (e.g. UART_NUM_1)

    volatile SyncState state_;
    volatile uint8_t packet_index_;
    volatile bool new_timestamp_;
    volatile uint64_t offset_us_64_;
    volatile bool has_synced_;
    alignas(uint32_t) volatile uint8_t sync_data_[4];

    friend class HarpCore;
};

#endif // HARP_SYNCHRONIZER_H
