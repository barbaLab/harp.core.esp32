/**
 * @file  bb_loadcell.h
 * @brief Four-channel parallel HX711 acquisition for the behaviour box.
 *
 * Drop-in component for harp.core.esp32s3 under firmware/inc/.
 * Mirrors the firmware/inc + firmware/src layout of harp-tech/core.pico.
 *
 * Usage summary
 * -------------
 *   bb::LoadCellArray lc;
 *   ESP_ERROR_CHECK(lc.init(cfg));
 *   ESP_ERROR_CHECK(lc.tare());
 *   ESP_ERROR_CHECK(lc.start());
 *
 *   bb::LoadCellSample s;
 *   if (lc.getSample(s, pdMS_TO_TICKS(50))) { ... }
 *
 * The acquisition task runs on a dedicated core (default: APP_CPU core 1) and
 * notifies itself via a GPIO NEGEDGE ISR on each DOUT line. Timestamps use
 * esp_timer_get_time() (64-bit µs, monotonic).
 *
 * tare() must be called while acquisition is STOPPED (i.e., before start() or
 * after stop()).  Calling it while running returns ESP_ERR_INVALID_STATE.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace bb {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Number of HX711 channels read in parallel.
static constexpr size_t LoadCellCount = 4;

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/**
 * @brief One timestamped sample from all four load cells.
 *
 * All quantities are in grams (force_g, total_g) or millimetres (cop_*).
 * cop_x_mm / cop_y_mm are only meaningful when total_g > min_valid_total_g.
 */
struct LoadCellSample {
    int64_t t_us = 0;                          ///< esp_timer_get_time() at read start
    int32_t raw[LoadCellCount] = {};           ///< 24-bit sign-extended ADC counts (after tare offset subtracted in compute)
    float   force_g[LoadCellCount] = {};       ///< Calibrated, optionally filtered force per cell [g]
    float   total_g = 0.f;                     ///< Sum of all force_g values [g]
    float   cop_x_mm = 0.f;                    ///< Centre-of-pressure X [mm]
    float   cop_y_mm = 0.f;                    ///< Centre-of-pressure Y [mm]
    uint8_t valid_mask = 0;                    ///< Bitmask: bit i set when cell i produced a valid reading
    bool    occupied = false;                  ///< true when total_g >= occupancy_threshold_g
};

/**
 * @brief Configuration for LoadCellArray.
 *
 * All fields have sensible defaults; override only what differs in your PCB.
 */
struct LoadCellConfig {
    // GPIO mapping -----------------------------------------------------------
    gpio_num_t dout[LoadCellCount] = {
        GPIO_NUM_4,
        GPIO_NUM_5,
        GPIO_NUM_6,
        GPIO_NUM_7,
    };
    gpio_num_t sck = GPIO_NUM_15;

    // Calibration ------------------------------------------------------------
    float   scale_g_per_count[LoadCellCount] = {1.f, 1.f, 1.f, 1.f};
    int32_t offset[LoadCellCount] = {};        ///< Set by tare(); or provide pre-calibrated values

    // Platform geometry (used for CoP calculation) ---------------------------
    /// X position of each cell on the platform [mm], positive = right
    float pos_x_mm[LoadCellCount] = { -160.f,  160.f,  160.f,  -160.f};
    /// Y position of each cell on the platform [mm], positive = front
    float pos_y_mm[LoadCellCount] = {  160.f,  160.f, -160.f, -160.f};

    // Signal processing ------------------------------------------------------
    float occupancy_threshold_g = 5.f;  ///< Minimum total force to declare platform occupied
    float min_valid_total_g     = 1.f;  ///< Minimum total force for meaningful CoP computation
    /// IIR low-pass coefficient: 1.0 = no filtering, 0.1 = heavy smoothing
    float filter_alpha          = 1.f;

    // FreeRTOS task ----------------------------------------------------------
    UBaseType_t  task_priority    = 10;
    uint32_t     task_stack_words = 4096;  ///< Stack depth in WORDS (not bytes)
    BaseType_t   task_core        = APP_CPU_NUM;
    TickType_t   notify_timeout_ticks = pdMS_TO_TICKS(20);
    size_t       queue_length     = 4;
};

// ---------------------------------------------------------------------------
// LoadCellArray
// ---------------------------------------------------------------------------

class LoadCellArray {
public:
    LoadCellArray();
    ~LoadCellArray();

    // Non-copyable / non-movable
    LoadCellArray(const LoadCellArray&)            = delete;
    LoadCellArray& operator=(const LoadCellArray&) = delete;

    /**
     * @brief Initialise GPIOs, install ISR service, create FreeRTOS queue.
     *
     * Must be called exactly once before start() or tare().
     * Returns ESP_ERR_INVALID_STATE if called more than once.
     */
    esp_err_t init(const LoadCellConfig& config);

    /**
     * @brief Launch the acquisition task and attach GPIO ISR handlers.
     *
     * Flushes the queue and resets the IIR filter on every (re-)start.
     */
    esp_err_t start();

    /**
     * @brief Gracefully request the acquisition task to stop, then join it.
     *
     * Blocks up to 500 ms waiting for the task to exit cleanly; force-deletes
     * it if that deadline is exceeded.  Safe to call multiple times.
     */
    esp_err_t stop();

    /// @return true between a successful start() and a subsequent stop().
    bool isRunning() const;

    /**
     * @brief Block until a sample is available in the queue.
     *
     * @param out            Receives the sample.
     * @param timeout_ticks  0 = non-blocking poll.
     * @return true if a sample was dequeued.
     */
    bool getSample(LoadCellSample& out, TickType_t timeout_ticks = 0);

    /**
     * @brief Copy the most recently computed sample without dequeuing it.
     *
     * Safe to call from any task.  Returns false if no sample has been
     * produced yet (i.e., acquisition was never started).
     */
    bool getLatestSample(LoadCellSample& out) const;

    /**
     * @brief Block-read N raw samples, average them, store result in cfg_.offset[].
     *
     * @pre  Acquisition must be STOPPED (returns ESP_ERR_INVALID_STATE otherwise).
     * @pre  Platform must be unloaded during the call.
     * @param sample_count         Number of averages (default 32).
     * @param inter_sample_delay   Delay between reads (default 5 ms).
     */
    esp_err_t tare(size_t     sample_count      = 32,
                   TickType_t inter_sample_delay = pdMS_TO_TICKS(5));

    // Per-cell calibration setters (safe to call at any time) ----------------
    void setScale(size_t index, float scale_g_per_count);
    void setOffset(size_t index, int32_t offset_counts);
    void setOccupancyThreshold(float threshold_g);
    /// Clamps alpha to [0, 1] and resets the filter state.
    void setFilterAlpha(float alpha);

    /// Read-only access to the active configuration.
    const LoadCellConfig& config() const;

private:
    // FreeRTOS glue ----------------------------------------------------------
    static void           taskEntry(void* arg);
    static void IRAM_ATTR doutIsr(void* arg);
    void                  taskLoop();

    // Acquisition helpers ----------------------------------------------------
    bool           allReady() const;
    bool           waitReady(TickType_t timeout_ticks) const;
    bool           readOneRaw(int32_t out[LoadCellCount], TickType_t timeout_ticks);
    void           read4Parallel(int32_t out[LoadCellCount]);
    void           computeSample(const int32_t raw[LoadCellCount],
                                 int64_t t_us,
                                 LoadCellSample& out);
    static int32_t signExtend24(uint32_t v);

    // Internal state management ----------------------------------------------
    void resetFilter();
    void installIsrHandlers();
    void removeIsrHandlers();
    void flushQueue();

    // State ------------------------------------------------------------------
    LoadCellConfig cfg_{};
    TaskHandle_t   task_handle_      = nullptr;
    QueueHandle_t  sample_queue_     = nullptr;

    mutable portMUX_TYPE sample_mux_ = portMUX_INITIALIZER_UNLOCKED;
    portMUX_TYPE         read_mux_   = portMUX_INITIALIZER_UNLOCKED;

    bool          initialized_           = false;
    bool          running_               = false;
    volatile bool stop_requested_        = false;
    bool          isr_handlers_installed_ = false;
    bool          filter_initialized_    = false;

    float          filtered_force_g_[LoadCellCount] = {};
    LoadCellSample latest_sample_{};
};

} // namespace bb