/**
 * @file  bb_loadcell.cpp
 * @brief Implementation of bb::LoadCellArray.
 *        See inc/bb_loadcell.h for the public API.
 */

#include "bb_loadcell.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

namespace bb {

namespace {
    constexpr char      TAG[]            = "bb_loadcell";
    constexpr TickType_t kStopJoinTimeout = pdMS_TO_TICKS(500);
} // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

LoadCellArray::LoadCellArray() = default;

LoadCellArray::~LoadCellArray()
{
    stop();
    if (sample_queue_ != nullptr) {
        vQueueDelete(sample_queue_);
        sample_queue_ = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

esp_err_t LoadCellArray::init(const LoadCellConfig& config)
{
    if (initialized_) {
        ESP_LOGE(TAG, "init() called more than once");
        return ESP_ERR_INVALID_STATE;
    }

    cfg_ = config;

    // Configure SCK as push-pull output, initially low
    gpio_config_t sck_cfg = {};
    sck_cfg.pin_bit_mask  = 1ULL << cfg_.sck;
    sck_cfg.mode          = GPIO_MODE_OUTPUT;
    sck_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
    sck_cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    sck_cfg.intr_type     = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&sck_cfg),       TAG, "SCK config failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(cfg_.sck, 0), TAG, "SCK low failed");

    // Configure all DOUT pins as inputs with falling-edge interrupt
    uint64_t dout_mask = 0;
    for (size_t i = 0; i < LoadCellCount; ++i) {
        dout_mask |= 1ULL << cfg_.dout[i];
    }
    gpio_config_t dout_cfg = {};
    dout_cfg.pin_bit_mask  = dout_mask;
    dout_cfg.mode          = GPIO_MODE_INPUT;
    dout_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
    dout_cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    dout_cfg.intr_type     = GPIO_INTR_NEGEDGE;
    ESP_RETURN_ON_ERROR(gpio_config(&dout_cfg), TAG, "DOUT config failed");

    // Create sample queue
    sample_queue_ = xQueueCreate(cfg_.queue_length, sizeof(LoadCellSample));
    if (sample_queue_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // Install global GPIO ISR service (no-op if already installed)
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return err;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "Initialised — SCK=GPIO%d, DOUT=[%d,%d,%d,%d]",
             cfg_.sck,
             cfg_.dout[0], cfg_.dout[1], cfg_.dout[2], cfg_.dout[3]);
    return ESP_OK;
}

esp_err_t LoadCellArray::start()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (running_) {
        ESP_LOGW(TAG, "start() called while already running — ignored");
        return ESP_OK;
    }

    stop_requested_ = false;
    resetFilter();
    flushQueue();

    BaseType_t ok = xTaskCreatePinnedToCore(
        &LoadCellArray::taskEntry,
        TAG,
        cfg_.task_stack_words,
        this,
        cfg_.task_priority,
        &task_handle_,
        cfg_.task_core);

    if (ok != pdPASS) {
        task_handle_ = nullptr;
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        return ESP_ERR_NO_MEM;
    }

    installIsrHandlers();
    running_ = true;
    ESP_LOGI(TAG, "Acquisition started (core %d, priority %d)",
             cfg_.task_core, cfg_.task_priority);
    return ESP_OK;
}

esp_err_t LoadCellArray::stop()
{
    if (!running_ && task_handle_ == nullptr) {
        return ESP_OK;
    }

    stop_requested_ = true;
    removeIsrHandlers();

    if (task_handle_ != nullptr) {
        // Unblock the task so it can see stop_requested_
        xTaskNotifyGive(task_handle_);

        TickType_t deadline = xTaskGetTickCount() + kStopJoinTimeout;
        while (task_handle_ != nullptr && xTaskGetTickCount() < deadline) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        if (task_handle_ != nullptr) {
            ESP_LOGW(TAG, "Task did not exit cleanly — force-deleting");
            vTaskDelete(task_handle_);
            task_handle_ = nullptr;
        }
    }

    running_        = false;
    stop_requested_ = false;
    ESP_LOGI(TAG, "Acquisition stopped");
    return ESP_OK;
}

bool LoadCellArray::isRunning() const
{
    return running_;
}

// ---------------------------------------------------------------------------
// Sample retrieval
// ---------------------------------------------------------------------------

bool LoadCellArray::getSample(LoadCellSample& out, TickType_t timeout_ticks)
{
    if (sample_queue_ == nullptr) {
        return false;
    }
    return xQueueReceive(sample_queue_, &out, timeout_ticks) == pdTRUE;
}

bool LoadCellArray::getLatestSample(LoadCellSample& out) const
{
    portENTER_CRITICAL(&sample_mux_);
    out = latest_sample_;
    const bool valid = (latest_sample_.t_us != 0);
    portEXIT_CRITICAL(&sample_mux_);
    return valid;
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

esp_err_t LoadCellArray::tare(size_t sample_count, TickType_t inter_sample_delay)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (running_) {
        ESP_LOGE(TAG, "tare() must be called while acquisition is stopped");
        return ESP_ERR_INVALID_STATE;
    }
    if (sample_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Taring: averaging %u samples…", (unsigned)sample_count);

    int64_t accum[LoadCellCount] = {};
    int32_t raw[LoadCellCount]   = {};

    for (size_t n = 0; n < sample_count; ++n) {
        if (!readOneRaw(raw, pdMS_TO_TICKS(250))) {
            ESP_LOGE(TAG, "tare: timeout waiting for sample %u", (unsigned)n);
            return ESP_ERR_TIMEOUT;
        }
        for (size_t i = 0; i < LoadCellCount; ++i) {
            accum[i] += raw[i];
        }
        if (inter_sample_delay > 0) {
            vTaskDelay(inter_sample_delay);
        }
    }

    for (size_t i = 0; i < LoadCellCount; ++i) {
        cfg_.offset[i] = static_cast<int32_t>(
            accum[i] / static_cast<int64_t>(sample_count));
    }

    resetFilter();
    ESP_LOGI(TAG, "Tare done — offsets: [%ld, %ld, %ld, %ld]",
             (long)cfg_.offset[0], (long)cfg_.offset[1],
             (long)cfg_.offset[2], (long)cfg_.offset[3]);
    return ESP_OK;
}

void LoadCellArray::setScale(size_t index, float scale_g_per_count)
{
    if (index < LoadCellCount) {
        cfg_.scale_g_per_count[index] = scale_g_per_count;
    }
}

void LoadCellArray::setOffset(size_t index, int32_t offset_counts)
{
    if (index < LoadCellCount) {
        cfg_.offset[index] = offset_counts;
    }
}

void LoadCellArray::setOccupancyThreshold(float threshold_g)
{
    cfg_.occupancy_threshold_g = threshold_g;
}

void LoadCellArray::setFilterAlpha(float alpha)
{
    if (alpha < 0.f) alpha = 0.f;
    if (alpha > 1.f) alpha = 1.f;
    cfg_.filter_alpha = alpha;
    resetFilter();
}

const LoadCellConfig& LoadCellArray::config() const
{
    return cfg_;
}

// ---------------------------------------------------------------------------
// FreeRTOS task
// ---------------------------------------------------------------------------

void LoadCellArray::taskEntry(void* arg)
{
    static_cast<LoadCellArray*>(arg)->taskLoop();
}

void IRAM_ATTR LoadCellArray::doutIsr(void* arg)
{
    auto* self = static_cast<LoadCellArray*>(arg);
    if (self == nullptr || self->task_handle_ == nullptr) {
        return;
    }
    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR(self->task_handle_, &hp_woken);
    portYIELD_FROM_ISR_IF(hp_woken);
}

void LoadCellArray::taskLoop()
{
    int32_t        raw[LoadCellCount] = {};
    LoadCellSample sample{};

    while (!stop_requested_) {
        // Wait until all DOUT lines are low (new sample ready)
        while (!stop_requested_ && !allReady()) {
            ulTaskNotifyTake(pdTRUE, cfg_.notify_timeout_ticks);
        }
        if (stop_requested_) {
            break;
        }

        const int64_t t_us = esp_timer_get_time();
        read4Parallel(raw);
        computeSample(raw, t_us, sample);

        // Update latest-sample cache
        portENTER_CRITICAL(&sample_mux_);
        latest_sample_ = sample;
        portEXIT_CRITICAL(&sample_mux_);

        // Push to queue — drop oldest if full, never block
        if (sample_queue_ != nullptr) {
            if (cfg_.queue_length == 1) {
                xQueueOverwrite(sample_queue_, &sample);
            } else {
                if (xQueueSend(sample_queue_, &sample, 0) != pdTRUE) {
                    LoadCellSample discard{};
                    xQueueReceive(sample_queue_, &discard, 0);
                    xQueueSend(sample_queue_, &sample, 0);
                }
            }
        }
    }

    running_     = false;
    task_handle_ = nullptr;
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Low-level HX711 read
// ---------------------------------------------------------------------------

bool LoadCellArray::allReady() const
{
    for (size_t i = 0; i < LoadCellCount; ++i) {
        if (gpio_get_level(cfg_.dout[i]) != 0) {
            return false;
        }
    }
    return true;
}

bool LoadCellArray::waitReady(TickType_t timeout_ticks) const
{
    const TickType_t deadline = xTaskGetTickCount() + timeout_ticks;
    while (!allReady()) {
        if (xTaskGetTickCount() >= deadline) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

bool LoadCellArray::readOneRaw(int32_t out[LoadCellCount], TickType_t timeout_ticks)
{
    if (!waitReady(timeout_ticks)) {
        return false;
    }
    read4Parallel(out);
    return true;
}

/*
 * Bit-bang 24-bit parallel read from all four HX711s simultaneously.
 * The SCK line is shared; DOUT pins are sampled after each rising edge.
 *
 * HX711 timing constraints (datasheet):
 *   PD_SCK high width  ≥ 200 ns  → 1 µs safe at any ESP32 CPU speed
 *   PD_SCK low width   ≥ 200 ns
 *   DOUT valid after falling edge ≤ 100 ns (well within the 1 µs low phase)
 *   PD_SCK must NOT be held high > 60 µs or the chip enters power-down.
 *
 * Critical section prevents FreeRTOS preemption mid-read.
 * Total read time: 24 bits × 2 µs + 1 gain pulse × 2 µs ≈ 50 µs < 60 µs limit.
 */
void LoadCellArray::read4Parallel(int32_t out[LoadCellCount])
{
    uint32_t raw[LoadCellCount] = {};

    portENTER_CRITICAL(&read_mux_);

    for (int bit = 0; bit < 24; ++bit) {
        gpio_set_level(cfg_.sck, 1);
        esp_rom_delay_us(1);

        for (size_t i = 0; i < LoadCellCount; ++i) {
            raw[i] = (raw[i] << 1) | static_cast<uint32_t>(gpio_get_level(cfg_.dout[i]));
        }

        gpio_set_level(cfg_.sck, 0);
        esp_rom_delay_us(1);
    }

    // 25th pulse → gain = 128 for next conversion (HX711 default channel A)
    gpio_set_level(cfg_.sck, 1);
    esp_rom_delay_us(1);
    gpio_set_level(cfg_.sck, 0);
    esp_rom_delay_us(1);

    portEXIT_CRITICAL(&read_mux_);

    for (size_t i = 0; i < LoadCellCount; ++i) {
        out[i] = signExtend24(raw[i]);
    }
}

int32_t LoadCellArray::signExtend24(uint32_t v)
{
    if ((v & 0x800000U) != 0U) {
        v |= 0xFF000000U;
    }
    return static_cast<int32_t>(v);
}

// ---------------------------------------------------------------------------
// Signal processing
// ---------------------------------------------------------------------------

void LoadCellArray::computeSample(
    const int32_t raw[LoadCellCount],
    int64_t       t_us,
    LoadCellSample& out)
{
    memset(&out, 0, sizeof(out));
    out.t_us = t_us;

    float weighted_x = 0.f;
    float weighted_y = 0.f;

    for (size_t i = 0; i < LoadCellCount; ++i) {
        out.raw[i] = raw[i];

        const float force_raw = cfg_.scale_g_per_count[i]
                              * static_cast<float>(raw[i] - cfg_.offset[i]);

        // First-order IIR: y[n] = α·x[n] + (1−α)·y[n−1]
        if (!filter_initialized_ || cfg_.filter_alpha >= 1.f) {
            filtered_force_g_[i] = force_raw;
        } else {
            filtered_force_g_[i] = cfg_.filter_alpha         * force_raw
                                 + (1.f - cfg_.filter_alpha) * filtered_force_g_[i];
        }

        out.force_g[i] = filtered_force_g_[i];
        out.total_g    += out.force_g[i];
        weighted_x     += out.force_g[i] * cfg_.pos_x_mm[i];
        weighted_y     += out.force_g[i] * cfg_.pos_y_mm[i];
        out.valid_mask |= static_cast<uint8_t>(1u << i);
    }

    filter_initialized_ = true;

    if (out.total_g > cfg_.min_valid_total_g) {
        out.cop_x_mm = weighted_x / out.total_g;
        out.cop_y_mm = weighted_y / out.total_g;
    }

    out.occupied = (out.total_g >= cfg_.occupancy_threshold_g);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void LoadCellArray::resetFilter()
{
    for (size_t i = 0; i < LoadCellCount; ++i) {
        filtered_force_g_[i] = 0.f;
    }
    filter_initialized_ = false;
}

void LoadCellArray::installIsrHandlers()
{
    if (isr_handlers_installed_) {
        return;
    }
    for (size_t i = 0; i < LoadCellCount; ++i) {
        gpio_isr_handler_add(cfg_.dout[i], &LoadCellArray::doutIsr, this);
    }
    isr_handlers_installed_ = true;
}

void LoadCellArray::removeIsrHandlers()
{
    if (!isr_handlers_installed_) {
        return;
    }
    for (size_t i = 0; i < LoadCellCount; ++i) {
        gpio_isr_handler_remove(cfg_.dout[i]);
    }
    isr_handlers_installed_ = false;
}

void LoadCellArray::flushQueue()
{
    if (sample_queue_ != nullptr) {
        xQueueReset(sample_queue_);
    }
    portENTER_CRITICAL(&sample_mux_);
    latest_sample_ = LoadCellSample{};
    portEXIT_CRITICAL(&sample_mux_);
}

} // namespace bb