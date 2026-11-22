/*
 * audio_capture.c
 *
 * How the data flows:
 *
 *   ADC1 hardware --(DMA)--> frame of 288 results --(interrupt)--> task wakes
 *   task reads results --> sorts them by channel --> ring buffer
 *
 * ADC1 has one converter that visits our three pins in a fixed pattern
 * (A, B, C, A, B, C, ...). Each result carries the channel it came from, so we
 * rebuild whole "frames" of one sample per microphone and store each frame.
 */
#include "audio_capture.h"

#include <limits.h>
#include <string.h>
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_ring_buffer.h"

static const char *LOG_TAG = "audio_capture";

/* ADC channel for each microphone, indexed by microphone number (A=0, B=1, C=2). */
static const uint8_t microphone_adc_channel[MICROPHONE_COUNT] = 
{
    MIC_A_ADC_CHANNEL, MIC_B_ADC_CHANNEL, MIC_C_ADC_CHANNEL,
};

static adc_continuous_handle_t adc_handle;
static TaskHandle_t capture_task_handle;

/* Statistics shared between the capture task (writer) and any reader task.
 * The spinlock keeps the two from touching the numbers at the same moment. */
static portMUX_TYPE statistics_lock = portMUX_INITIALIZER_UNLOCKED;
static int32_t  interval_sum_raw[MICROPHONE_COUNT];
static int16_t  interval_minimum_raw[MICROPHONE_COUNT];
static int16_t  interval_maximum_raw[MICROPHONE_COUNT];
static uint32_t interval_sample_count;
static uint32_t out_of_order_result_total;
static uint32_t pool_overflow_total;

/**
 * Restarts the interval statistics (sums, extremes, counts).
 * Inputs:  none
 * Outputs: none. Caller must hold statistics_lock.
 */
static void reset_interval_statistics_locked(void)
{
    for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++)
    {
        interval_sum_raw[microphone_index] = 0;
        interval_minimum_raw[microphone_index] = INT16_MAX;
        interval_maximum_raw[microphone_index] = INT16_MIN;
    }
    interval_sample_count = 0;
}

/**
 * Finds which microphone a conversion result belongs to.
 * Inputs:  adc_channel - ADC1 channel number reported with the result
 * Outputs: microphone index 0-2, or -1 if the channel is not one of ours
 */
static int microphone_index_for_adc_channel(uint32_t adc_channel)
{
    for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++)
    {
        if (microphone_adc_channel[microphone_index] == adc_channel)
        {
            return microphone_index;
        }
    }
    return -1;
}

/**
 * Runs in interrupt context each time the ADC finishes a DMA frame; wakes the capture task.
 * Inputs:  handle, event_data, user_data - supplied by the ADC driver (unused)
 * Outputs: true if a higher-priority task was woken (so the system switches to it now)
 */
static bool on_conversion_frame_done(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *event_data, void *user_data)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(capture_task_handle, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

/**
 * Configures ADC1 to sample the three microphone pins continuously with DMA, and starts it.
 * Inputs:  none
 * Outputs: true on success, false if any ADC driver call failed (logged)
 */
static bool start_adc_sampling(void)
{
    adc_continuous_handle_cfg_t handle_config = 
    {
        .max_store_buf_size = ADC_POOL_BYTES,
        .conv_frame_size = ADC_FRAME_BYTES,
    };
    if (adc_continuous_new_handle(&handle_config, &adc_handle) != ESP_OK) 
    {
        ESP_LOGE(LOG_TAG, "adc_continuous_new_handle failed");
        return false;
    }

    /* The "pattern" is the list of channels the converter visits, in order. */
    adc_digi_pattern_config_t pattern[MICROPHONE_COUNT];
    for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++)
    {
        pattern[microphone_index] = (adc_digi_pattern_config_t)
        {
            .atten = ADC_ATTEN_DB_12,
            .channel = microphone_adc_channel[microphone_index],
            .unit = ADC_UNIT_1,
            .bit_width = ADC_BITWIDTH_12,
        };
    }

    adc_continuous_config_t sampling_config = 
    {
        .pattern_num = MICROPHONE_COUNT,
        .adc_pattern = pattern,
        .sample_freq_hz = ADC_CONVERSION_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    if (adc_continuous_config(adc_handle, &sampling_config) != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "adc_continuous_config failed");
        return false;
    }

    adc_continuous_evt_cbs_t callbacks = { .on_conv_done = on_conversion_frame_done };

    if (adc_continuous_register_event_callbacks(adc_handle, &callbacks, NULL) != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "adc_continuous_register_event_callbacks failed");
        return false;
    }

    if (adc_continuous_start(adc_handle) != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "adc_continuous_start failed");
        return false;
    }
    return true;
}

/**
 * Turns a block of raw ADC results into whole frames, stores them, and updates the statistics.
 * Inputs:  result_bytes - the bytes read from the ADC driver
 *          result_byte_count - how many bytes are valid
 * Outputs: none. Each complete A,B,C set is written to the ring buffer; a result
 *          out of order is counted and the frame being built restarts at A.
 */
static void process_adc_results(const uint8_t *result_bytes, uint32_t result_byte_count)
{
    /* Frame being assembled across calls: which microphone we expect next. */
    static int next_expected_microphone = 0;
    static int16_t frame_under_construction[MICROPHONE_COUNT];

    int32_t  block_sum_raw[MICROPHONE_COUNT] = {0};
    int16_t  block_minimum_raw[MICROPHONE_COUNT];
    int16_t  block_maximum_raw[MICROPHONE_COUNT];
    uint32_t block_sample_count = 0;
    uint32_t block_out_of_order_count = 0;

    for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++)
    {
        block_minimum_raw[microphone_index] = INT16_MAX;
        block_maximum_raw[microphone_index] = INT16_MIN;
    }

    for (uint32_t byte_offset = 0; byte_offset + ADC_RESULT_BYTES <= result_byte_count; byte_offset += ADC_RESULT_BYTES)
    {
        const adc_digi_output_data_t *result = (const adc_digi_output_data_t *)&result_bytes[byte_offset];
        int microphone_index = microphone_index_for_adc_channel(result->type2.channel);

        if (microphone_index < 0)
        {
            block_out_of_order_count++;
            next_expected_microphone = 0;
            continue;
        }

        if (microphone_index != next_expected_microphone)
        {
            /* A result went missing. Throw away the half-built frame; if this
             * result is microphone A it can start a fresh one. */
            block_out_of_order_count++;
            next_expected_microphone = 0;
            if (microphone_index != 0)
            {
                continue;
            }
        }

        frame_under_construction[microphone_index] = (int16_t)result->type2.data;
        next_expected_microphone++;

        if (next_expected_microphone == MICROPHONE_COUNT)
        {
            audio_ring_buffer_write_frame(frame_under_construction);
            for (int index = 0; index < MICROPHONE_COUNT; index++)
            {
                int16_t raw_value = frame_under_construction[index];
                block_sum_raw[index] += raw_value;
                if (raw_value < block_minimum_raw[index]) block_minimum_raw[index] = raw_value;
                if (raw_value > block_maximum_raw[index]) block_maximum_raw[index] = raw_value;
            }
            block_sample_count++;
            next_expected_microphone = 0;
        }
    }

    /* Merge the block into the shared totals once, to keep the lock short. */
    portENTER_CRITICAL(&statistics_lock);
    for (int index = 0; index < MICROPHONE_COUNT; index++)
    {
        interval_sum_raw[index] += block_sum_raw[index];
        if (block_minimum_raw[index] < interval_minimum_raw[index]) interval_minimum_raw[index] = block_minimum_raw[index];
        if (block_maximum_raw[index] > interval_maximum_raw[index]) interval_maximum_raw[index] = block_maximum_raw[index];
    }
    interval_sample_count += block_sample_count;
    out_of_order_result_total += block_out_of_order_count;
    portEXIT_CRITICAL(&statistics_lock);
}

/**
 * Task body: sets up the ADC, then forever waits for DMA frames and stores them.
 * Inputs:  task_argument - unused
 * Outputs: none (never returns unless the ADC fails to start)
 */
static void audio_capture_task(void *task_argument)
{
    /* The interrupt callback needs this handle to wake us. Taking it from
     * inside the task avoids a race with xTaskCreate storing it. */
    capture_task_handle = xTaskGetCurrentTaskHandle();

    /* The ADC interrupt is attached to the core that creates the driver, so
     * setup happens here on Core 1 rather than in app_main on Core 0. */
    if (!start_adc_sampling())
    {
        ESP_LOGE(LOG_TAG, "ADC start failed; capture task stopping");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(LOG_TAG, "Sampling started: %d conversions/s, %d bytes per DMA frame",
             ADC_CONVERSION_RATE_HZ, ADC_FRAME_BYTES);

    static uint8_t read_buffer[ADC_FRAME_BYTES];

    while (true)
    {
        /* Sleep until the interrupt says a frame is ready (or 100 ms passes, as a safety net). */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        /* Drain everything waiting, in case more than one frame arrived. */
        esp_err_t read_result;
        do
        {
            uint32_t bytes_read = 0;
            read_result = adc_continuous_read(adc_handle, read_buffer, sizeof(read_buffer), &bytes_read, 0);
            if (read_result == ESP_OK)
            {
                process_adc_results(read_buffer, bytes_read);
            } else if (read_result == ESP_ERR_INVALID_STATE)
            {
                /* The driver's pool overflowed: we fell behind and samples were lost. */
                portENTER_CRITICAL(&statistics_lock);
                pool_overflow_total++;
                portEXIT_CRITICAL(&statistics_lock);
            }
        } while (read_result == ESP_OK);
    }
}

void audio_capture_start(void)
{
    audio_ring_buffer_initialize();
    portENTER_CRITICAL(&statistics_lock);
    reset_interval_statistics_locked();
    portEXIT_CRITICAL(&statistics_lock);

    xTaskCreatePinnedToCore(audio_capture_task, "audio_capture", AUDIO_RX_TASK_STACK_BYTES, NULL, AUDIO_RX_TASK_PRIORITY, &capture_task_handle, AUDIO_RX_TASK_CORE);
}

void audio_capture_read_and_reset_statistics(audio_capture_statistics_t *statistics)
{
    portENTER_CRITICAL(&statistics_lock);
    statistics->sample_count_per_microphone = interval_sample_count;
    for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++) {
        statistics->mean_raw[microphone_index] = interval_sample_count > 0 ? (float)interval_sum_raw[microphone_index] / (float)interval_sample_count : 0.0f;
        statistics->minimum_raw[microphone_index] = interval_minimum_raw[microphone_index];
        statistics->maximum_raw[microphone_index] = interval_maximum_raw[microphone_index];
    }
    statistics->out_of_order_result_count = out_of_order_result_total;
    statistics->pool_overflow_count = pool_overflow_total;
    reset_interval_statistics_locked();
    portEXIT_CRITICAL(&statistics_lock);
}
