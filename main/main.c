/*
 * main.c
 *
 * Entry point. app_main only starts the subsystems; all real work happens
 * inside them (Section 7.2). At this step it starts audio capture and prints a
 * once-per-second report so the capture can be checked before building on it.
 */
#include <stdio.h>
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "app_config.h"
#include "event_types.h"
#include "audio_capture.h"
#include "audio_ring_buffer.h"

static const char *LOG_TAG = "main";

/**
 * Prints the chip, build settings and array configuration to the serial monitor.
 * Inputs:  none
 * Outputs: log lines only
 */
static void print_startup_summary(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size_bytes = 0;
    esp_flash_get_size(NULL, &flash_size_bytes);

    ESP_LOGI(LOG_TAG, "Localizing Optical Microphone - acoustic baseline firmware");
    ESP_LOGI(LOG_TAG, "Chip: ESP32-S3 rev %d, %d cores, CPU %d MHz, flash %lu MB",
             chip_info.revision, chip_info.cores, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
             (unsigned long)(flash_size_bytes / (1024 * 1024)));
    ESP_LOGI(LOG_TAG, "FreeRTOS tick: %d Hz", CONFIG_FREERTOS_HZ);
    ESP_LOGI(LOG_TAG, "Sampling: %d Hz per channel x %d channels = %d conversions/s",
             SAMPLE_RATE_PER_CHANNEL_HZ, MICROPHONE_COUNT, ADC_CONVERSION_RATE_HZ);
    ESP_LOGI(LOG_TAG, "Mic A (%.2f, %.2f) ft  Mic B (%.2f, %.2f) ft  Mic C (%.2f, %.2f) ft",
             MIC_A_X_FT, MIC_A_Y_FT, MIC_B_X_FT, MIC_B_Y_FT, MIC_C_X_FT, MIC_C_Y_FT);
    ESP_LOGI(LOG_TAG, "Event record size: %u bytes", (unsigned)sizeof(localization_event_t));
}

/**
 * Prints one capture report: measured sample rate, per-microphone statistics, and error counts.
 * Inputs:  none
 * Outputs: log lines only. The measured rate is the samples stored since the
 *          previous report divided by the real time that passed, so it checks
 *          that the ADC really runs at ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ.
 */
static void print_capture_report(void)
{
    static uint32_t previous_sample_count = 0;
    static int64_t previous_time_us = 0;

    uint32_t current_sample_count = audio_ring_buffer_get_sample_count();
    int64_t current_time_us = esp_timer_get_time();
    audio_capture_statistics_t statistics;
    audio_capture_read_and_reset_statistics(&statistics);

    if (previous_time_us != 0) {
        float elapsed_seconds = (float)(current_time_us - previous_time_us) / 1.0e6f;
        float measured_rate_hz = (float)(uint32_t)(current_sample_count - previous_sample_count) / elapsed_seconds;
        ESP_LOGI(LOG_TAG, "rate %.0f Hz/channel (expected %d) | out-of-order %lu | overflows %lu",
                 measured_rate_hz, ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ,
                 (unsigned long)statistics.out_of_order_result_count,
                 (unsigned long)statistics.pool_overflow_count);
        static const char microphone_name[MICROPHONE_COUNT] = { 'A', 'B', 'C' };
        for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++) {
            ESP_LOGI(LOG_TAG, "  mic %c: mean %7.1f  min %4d  max %4d",
                     microphone_name[microphone_index], statistics.mean_raw[microphone_index],
                     statistics.minimum_raw[microphone_index], statistics.maximum_raw[microphone_index]);
        }
    }
    previous_sample_count = current_sample_count;
    previous_time_us = current_time_us;
}

void app_main(void)
{
    print_startup_summary();
    audio_capture_start();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(CAPTURE_REPORT_INTERVAL_MS));
        print_capture_report();
    }
}
