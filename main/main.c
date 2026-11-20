/*
 * main.c
 *
 * Entry point. app_main only creates the queues and tasks; all real work
 * happens inside the subsystems (Section 7.2). In Step 0 it just reports the
 * configuration so we can confirm the build settings reached the board.
 */
#include <stdio.h>
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "sdkconfig.h"
#include "app_config.h"
#include "event_types.h"

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

void app_main(void)
{
    print_startup_summary();
}
