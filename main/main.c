/*
 * main.c
 *
 * Entry point. app_main only starts the subsystems; all real work happens
 * inside them (Section 7.2). At this step it starts the audio source (real ADC
 * capture or the synthetic clap source) and prints a once-per-second report.
 * With the synthetic source it also checks every fake clap: it measures the
 * delays between microphones straight from the ring buffer and compares them
 * with the delays the generator actually used.
 */
#include <math.h>
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
#include "synthetic_clap_source.h"

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
    ESP_LOGI(LOG_TAG, "Sampling: %d Hz per channel (requested %d) x %d channels",
             ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ, SAMPLE_RATE_PER_CHANNEL_HZ, MICROPHONE_COUNT);
    ESP_LOGI(LOG_TAG, "Mic A (%.2f, %.2f) ft  Mic B (%.2f, %.2f) ft  Mic C (%.2f, %.2f) ft",
             MIC_A_X_FT, MIC_A_Y_FT, MIC_B_X_FT, MIC_B_Y_FT, MIC_C_X_FT, MIC_C_Y_FT);
    ESP_LOGI(LOG_TAG, "Event record size: %u bytes", (unsigned)sizeof(localization_event_t));
    ESP_LOGI(LOG_TAG, "Audio source: %s",
             AUDIO_SOURCE_SYNTHETIC ? "SYNTHETIC claps (no microphones)" : "ADC1 microphones");
}

/**
 * Prints one capture report: measured sample rate, plus per-microphone statistics for real capture.
 * Inputs:  none
 * Outputs: log lines only. The measured rate is the samples stored since the
 *          previous report divided by the real time that passed, so it checks
 *          that samples really arrive at ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ.
 */
static void print_capture_report(void)
{
    static uint32_t previous_sample_count = 0;
    static int64_t previous_time_us = 0;

    uint32_t current_sample_count = audio_ring_buffer_get_sample_count();
    int64_t current_time_us = esp_timer_get_time();
#if !AUDIO_SOURCE_SYNTHETIC
    audio_capture_statistics_t statistics;
    audio_capture_read_and_reset_statistics(&statistics);
#endif

    if (previous_time_us != 0) {
        float elapsed_seconds = (float)(current_time_us - previous_time_us) / 1.0e6f;
        float measured_rate_hz = (float)(uint32_t)(current_sample_count - previous_sample_count) / elapsed_seconds;
#if AUDIO_SOURCE_SYNTHETIC
        ESP_LOGI(LOG_TAG, "rate %.0f Hz/channel (expected %d) | longest generator cycle %lu us (budget %d)",
                 measured_rate_hz, ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ,
                 (unsigned long)synthetic_clap_source_read_and_reset_longest_cycle_us(),
                 SYNTHETIC_BLOCK_INTERVAL_MS * 1000);
#else
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
#endif
    }
    previous_sample_count = current_sample_count;
    previous_time_us = current_time_us;
}

#if AUDIO_SOURCE_SYNTHETIC
/* ------------------------------------------------------------------
 * Synthetic clap check (temporary, Step 1b). Copies a window around each
 * fake clap and finds each pair's delay by plain cross-correlation: slide one
 * signal past the other and find the shift where they match best.
 * ------------------------------------------------------------------ */

#define CHECK_PREROLL_SAMPLES       64    /* window starts this many samples before the emission */
#define CHECK_WINDOW_SAMPLES        768   /* about 31 ms                                          */
#define CHECK_MAXIMUM_LAG_SAMPLES   72    /* 3 ft / 1125 ft/s = 65.3 samples, plus skew and margin */
#define CHECK_TOLERANCE_SAMPLES     0.5f  /* about 20 us                                          */

static int16_t check_window_raw[MICROPHONE_COUNT][CHECK_WINDOW_SAMPLES];
static float   check_window_centered[MICROPHONE_COUNT][CHECK_WINDOW_SAMPLES];

/**
 * Finds how many samples later an event appears in one signal than in another.
 * Inputs:  first_signal, second_signal - CHECK_WINDOW_SAMPLES values each, mean removed
 * Outputs: the shift L (fractional) for which first_signal[n] best matches
 *          second_signal[n - L]. Positive L means first_signal hears it later.
 */
static float measure_delay_samples(const float *first_signal, const float *second_signal)
{
    float correlation[2 * CHECK_MAXIMUM_LAG_SAMPLES + 1];
    int best_lag = 0;
    float best_value = -INFINITY;

    for (int lag = -CHECK_MAXIMUM_LAG_SAMPLES; lag <= CHECK_MAXIMUM_LAG_SAMPLES; lag++) {
        float sum = 0.0f;
        for (int index = CHECK_MAXIMUM_LAG_SAMPLES; index < CHECK_WINDOW_SAMPLES - CHECK_MAXIMUM_LAG_SAMPLES; index++) {
            sum += first_signal[index] * second_signal[index - lag];
        }
        correlation[lag + CHECK_MAXIMUM_LAG_SAMPLES] = sum;
        if (sum > best_value) {
            best_value = sum;
            best_lag = lag;
        }
    }

    if (best_lag == -CHECK_MAXIMUM_LAG_SAMPLES || best_lag == CHECK_MAXIMUM_LAG_SAMPLES) {
        return (float)best_lag;
    }

    /* Fit a parabola through the peak and its two neighbours; its top lies
     * between samples, which gives a fraction-of-a-sample answer. */
    float left = correlation[best_lag + CHECK_MAXIMUM_LAG_SAMPLES - 1];
    float center = correlation[best_lag + CHECK_MAXIMUM_LAG_SAMPLES];
    float right = correlation[best_lag + CHECK_MAXIMUM_LAG_SAMPLES + 1];
    float curvature = left - 2.0f * center + right;
    float fraction = curvature != 0.0f ? 0.5f * (left - right) / curvature : 0.0f;
    return (float)best_lag + fraction;
}

/**
 * Prints a synthetic clap's ground truth and checks the delays measured from the ring buffer against it.
 * Inputs:  truth - what the generator played
 * Outputs: log lines with expected vs measured delay for each pair, and PASS/FAIL
 */
static void check_synthetic_clap(const synthetic_clap_truth_t *truth)
{
    static const char microphone_name[MICROPHONE_COUNT] = { 'A', 'B', 'C' };

    ESP_LOGI(LOG_TAG, "SYNTH #%lu: %s at (%.2f, %.2f) ft, %s",
             (unsigned long)truth->clap_number, truth->description, truth->x_ft, truth->y_ft,
             truth->is_inside_area ? "inside area" : "OUTSIDE area (must be rejected)");
    ESP_LOGI(LOG_TAG, "  true delays: AB %+7.1f us   BC %+7.1f us   CA %+7.1f us",
             truth->delay_ab_us, truth->delay_bc_us, truth->delay_ca_us);

    if (!truth->is_clap) {
        ESP_LOGI(LOG_TAG, "  not a clap: a pure tone repeats every cycle, so it has no single delay. Step 2 must ignore it.");
        return;
    }

    uint32_t first_sample_number = truth->emission_sample_number - CHECK_PREROLL_SAMPLES;
    for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++) {
        ring_buffer_status_t status = audio_ring_buffer_copy_samples(
            microphone_index, first_sample_number, CHECK_WINDOW_SAMPLES, check_window_raw[microphone_index]);
        if (status != RING_BUFFER_OK) {
            ESP_LOGW(LOG_TAG, "  window for mic %c not available (status %d)", microphone_name[microphone_index], status);
            return;
        }

        float sum = 0.0f;
        for (int index = 0; index < CHECK_WINDOW_SAMPLES; index++) {
            sum += check_window_raw[microphone_index][index];
        }
        float mean = sum / CHECK_WINDOW_SAMPLES;
        for (int index = 0; index < CHECK_WINDOW_SAMPLES; index++) {
            check_window_centered[microphone_index][index] = check_window_raw[microphone_index][index] - mean;
        }
    }

    /* Pairs in the order AB, BC, CA, matching the delay names. */
    static const int pair_first[MICROPHONE_COUNT]  = { 0, 1, 2 };
    static const int pair_second[MICROPHONE_COUNT] = { 1, 2, 0 };
    const float true_delay_us[MICROPHONE_COUNT] = { truth->delay_ab_us, truth->delay_bc_us, truth->delay_ca_us };

    for (int pair = 0; pair < MICROPHONE_COUNT; pair++) {
        int first = pair_first[pair];
        int second = pair_second[pair];

        /* What the raw samples should show: the true delay in samples, plus the
         * multiplexing skew. A later-sampled microphone catches the sound up to
         * one sample "earlier" in its own sample numbering. */
        float skew_samples = (float)(second - first) / (float)MICROPHONE_COUNT;
        float expected_samples = true_delay_us[pair] * 1.0e-6f * ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ + skew_samples;
        float measured_samples = measure_delay_samples(check_window_centered[first], check_window_centered[second]);
        float error_samples = measured_samples - expected_samples;

        ESP_LOGI(LOG_TAG, "  %c%c: expected %+7.2f samples (skew %+.2f)  measured %+7.2f  error %+5.2f  %s",
                 microphone_name[first], microphone_name[second], expected_samples, skew_samples,
                 measured_samples, error_samples,
                 fabsf(error_samples) <= CHECK_TOLERANCE_SAMPLES ? "PASS" : "FAIL");
    }
}

/**
 * Runs the clap check once for each new synthetic clap, as soon as its whole window is recorded.
 * Inputs:  none
 * Outputs: none (calls check_synthetic_clap, which logs)
 */
static void check_for_new_synthetic_clap(void)
{
    static uint32_t last_checked_clap_number = 0;

    synthetic_clap_truth_t truth;
    if (!synthetic_clap_source_get_last_truth(&truth) || truth.clap_number == last_checked_clap_number) {
        return;
    }

    uint32_t window_end_sample_number = truth.emission_sample_number - CHECK_PREROLL_SAMPLES + CHECK_WINDOW_SAMPLES;
    if ((int32_t)(audio_ring_buffer_get_sample_count() - window_end_sample_number) < 0) {
        return;  /* not fully recorded yet; try again next loop */
    }

    last_checked_clap_number = truth.clap_number;
    check_synthetic_clap(&truth);
}
#endif

void app_main(void)
{
    print_startup_summary();

#if AUDIO_SOURCE_SYNTHETIC
    synthetic_clap_source_start();
#else
    audio_capture_start();
#endif

    int64_t next_report_time_us = esp_timer_get_time() + CAPTURE_REPORT_INTERVAL_MS * 1000LL;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_INTERVAL_MS));
#if AUDIO_SOURCE_SYNTHETIC
        check_for_new_synthetic_clap();
#endif
        if (esp_timer_get_time() >= next_report_time_us) {
            print_capture_report();
            next_report_time_us += CAPTURE_REPORT_INTERVAL_MS * 1000LL;
        }
    }
}
