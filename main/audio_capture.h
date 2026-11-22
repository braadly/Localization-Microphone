/*
 * audio_capture.h
 *
 * Subsystem 1 (Section 7.3) for the demo hardware: reads the three microphones
 * with the ESP32-S3's internal ADC1 and stores the samples in the audio ring
 * buffer. The rest of the firmware only sees the ring buffer, so this file can
 * later be replaced by the PCM1864 driver without touching anything else.
 */
#pragma once

#include <stdint.h>
#include "app_config.h"

/* Measurements of the capture since the previous call to
 * audio_capture_read_and_reset_statistics(). Raw values are ADC counts, 0-4095. */
typedef struct {
    uint32_t sample_count_per_microphone;
    float    mean_raw[MICROPHONE_COUNT];
    int16_t  minimum_raw[MICROPHONE_COUNT];
    int16_t  maximum_raw[MICROPHONE_COUNT];
    uint32_t out_of_order_result_count;  /* total since boot: results that arrived in the wrong order */
    uint32_t pool_overflow_count;        /* total since boot: times the task fell too far behind       */
} audio_capture_statistics_t;

/**
 * Creates the capture task on Core 1, which sets up the ADC and starts sampling.
 * Inputs:  none
 * Outputs: none. Samples begin appearing in the audio ring buffer; failures
 *          are reported in the log.
 */
void audio_capture_start(void);

/**
 * Returns the capture statistics gathered since the last call, then restarts them.
 * Inputs:  statistics - struct that receives the measurements
 * Outputs: fills statistics. Safe to call from any task.
 */
void audio_capture_read_and_reset_statistics(audio_capture_statistics_t *statistics);
