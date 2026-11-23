/*
 * app_config.h
 *
 * Every tunable number in the firmware lives here, with its units in the name,
 * so the design can be read (and changed) in one place. Each later step adds
 * its own section.
 */
#pragma once

/* ===================================================================
 * Audio capture (demo hardware: ESP32-S3 internal ADC1)
 * =================================================================== */

/* Number of microphones the localization pipeline uses (A, B, C). */
#define MICROPHONE_COUNT                    3

/* Rate at which EACH microphone is sampled. ADC1 has one converter shared by
 * all three inputs, so the converter itself runs at three times this rate.
 * The S3 tops out at 83,333 conversions/s; 72,000 keeps a margin and gives a
 * round 24 kHz per channel (41.67 us per sample = 0.56 in of sound path). */
#define SAMPLE_RATE_PER_CHANNEL_HZ          24000
#define ADC_CONVERSION_RATE_HZ              (SAMPLE_RATE_PER_CHANNEL_HZ * MICROPHONE_COUNT)

/* The ADC's sample clock is divided down from the bus clock, so only certain
 * rates exist and the driver rounds our request to the nearest one. Measured
 * with esp_timer on the real board (Step 1): about 24,500 samples/s per channel.
 * All sample-count to time conversions must use this value, not the request
 * above. Re-measure if the ADC clock settings ever change. */
#define ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ   24500

/* ADC1 channel number for each microphone (ESP32-S3: ADC1_CHn is GPIO n+1).
 * GPIO0/GPIO3 are strapping pins and GPIO4-GPIO9 are reserved for the
 * PCM1864 and its I2C bus on the final board, so they are avoided here. */
#define MIC_A_ADC_CHANNEL                   0   /* GPIO1  */
#define MIC_B_ADC_CHANNEL                   1   /* GPIO2  */
#define MIC_C_ADC_CHANNEL                   9   /* GPIO10 */

/* The ADC is told to attenuate its input by 12 dB so it can read up to about
 * 3.1 V. A 12-bit conversion gives raw values 0..4095; silence sits near the
 * middle (about 2048, roughly 1.65 V) because the preamp biases the mic there. */
#define ADC_RAW_MAXIMUM                     4095

/* The ADC hardware fills a DMA buffer with conversions and raises one
 * interrupt per full "frame". 288 conversions = 96 samples of each microphone
 * = 4 ms of audio, short enough for low latency, long enough to keep the
 * interrupt rate low (250 per second). Each conversion result is 4 bytes. */
#define ADC_CONVERSIONS_PER_FRAME           288
#define ADC_RESULT_BYTES                    4
#define ADC_FRAME_BYTES                     (ADC_CONVERSIONS_PER_FRAME * ADC_RESULT_BYTES)

/* Driver-internal pool that holds frames the task has not read yet (4 frames,
 * 16 ms). If the task falls further behind than this, samples are lost. */
#define ADC_POOL_BYTES                      (4 * ADC_FRAME_BYTES)

/* ===================================================================
 * Audio ring buffer. Holds the most recent audio of every microphone so the
 * pipeline can look back from the moment an onset is detected.
 * 4096 samples = 170 ms at 24 kHz. A power of two lets the buffer wrap with a
 * bit mask instead of a division. (3 channels x 2 bytes x 4096 = 24 KB.)
 * =================================================================== */
#define RING_BUFFER_CAPACITY_SAMPLES        4096

/* How often the serial monitor prints capture statistics. */
#define CAPTURE_REPORT_INTERVAL_MS          1000

/* How often app_main wakes to check for work (reports, synthetic clap checks). */
#define MAIN_LOOP_INTERVAL_MS               50

/* ===================================================================
 * Synthetic clap source (Step 1b). Fake microphones for testing without
 * hardware: noise plus claps from known positions, written into the ring
 * buffer at the real capture rate.
 * =================================================================== */

/* 1 = feed the pipeline synthetic claps (no microphones needed),
 * 0 = real capture from ADC1. */
#define AUDIO_SOURCE_SYNTHETIC              1

/* Silence level (mid-scale, where the preamp biases the mic) and the size of
 * the background noise (standard deviation, in ADC counts). */
#define SYNTHETIC_BASELINE_RAW              2048.0f
#define SYNTHETIC_NOISE_SIGMA_RAW           6.0f

/* Frames are written in blocks every 4 ms: 24,500 x 0.004 = 98 frames,
 * the same block length as the real ADC's DMA frames. */
#define SYNTHETIC_BLOCK_INTERVAL_MS         4
#define SYNTHETIC_BLOCK_SAMPLES             (ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ * SYNTHETIC_BLOCK_INTERVAL_MS / 1000)

/* First sound 1 s after boot, then one every 1.5 s (time to read each result). */
#define SYNTHETIC_FIRST_SCENARIO_DELAY_MS   1000
#define SYNTHETIC_SCENARIO_INTERVAL_MS      1500

/* Fixed seed: every run plays exactly the same audio, so results repeat. */
#define SYNTHETIC_RANDOM_SEED               12345u

/* Wall for the echo scenario: 1 ft left of the sensing area, reflecting half the sound. */
#define SYNTHETIC_WALL_X_FT                 (-1.0f)
#define SYNTHETIC_WALL_REFLECTION           0.5f

/* ===================================================================
 * Array geometry (Figure 2.6.1). Origin = lower-left corner of the
 * sensing area, X to the right, Y up, all distances in feet.
 * Equilateral triangle, 3 ft on a side.
 * =================================================================== */
#define SENSING_AREA_WIDTH_FT               5.0f
#define SENSING_AREA_HEIGHT_FT              5.0f

#define MIC_A_X_FT                          1.0f
#define MIC_A_Y_FT                          3.5f
#define MIC_B_X_FT                          4.0f
#define MIC_B_Y_FT                          3.5f
#define MIC_C_X_FT                          2.5f
#define MIC_C_Y_FT                          0.902f  /* 3.5 - 3*sqrt(3)/2 */

/* Speed of sound in air at about 20 C (343 m/s). */
#define SPEED_OF_SOUND_FT_PER_S             1125.0f

/* ===================================================================
 * FreeRTOS tasks (Table 7.2.4). Core 1 = real-time audio path,
 * Core 0 = state + Wi-Fi (ESP-IDF runs the Wi-Fi stack on Core 0).
 * Higher number = higher priority.
 * =================================================================== */
#define AUDIO_RX_TASK_CORE                  1
#define AUDIO_RX_TASK_PRIORITY              20
#define AUDIO_RX_TASK_STACK_BYTES           4096

#define LOCALIZE_TASK_CORE                  1
#define LOCALIZE_TASK_PRIORITY              15
#define LOCALIZE_TASK_STACK_BYTES           8192

#define STATE_MANAGER_TASK_CORE             0
#define STATE_MANAGER_TASK_PRIORITY         10
#define STATE_MANAGER_TASK_STACK_BYTES      4096

#define NETWORK_TASK_CORE                   0
#define NETWORK_TASK_PRIORITY               5
#define NETWORK_TASK_STACK_BYTES            6144
