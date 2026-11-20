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

/* ADC1 channel number for each microphone (ESP32-S3: ADC1_CHn is GPIO n+1).
 * GPIO0/GPIO3 are strapping pins and GPIO4-GPIO9 are reserved for the
 * PCM1864 and its I2C bus on the final board, so they are avoided here. */
#define MIC_A_ADC_CHANNEL                   0   /* GPIO1  */
#define MIC_B_ADC_CHANNEL                   1   /* GPIO2  */
#define MIC_C_ADC_CHANNEL                   9   /* GPIO10 */

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
