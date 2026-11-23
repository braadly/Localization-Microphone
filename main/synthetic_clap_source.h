/*
 * synthetic_clap_source.h
 *
 * Test stand-in for the microphones (Step 1b). Writes fake audio into the
 * audio ring buffer at the real capture rate: background noise, plus a clap
 * from a known (x, y) position every SYNTHETIC_SCENARIO_INTERVAL_MS. Because
 * the position is chosen, the correct delays are known exactly ("ground
 * truth"), so every later step can be checked against them.
 *
 * Selected with AUDIO_SOURCE_SYNTHETIC in app_config.h; the rest of the
 * firmware cannot tell it apart from real capture.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* What the generator actually played, for comparing against the pipeline's answer.
 * Delay convention (same as localization_event_t): delay_xy = arrival time at
 * microphone X minus arrival time at microphone Y. These are the PHYSICAL
 * delays; the ADC multiplexing skew is not included. */
typedef struct
{
    uint32_t    clap_number;            /* 1, 2, 3, ... (0 = nothing played yet)           */
    const char *description;            /* e.g. "clap at centroid"                          */
    float       x_ft;
    float       y_ft;
    bool        is_inside_area;         /* false: the pipeline must reject it (SW-2)        */
    bool        is_clap;                /* false for the 300 Hz tone: must not trigger (SW-11) */
    uint32_t    emission_sample_number; /* ring buffer sample number when the sound started */
    float       delay_ab_us;
    float       delay_bc_us;
    float       delay_ca_us;
} synthetic_clap_truth_t;

/**
 * Creates the generator task on Core 1, which starts filling the ring buffer.
 * Inputs:  none
 * Outputs: none. Replaces audio_capture_start() when AUDIO_SOURCE_SYNTHETIC is 1.
 */
void synthetic_clap_source_start(void);

/**
 * Returns the ground truth of the most recent synthetic sound.
 * Inputs:  truth - struct that receives a copy
 * Outputs: true if a sound has been played (truth filled), false if not yet.
 *          Safe to call from any task.
 */
bool synthetic_clap_source_get_last_truth(synthetic_clap_truth_t *truth);

/**
 * Returns the longest time one generator cycle (one block) took since the last call, then restarts it.
 * Inputs:  none
 * Outputs: microseconds. Must stay well under SYNTHETIC_BLOCK_INTERVAL_MS
 *          (4000 us), or the generator falls behind real time.
 */
uint32_t synthetic_clap_source_read_and_reset_longest_cycle_us(void);
