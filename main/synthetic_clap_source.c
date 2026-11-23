/*
 * synthetic_clap_source.c
 *
 * Fake microphones. Every 4 ms this task writes 98 frames (one sample per
 * microphone each) into the ring buffer, which is exactly 24,500 samples/s,
 * the measured rate of the real ADC. It uses the same
 * audio_ring_buffer_write_frame() call as real capture.
 *
 * What the fake audio contains:
 *   - silence: mid-scale (2048) plus random noise
 *   - a sound from a chosen (x, y): it reaches each microphone after
 *     distance / speed of sound, and is quieter the farther it travels
 *   - the ADC multiplexing skew: microphone B is sampled 1/3 of a sample
 *     after A, and C 2/3 after A, exactly like the real ADC1 pattern
 *   - optionally an echo off a wall (modeled as a mirror-image source)
 *
 * How a sound is built: a sum of damped sine waves ("components"),
 *     amplitude * envelope(t) * sin(2*pi*frequency*t + phase)
 *     envelope(t) = exp(-t/decay_time) - exp(-t/fast_time)   (quick rise, slow decay)
 * This formula can be evaluated at ANY time, even between samples, so the
 * delays are exact. Calling sin() and exp() for every sample would be slow,
 * so each component is kept as a "phasor": a 2-D vector that shrinks and
 * rotates by a fixed step every sample. Its vertical (imaginary) part is the
 * damped sine wave. One multiply per sample replaces sin() and exp().
 */
#include "synthetic_clap_source.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_config.h"
#include "audio_ring_buffer.h"

static const char *LOG_TAG = "synthetic_clap";

#define SAMPLE_PERIOD_S             (1.0f / (float)ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ)
#define TWO_PI                      6.28318531f
#define MAXIMUM_COMPONENT_COUNT     20
#define EMITTER_COUNT               2      /* 0 = direct sound, 1 = wall echo         */
#define DIRECT_EMITTER              0
#define ECHO_EMITTER                1
#define MINIMUM_DISTANCE_FT         0.5f   /* closer than this, loudness stops growing */
#define SOUND_LENGTH_DECAY_TIMES    8.0f   /* after 8 decay times the sound is < 0.04% */
#define PEAK_SEARCH_DECAY_TIMES     2.0f   /* the peak is always early: by 2 decay times the
                                              envelope has fallen to 14% of its maximum   */

_Static_assert((ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ * SYNTHETIC_BLOCK_INTERVAL_MS) % 1000 == 0,
               "A block must hold a whole number of samples");

/* ------------------------------------------------------------------
 * Sounds and scenarios
 * ------------------------------------------------------------------ */

typedef enum
{
    SOUND_CLAP,  /* broadband burst: what the system must localize       */
    SOUND_TONE,  /* 300 Hz hum, like floor vibration: must be ignored    */
} sound_kind_t;

typedef struct
{
    float lowest_frequency_hz;
    float highest_frequency_hz;
    int   component_count;
    float rise_time_s;
    float decay_time_s;
} sound_shape_t;

static const sound_shape_t sound_shapes[] =
{
    /* A clap is mostly 1-5 kHz; the band edges let a little energy fall outside it. */
    [SOUND_CLAP] = { 800.0f, 6000.0f, 20, 0.0001f, 0.004f },
    [SOUND_TONE] = { 300.0f,  300.0f,  1, 0.005f,  0.060f },
};

typedef struct
{
    const char  *description;
    float        x_ft;
    float        y_ft;
    sound_kind_t sound;
    float        level_raw_at_1_ft;  /* peak size in ADC counts, heard 1 ft away */
    bool         add_wall_echo;
} scenario_t;

/* Played in order, one every SYNTHETIC_SCENARIO_INTERVAL_MS, then repeated.
 * The 9 grid points cover the corners, edges and middle of the 5 x 5 ft area. */
static const scenario_t scenarios[] =
{
    { "clap at centroid",           2.5f, 2.634f, SOUND_CLAP, 1000.0f, false },
    { "clap at grid (0.5, 0.5)",    0.5f, 0.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap at grid (2.5, 0.5)",    2.5f, 0.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap at grid (4.5, 0.5)",    4.5f, 0.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap at grid (0.5, 2.5)",    0.5f, 2.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap at grid (2.5, 2.5)",    2.5f, 2.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap at grid (4.5, 2.5)",    4.5f, 2.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap at grid (0.5, 4.5)",    0.5f, 4.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap at grid (2.5, 4.5)",    2.5f, 4.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap at grid (4.5, 4.5)",    4.5f, 4.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap with wall echo",        1.5f, 1.5f,   SOUND_CLAP, 1000.0f, true  },
    { "quiet clap (low SNR)",       3.0f, 2.0f,   SOUND_CLAP,  150.0f, false },
    { "clap outside, right",        6.5f, 2.5f,   SOUND_CLAP, 1000.0f, false },
    { "clap outside, below",        2.5f, -1.5f,  SOUND_CLAP, 1000.0f, false },
    { "300 Hz tone (floor hum)",    2.5f, 2.5f,   SOUND_TONE,  600.0f, false },
};
#define SCENARIO_COUNT ((int)(sizeof(scenarios) / sizeof(scenarios[0])))

static const float microphone_x_ft[MICROPHONE_COUNT] = { MIC_A_X_FT, MIC_B_X_FT, MIC_C_X_FT };
static const float microphone_y_ft[MICROPHONE_COUNT] = { MIC_A_Y_FT, MIC_B_Y_FT, MIC_C_Y_FT };

/* ------------------------------------------------------------------
 * Random numbers (fixed seed, so every run plays identical audio)
 * ------------------------------------------------------------------ */

static uint32_t random_state = SYNTHETIC_RANDOM_SEED;

/**
 * Produces the next pseudo-random 32-bit number (xorshift generator).
 * Inputs:  none
 * Outputs: a number spread evenly over all 32-bit values
 */
static uint32_t next_random(void)
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state;
}

/**
 * Produces a random number spread evenly between 0 and 1.
 * Inputs:  none
 * Outputs: a value in [0, 1)
 */
static float random_uniform(void)
{
    return (float)(next_random() >> 8) * (1.0f / 16777216.0f);
}

/**
 * Produces bell-curve (Gaussian-like) noise with mean 0 and standard deviation 1.
 * Inputs:  none
 * Outputs: a random value, usually within -3..+3. Adding four even random
 *          numbers gives a bell curve (much cheaper than the exact method);
 *          their spread is 1/3, so multiplying by sqrt(3) makes it 1.
 */
static float random_gaussian(void)
{
    float sum = random_uniform() + random_uniform() + random_uniform() + random_uniform();
    return (sum - 2.0f) * 1.7320508f;
}

/* ------------------------------------------------------------------
 * Phasors (2-D vectors / complex numbers)
 * ------------------------------------------------------------------ */

typedef struct
{
    float real;
    float imaginary;
} phasor_t;

/**
 * Multiplies two phasors: magnitudes multiply and angles add.
 * Inputs:  first, second - the two phasors
 * Outputs: their product
 */
static inline phasor_t phasor_multiply(phasor_t first, phasor_t second)
{
    phasor_t product =
    {
        first.real * second.real - first.imaginary * second.imaginary,
        first.real * second.imaginary + first.imaginary * second.real,
    };
    return product;
}

/**
 * Builds a phasor from its length and direction.
 * Inputs:  magnitude - length of the vector
 *          angle_rad - direction in radians
 * Outputs: the phasor (magnitude*cos(angle), magnitude*sin(angle))
 */
static phasor_t make_phasor(float magnitude, float angle_rad)
{
    phasor_t result = { magnitude * cosf(angle_rad), magnitude * sinf(angle_rad) };
    return result;
}

/* ------------------------------------------------------------------
 * The sound currently being played (shared by all microphones and the echo)
 * ------------------------------------------------------------------ */

static int      component_count;
static float    component_frequency_hz[MAXIMUM_COMPONENT_COUNT];
static float    component_phase_rad[MAXIMUM_COMPONENT_COUNT];
static float    component_amplitude[MAXIMUM_COMPONENT_COUNT];
static float    slow_decay_rate_per_s;   /* 1 / decay_time                   */
static float    fast_decay_rate_per_s;   /* 1 / decay_time + 1 / rise_time   */
static float    sound_length_s;
static phasor_t slow_step[MAXIMUM_COMPONENT_COUNT];  /* applied once per sample */
static phasor_t fast_step[MAXIMUM_COMPONENT_COUNT];

/**
 * Finds the largest value the current sound reaches, by tracing its loud early part sample by sample.
 * Inputs:  none (uses the component arrays)
 * Outputs: the peak absolute value (1 if the sound is silent). Only used to
 *          scale the sound, so tracing at the sample rate is precise enough.
 */
static float measure_sound_peak(void)
{
    const float step_s = SAMPLE_PERIOD_S;
    phasor_t slow[MAXIMUM_COMPONENT_COUNT], fast[MAXIMUM_COMPONENT_COUNT];
    phasor_t slow_fine_step[MAXIMUM_COMPONENT_COUNT], fast_fine_step[MAXIMUM_COMPONENT_COUNT];

    for (int component = 0; component < component_count; component++)
    {
        float rotation_rad = TWO_PI * component_frequency_hz[component] * step_s;
        slow[component] = make_phasor(component_amplitude[component], component_phase_rad[component]);
        fast[component] = slow[component];
        slow_fine_step[component] = make_phasor(expf(-slow_decay_rate_per_s * step_s), rotation_rad);
        fast_fine_step[component] = make_phasor(expf(-fast_decay_rate_per_s * step_s), rotation_rad);
    }

    float peak = 0.0f;
    int step_count = (int)(PEAK_SEARCH_DECAY_TIMES / slow_decay_rate_per_s / step_s);
    for (int step = 0; step < step_count; step++)
    {
        float value = 0.0f;
        for (int component = 0; component < component_count; component++)
        {
            value += slow[component].imaginary - fast[component].imaginary;
            slow[component] = phasor_multiply(slow[component], slow_fine_step[component]);
            fast[component] = phasor_multiply(fast[component], fast_fine_step[component]);
        }
        peak = fmaxf(peak, fabsf(value));
    }
    return peak > 0.0f ? peak : 1.0f;
}

/**
 * Picks random frequencies, phases and amplitudes for a new sound, scaled so its peak is 1.
 * Inputs:  kind - which sound shape to build (clap or tone)
 * Outputs: none. Fills the "current sound" variables above.
 */
static void design_sound(sound_kind_t kind)
{
    const sound_shape_t *shape = &sound_shapes[kind];
    component_count = shape->component_count;
    slow_decay_rate_per_s = 1.0f / shape->decay_time_s;
    fast_decay_rate_per_s = slow_decay_rate_per_s + 1.0f / shape->rise_time_s;
    sound_length_s = SOUND_LENGTH_DECAY_TIMES * shape->decay_time_s;

    for (int component = 0; component < component_count; component++)
    {
        float frequency_span_hz = shape->highest_frequency_hz - shape->lowest_frequency_hz;
        component_frequency_hz[component] = shape->lowest_frequency_hz + frequency_span_hz * random_uniform();
        component_phase_rad[component] = TWO_PI * random_uniform();
        component_amplitude[component] = 0.5f + 0.5f * random_uniform();
    }

    float peak = measure_sound_peak();
    for (int component = 0; component < component_count; component++)
    {
        component_amplitude[component] /= peak;
        float rotation_rad = TWO_PI * component_frequency_hz[component] * SAMPLE_PERIOD_S;
        slow_step[component] = make_phasor(expf(-slow_decay_rate_per_s * SAMPLE_PERIOD_S), rotation_rad);
        fast_step[component] = make_phasor(expf(-fast_decay_rate_per_s * SAMPLE_PERIOD_S), rotation_rad);
    }
}

/* ------------------------------------------------------------------
 * Sound paths: the current sound as heard by one microphone from one emitter
 * ------------------------------------------------------------------ */

typedef struct
{
    bool     active;
    int32_t  first_sample_offset;  /* first sample (counted from emission) that hears the sound */
    int32_t  last_sample_offset;   /* after this sample the sound has died away                 */
    phasor_t slow_phasor[MAXIMUM_COMPONENT_COUNT];
    phasor_t fast_phasor[MAXIMUM_COMPONENT_COUNT];
} sound_path_t;

static sound_path_t sound_paths[EMITTER_COUNT][MICROPHONE_COUNT];
static uint32_t     emission_sample_number;

/**
 * Prepares one microphone to hear the current sound after it travels a given distance.
 * Inputs:  path - the path to set up
 *          microphone_index - 0 = A, 1 = B, 2 = C (sets the multiplexing skew)
 *          distance_ft - how far the sound travels to this microphone
 *          level_raw_at_1_ft - loudness 1 ft from the source, in ADC counts
 * Outputs: none. The path's phasors start exactly where the sound is at the
 *          microphone's first sample after arrival.
 */
static void start_sound_path(sound_path_t *path, int microphone_index, float distance_ft, float level_raw_at_1_ft)
{
    float arrival_s = distance_ft / SPEED_OF_SOUND_FT_PER_S;

    /* Sample k of this microphone is taken at k*T + skew after the emission. */
    float skew_s = (float)microphone_index * SAMPLE_PERIOD_S / (float)MICROPHONE_COUNT;
    int32_t first_sample_offset = (int32_t)ceilf((arrival_s - skew_s) / SAMPLE_PERIOD_S);

    /* How far into the sound that first sample lands: between 0 and one sample period.
     * This fraction is what makes the delay exact rather than rounded to a sample. */
    float time_into_sound_s = (float)first_sample_offset * SAMPLE_PERIOD_S + skew_s - arrival_s;
    float level_raw = level_raw_at_1_ft / fmaxf(distance_ft, MINIMUM_DISTANCE_FT);

    for (int component = 0; component < component_count; component++)
    {
        float angle_rad = TWO_PI * component_frequency_hz[component] * time_into_sound_s + component_phase_rad[component];
        float magnitude = level_raw * component_amplitude[component];
        path->slow_phasor[component] = make_phasor(magnitude * expf(-slow_decay_rate_per_s * time_into_sound_s), angle_rad);
        path->fast_phasor[component] = make_phasor(magnitude * expf(-fast_decay_rate_per_s * time_into_sound_s), angle_rad);
    }
    path->first_sample_offset = first_sample_offset;
    path->last_sample_offset = first_sample_offset + (int32_t)(sound_length_s / SAMPLE_PERIOD_S);
    path->active = true;
}

/**
 * Gives a path's contribution to one sample, then advances it to the next sample.
 * Inputs:  path - the path to evaluate
 *          sample_offset - samples since the emission (must rise by exactly 1 per call)
 * Outputs: the sound's value in ADC counts (0 before arrival and after it dies away)
 */
static float next_path_value(sound_path_t *path, int32_t sample_offset)
{
    if (!path->active || sample_offset < path->first_sample_offset)
    {
        return 0.0f;
    }
    if (sample_offset > path->last_sample_offset)
    {
        path->active = false;
        return 0.0f;
    }

    float value = 0.0f;
    for (int component = 0; component < component_count; component++)
    {
        value += path->slow_phasor[component].imaginary - path->fast_phasor[component].imaginary;
        path->slow_phasor[component] = phasor_multiply(path->slow_phasor[component], slow_step[component]);
        path->fast_phasor[component] = phasor_multiply(path->fast_phasor[component], fast_step[component]);
    }
    return value;
}

/* ------------------------------------------------------------------
 * Scenarios and ground truth
 * ------------------------------------------------------------------ */

static portMUX_TYPE           truth_lock = portMUX_INITIALIZER_UNLOCKED;
static synthetic_clap_truth_t last_truth;  /* clap_number 0 = nothing played yet */
static int                    next_scenario_index;
static bool                   next_sound_designed;
static uint32_t               longest_cycle_us;  /* protected by truth_lock */

/**
 * Tells whether any microphone is still hearing the current sound (direct or echo).
 * Inputs:  none
 * Outputs: true while any sound path is active
 */
static bool any_sound_playing(void)
{
    for (int emitter = 0; emitter < EMITTER_COUNT; emitter++)
    {
        for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++)
        {
            if (sound_paths[emitter][microphone_index].active)
            {
                return true;
            }
        }
    }
    return false;
}

/**
 * Starts the next scenario: aims its (already designed) sound at every microphone and publishes the truth.
 * Inputs:  sample_number - ring buffer sample number at which the sound is emitted
 * Outputs: none. Sound paths are set up; last_truth is updated.
 */
static void start_next_scenario(uint32_t sample_number)
{
    const scenario_t *scenario = &scenarios[next_scenario_index];
    next_scenario_index = (next_scenario_index + 1) % SCENARIO_COUNT;
    next_sound_designed = false;

    emission_sample_number = sample_number;

    float direct_distance_ft[MICROPHONE_COUNT];
    for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++)
    {
        direct_distance_ft[microphone_index] = hypotf(scenario->x_ft - microphone_x_ft[microphone_index], scenario->y_ft - microphone_y_ft[microphone_index]);
        start_sound_path(&sound_paths[DIRECT_EMITTER][microphone_index], microphone_index, direct_distance_ft[microphone_index], scenario->level_raw_at_1_ft);

        if (scenario->add_wall_echo)
        {
            /* An echo off a wall sounds like a second source at the mirror image
             * of the real one, behind the wall, and a bit quieter. */
            float image_x_ft = 2.0f * SYNTHETIC_WALL_X_FT - scenario->x_ft;
            float echo_distance_ft = hypotf(image_x_ft - microphone_x_ft[microphone_index], scenario->y_ft - microphone_y_ft[microphone_index]);
            start_sound_path(&sound_paths[ECHO_EMITTER][microphone_index], microphone_index, echo_distance_ft, scenario->level_raw_at_1_ft * SYNTHETIC_WALL_REFLECTION);
        } else
        {
            sound_paths[ECHO_EMITTER][microphone_index].active = false;
        }
    }

    synthetic_clap_truth_t truth =
    {
        .clap_number = last_truth.clap_number + 1,
        .description = scenario->description,
        .x_ft = scenario->x_ft,
        .y_ft = scenario->y_ft,
        .is_inside_area = scenario->x_ft >= 0.0f && scenario->x_ft <= SENSING_AREA_WIDTH_FT && scenario->y_ft >= 0.0f && scenario->y_ft <= SENSING_AREA_HEIGHT_FT,
        .is_clap = scenario->sound == SOUND_CLAP,
        .emission_sample_number = sample_number,
        .delay_ab_us = (direct_distance_ft[0] - direct_distance_ft[1]) / SPEED_OF_SOUND_FT_PER_S * 1.0e6f,
        .delay_bc_us = (direct_distance_ft[1] - direct_distance_ft[2]) / SPEED_OF_SOUND_FT_PER_S * 1.0e6f,
        .delay_ca_us = (direct_distance_ft[2] - direct_distance_ft[0]) / SPEED_OF_SOUND_FT_PER_S * 1.0e6f,
    };
    portENTER_CRITICAL(&truth_lock);
    last_truth = truth;
    portEXIT_CRITICAL(&truth_lock);
}

/**
 * Turns a computed value into a valid 12-bit ADC reading.
 * Inputs:  value_raw - value in ADC counts (may be fractional or out of range)
 * Outputs: the value rounded and clipped to 0..ADC_RAW_MAXIMUM, as the real ADC would
 */
static int16_t clip_to_adc_range(float value_raw)
{
    if (value_raw < 0.0f)
    {
        return 0;
    }
    if (value_raw > (float)ADC_RAW_MAXIMUM)
    {
        return ADC_RAW_MAXIMUM;
    }
    return (int16_t)(value_raw + 0.5f);
}

/**
 * Generates one block of frames (noise plus any active sound) and stores them in the ring buffer.
 * Inputs:  none
 * Outputs: none. SYNTHETIC_BLOCK_SAMPLES frames are written.
 */
static void write_block(void)
{
    for (int block_index = 0; block_index < SYNTHETIC_BLOCK_SAMPLES; block_index++)
    {
        int32_t sample_offset = (int32_t)(audio_ring_buffer_get_sample_count() - emission_sample_number);
        int16_t frame[MICROPHONE_COUNT];

        for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++)
        {
            float value_raw = SYNTHETIC_BASELINE_RAW + SYNTHETIC_NOISE_SIGMA_RAW * random_gaussian();
            for (int emitter = 0; emitter < EMITTER_COUNT; emitter++)
            {
                value_raw += next_path_value(&sound_paths[emitter][microphone_index], sample_offset);
            }
            frame[microphone_index] = clip_to_adc_range(value_raw);
        }
        audio_ring_buffer_write_frame(frame);
    }
}

/**
 * Task body: every block interval, starts a scenario if one is due, then writes a block.
 * Inputs:  task_argument - unused
 * Outputs: none (never returns)
 */
static void synthetic_clap_source_task(void *task_argument)
{
    const uint32_t scenario_interval_samples = (uint32_t)SYNTHETIC_SCENARIO_INTERVAL_MS * ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ / 1000;
    uint32_t next_scenario_sample = (uint32_t)SYNTHETIC_FIRST_SCENARIO_DELAY_MS * ACTUAL_SAMPLE_RATE_PER_CHANNEL_HZ / 1000;

    ESP_LOGI(LOG_TAG, "Synthetic source started: %d scenarios, one every %d ms, noise sigma %.1f counts", SCENARIO_COUNT, SYNTHETIC_SCENARIO_INTERVAL_MS, SYNTHETIC_NOISE_SIGMA_RAW);

    /* vTaskDelayUntil wakes at fixed 4 ms marks (not 4 ms after we finish),
     * so the long-run rate is exact even if one block takes longer. */
    TickType_t last_wake_tick = xTaskGetTickCount();
    while (true)
    {
        int64_t cycle_start_us = esp_timer_get_time();
        uint32_t sample_number = audio_ring_buffer_get_sample_count();
        bool scenario_due = (int32_t)(sample_number - next_scenario_sample) >= 0;

        /* Design the next sound during the quiet gap, not at the moment it is
         * emitted, so emission costs no extra time. design_sound() replaces the
         * shared component data, so it must wait until the last sound is over. */
        if (!next_sound_designed && (!any_sound_playing() || scenario_due))
        {
            design_sound(scenarios[next_scenario_index].sound);
            next_sound_designed = true;
        }
        if (scenario_due)
        {
            start_next_scenario(sample_number);
            next_scenario_sample += scenario_interval_samples;
        }
        write_block();

        uint32_t cycle_us = (uint32_t)(esp_timer_get_time() - cycle_start_us);
        portENTER_CRITICAL(&truth_lock);
        if (cycle_us > longest_cycle_us)
        {
            longest_cycle_us = cycle_us;
        }
        portEXIT_CRITICAL(&truth_lock);

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(SYNTHETIC_BLOCK_INTERVAL_MS));
    }
}

void synthetic_clap_source_start(void)
{
    audio_ring_buffer_initialize();
    xTaskCreatePinnedToCore(synthetic_clap_source_task, "synthetic_clap", AUDIO_RX_TASK_STACK_BYTES, NULL, AUDIO_RX_TASK_PRIORITY, NULL, AUDIO_RX_TASK_CORE);
}

bool synthetic_clap_source_get_last_truth(synthetic_clap_truth_t *truth)
{
    portENTER_CRITICAL(&truth_lock);
    *truth = last_truth;
    portEXIT_CRITICAL(&truth_lock);
    return truth->clap_number != 0;
}

uint32_t synthetic_clap_source_read_and_reset_longest_cycle_us(void)
{
    portENTER_CRITICAL(&truth_lock);
    uint32_t longest_us = longest_cycle_us;
    longest_cycle_us = 0;
    portEXIT_CRITICAL(&truth_lock);
    return longest_us;
}
