/*
 * audio_ring_buffer.h
 *
 * Circular store for the most recent audio of all microphones.
 *
 * One task (the audio capture task) writes; the localizing pipeline reads.
 * Every sample gets a number: the first sample ever stored is sample 0, the
 * next is sample 1, and so on. That number is the project's clock. Delays
 * between microphones are measured in these sample numbers (Section 7.3.3),
 * and an event's timestamp is the sample number where it started.
 *
 * Because the buffer is circular, a sample is only available for a short time:
 * after RING_BUFFER_CAPACITY_SAMPLES more samples arrive it is overwritten.
 */
#pragma once

#include <stdint.h>

typedef enum
{
    RING_BUFFER_OK = 0,
    RING_BUFFER_NOT_YET_AVAILABLE,  /* part of the range has not been written yet */
    RING_BUFFER_ALREADY_OVERWRITTEN /* part of the range is too old and was lost    */
} ring_buffer_status_t;

/**
 * Clears the buffer and resets the sample count to zero.
 * Inputs:  none
 * Outputs: none (all samples read as zero afterwards)
 */
void audio_ring_buffer_initialize(void);

/**
 * Stores one sample from every microphone, as the next sample number.
 * Inputs:  sample_per_microphone - one value for each microphone, in
 *          microphone order A, B, C (raw ADC counts)
 * Outputs: none. Only the writer task may call this.
 */
void audio_ring_buffer_write_frame(const int16_t sample_per_microphone[]);

/**
 * Returns how many samples of each microphone have been stored so far.
 * Inputs:  none
 * Outputs: the sample count, which is also the number the NEXT sample gets.
 *          It wraps to zero after about 49 hours; compare counts by
 *          subtracting, never with < or >.
 */
uint32_t audio_ring_buffer_get_sample_count(void);

/**
 * Copies a run of consecutive samples of one microphone out of the buffer.
 * Inputs:  microphone_index - 0 = A, 1 = B, 2 = C
 *          first_sample_number - number of the first sample wanted
 *          sample_count - how many samples to copy
 *          destination - array that receives the samples (sample_count long)
 * Outputs: RING_BUFFER_OK if destination now holds valid samples, otherwise
 *          why the request failed (destination contents are then undefined)
 */
ring_buffer_status_t audio_ring_buffer_copy_samples(int microphone_index, uint32_t first_sample_number, uint32_t sample_count, int16_t *destination);
