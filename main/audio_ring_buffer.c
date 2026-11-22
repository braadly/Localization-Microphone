/*
 * audio_ring_buffer.c
 *
 * Single-writer, single-reader circular buffer. No lock is used because the
 * writer must never wait for the reader (a late write loses audio). Instead
 * the writer publishes its progress with one atomic number, the sample count:
 *
 *   writer: store the samples, THEN publish the new count (release)
 *   reader: read the count (acquire), THEN read samples older than it,
 *           and afterwards check the writer did not lap it during the copy
 */
#include "audio_ring_buffer.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>
#include "app_config.h"

/* The wrap-around mask only works when the capacity is a power of two. */
_Static_assert((RING_BUFFER_CAPACITY_SAMPLES & (RING_BUFFER_CAPACITY_SAMPLES - 1)) == 0, "RING_BUFFER_CAPACITY_SAMPLES must be a power of two");

#define RING_BUFFER_INDEX_MASK (RING_BUFFER_CAPACITY_SAMPLES - 1)

static int16_t sample_storage[MICROPHONE_COUNT][RING_BUFFER_CAPACITY_SAMPLES];
static _Atomic uint32_t stored_sample_count;

void audio_ring_buffer_initialize(void)
{
    memset(sample_storage, 0, sizeof(sample_storage));
    atomic_store_explicit(&stored_sample_count, 0, memory_order_release);
}

void audio_ring_buffer_write_frame(const int16_t sample_per_microphone[])
{
    uint32_t next_sample_number = atomic_load_explicit(&stored_sample_count, memory_order_relaxed);
    uint32_t storage_index = next_sample_number & RING_BUFFER_INDEX_MASK;

    for (int microphone_index = 0; microphone_index < MICROPHONE_COUNT; microphone_index++)
    {
        sample_storage[microphone_index][storage_index] = sample_per_microphone[microphone_index];
    }

    /* Publishing the count last guarantees a reader that sees the new count
     * also sees the samples that were stored before it. */
    atomic_store_explicit(&stored_sample_count, next_sample_number + 1, memory_order_release);
}

uint32_t audio_ring_buffer_get_sample_count(void)
{
    return atomic_load_explicit(&stored_sample_count, memory_order_acquire);
}

/**
 * Tells whether a run of samples has been lost to overwriting, given the count.
 * Inputs:  newest_count - the published sample count
 *          first_sample_number - first sample of the run
 * Outputs: true if the run's first sample has been (or is being) overwritten.
 *          The sample numbered (newest_count - capacity) shares its slot with
 *          the sample the writer may be storing right now, so it counts as lost.
 */
static bool run_is_overwritten(uint32_t newest_count, uint32_t first_sample_number)
{
    return (uint32_t)(newest_count - first_sample_number) >= RING_BUFFER_CAPACITY_SAMPLES;
}

ring_buffer_status_t audio_ring_buffer_copy_samples(int microphone_index, uint32_t first_sample_number, uint32_t sample_count, int16_t *destination)
{
    uint32_t newest_count = audio_ring_buffer_get_sample_count();
    uint32_t end_sample_number = first_sample_number + sample_count;

    /* Subtracting then casting to signed handles the 32-bit wrap-around. */
    if ((int32_t)(newest_count - end_sample_number) < 0)
    {
        return RING_BUFFER_NOT_YET_AVAILABLE;
    }

    if (run_is_overwritten(newest_count, first_sample_number))
    {
        return RING_BUFFER_ALREADY_OVERWRITTEN;
    }

    for (uint32_t offset = 0; offset < sample_count; offset++)
    {
        destination[offset] =
            sample_storage[microphone_index][(first_sample_number + offset) & RING_BUFFER_INDEX_MASK];
    }

    /* The writer keeps running while we copy. If it lapped our first sample
     * during the copy, what we copied is a mixture and cannot be trusted. */
    if (run_is_overwritten(audio_ring_buffer_get_sample_count(), first_sample_number))
    {
        return RING_BUFFER_ALREADY_OVERWRITTEN;
    }
    return RING_BUFFER_OK;
}
