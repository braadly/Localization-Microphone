/*
 * event_types.h
 *
 * Data types shared by more than one subsystem. The event record is the one
 * thing that crosses from Core 1 (localizing pipeline) to Core 0 (state
 * manager and phone), so its layout is defined here once (Table 7.2.5).
 */
#pragma once

#include <stdint.h>

/* Which three channels the pipeline localizes with (Section 7.4.1). Hybrid
 * mode is kept in the types so the optical channel can be added later without
 * changing the event format; the demo runs Acoustic mode only. */
typedef enum {
    SYSTEM_MODE_ACOUSTIC = 0,   /* microphones A, B, C          */
    SYSTEM_MODE_HYBRID   = 1,   /* microphones A, B + optical   */
} system_mode_t;

/* Outcome of event validation (Figure 7.4.2). */
typedef enum {
    EVENT_STATUS_VALID    = 0,  /* plotted on the map and logged */
    EVENT_STATUS_REJECTED = 1,  /* logged only, with a reason    */
} event_status_t;

/* Why an event was rejected; REJECTION_REASON_NONE for valid events. */
typedef enum {
    REJECTION_REASON_NONE              = 0,
    REJECTION_REASON_FAILED_CLOSURE    = 1,  /* tau_AB + tau_BC + tau_CA far from 0 */
    REJECTION_REASON_OUTSIDE_PERIMETER = 2,  /* position outside the 5 x 5 ft area  */
    REJECTION_REASON_LOW_CONFIDENCE    = 3,  /* SRP-PHAT peak below the threshold   */
} rejection_reason_t;

/* One detected impulse, valid or rejected (about 40 bytes). */
typedef struct {
    uint32_t           event_id;             /* increments by one per event                    */
    uint32_t           onset_sample_count;   /* timestamp: per-channel sample count at onset    */
    float              x_ft;                 /* estimated position, feet from lower-left corner */
    float              y_ft;
    float              confidence;           /* normalized SRP-PHAT peak, 0 to 1                */
    float              delay_ab_us;          /* arrival-time differences after offset correction */
    float              delay_bc_us;
    float              delay_ca_us;
    system_mode_t      mode;
    event_status_t     status;
    rejection_reason_t rejection_reason;
} localization_event_t;
