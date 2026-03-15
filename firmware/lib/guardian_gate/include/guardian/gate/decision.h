#ifndef GUARDIAN_GATE_DECISION_H
#define GUARDIAN_GATE_DECISION_H

#include <stdint.h>
#include <stdbool.h>
#include <arm_math.h>
#include "guardian/resonator_df2t.h"
#include "guardian/features/energy.h"

/* ── Gate configuration ───────────────────────────────────────────────────── */

typedef struct {
    q15_t    correlation_threshold; /* 0.45 in Q15 = 14746 — main speech discriminator */
    q15_t    energy_ratio_min;      /* min energy to avoid waking on silence (unused in scoring) */
    q15_t    coherence_threshold;   /* pitch autocorrelation peak threshold                     */
    uint16_t zcr_max;               /* max zero-crossings per frame (noise has high ZCR)        */
    q15_t    noise_floor;           /* current noise estimate — updated by noise_tracker_t      */
} gate_config_t;

/* Defaults tuned to F1=0.678 from Python simulation:
 *   correlation_threshold  = 0.45  → 14746 in Q15
 *   noise_floor starts at 500 (overridden quickly by tracker)
 *   coherence_threshold    = 1000  (raw autocorr peak units)
 *   zcr_max                = 80    (speech: ~30-80, noise: >100)
 */
#define GATE_CONFIG_DEFAULT {          \
    .correlation_threshold = 14746,   \
    .energy_ratio_min      = 200,     \
    .coherence_threshold   = 1000,    \
    .zcr_max               = 80,      \
    .noise_floor           = 500,     \
}

/* ── Gate decision result ─────────────────────────────────────────────────── */

typedef struct {
    bool     should_wake;   /* true = run TinyML inference; false = abort */
    uint8_t  confidence;    /* 0–100 scoring sum                          */
    uint32_t elapsed_us;    /* gate processing time in microseconds       */

    struct {
        q15_t    correlation; /* max inter-resonator correlation (Q15)    */
        q15_t    energy;      /* average RMS across resonator bank (Q15)  */
        q15_t    coherence;   /* pitch autocorrelation peak               */
        uint16_t zcr;         /* zero-crossing rate per frame             */
    } features_used;
} gate_decision_t;

/* ── Adaptive noise floor tracker ────────────────────────────────────────── */

typedef struct {
    q15_t    noise_estimate; /* exponential moving average of quiet-frame energy */
    uint32_t frame_count;    /* total frames processed                           */
} noise_tracker_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/**
 * Run the gate decision on the latest resonator bank outputs.
 *
 * Call AFTER resonator_bank_df2t_process().
 * All feature extraction happens internally.
 */
gate_decision_t gate_decide(const resonator_bank_df2t_t *bank,
                            const gate_config_t *config);

/**
 * Update the noise floor estimate.
 *
 * Call AFTER gate_decide() each frame.
 * Copy tracker->noise_estimate → config->noise_floor after calling.
 *
 * Uses EMA: noise = 0.9 * noise + 0.1 * current  (Q15 coefficients 29491 / 3277)
 * Only updates during non-speech frames to avoid pulling estimate upward.
 */
void update_noise_floor(noise_tracker_t *tracker,
                        const energy_features_t *energy,
                        bool speech_detected);

#endif /* GUARDIAN_GATE_DECISION_H */
