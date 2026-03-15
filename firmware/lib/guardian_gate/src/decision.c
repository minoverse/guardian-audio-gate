#include "guardian/gate/decision.h"
#include "guardian/features/correlation.h"
#include "guardian/features/coherence.h"
#include "guardian/features/zcr.h"
#include <zephyr/kernel.h>

gate_decision_t gate_decide(const resonator_bank_df2t_t *bank,
                            const gate_config_t *config)
{
    gate_decision_t decision = {0};
    uint64_t start = k_cycle_get_64();

    /* Feature extraction — cast needed: bank->outputs is int16_t[4][320],
     * functions expect const int16_t (*)[FRAME_SIZE]                      */
    energy_features_t      energy;
    correlation_features_t corr;
    coherence_features_t   coherence;
    uint16_t               zcr;

    extract_energy_features(
        (const int16_t (*)[FRAME_SIZE])bank->outputs, &energy);
    extract_correlation_features(
        (const int16_t (*)[FRAME_SIZE])bank->outputs, &corr);
    extract_coherence_features(bank->outputs[0], FRAME_SIZE, &coherence);
    zcr = compute_zcr(bank->outputs[0], FRAME_SIZE);

    /* Store features for debug / main.c printk */
    decision.features_used.correlation = corr.max_corr;
    decision.features_used.energy      = energy.total_energy;
    decision.features_used.coherence   = coherence.autocorr_peak;
    decision.features_used.zcr         = zcr;

    /* ── Scoring rules (from Python simulation, target F1 = 0.678) ──────── */
    uint8_t score = 0;

    /* Rule 1 (+40): High inter-resonator correlation → likely speech.
     * This is the primary discriminator: speech harmonics produce correlated
     * outputs across resonator channels; noise does not.                    */
    if (corr.max_corr > config->correlation_threshold) {
        score += 40;
    }

    /* Rule 2 (+20): Energy above 2× noise floor → not silence.             */
    if (energy.total_energy > config->noise_floor * 2) {
        score += 20;
    }

    /* Rule 3 (+25): Pitch coherence present → periodic (voiced) signal.
     * autocorr_peak is the max lag-80..250 autocorrelation of resonator[0].*/
    if (coherence.autocorr_peak > config->coherence_threshold) {
        score += 25;
    }

    /* Rule 4 (+15): ZCR below max → not broadband noise (which has high ZCR).
     * Speech: ~30-80 crossings/frame; white noise: >> 100.                  */
    if (zcr < config->zcr_max) {
        score += 15;
    }

    /* Wake if combined score ≥ 55 — lowered from 60 since E feature rounds
     * to 0 for typical speech levels (arm_rms_q15 truncates small values).
     * Rule1(C)+Rule4(ZCR) = 55 is sufficient to discriminate speech. */
    decision.should_wake = (score >= 55);
    decision.confidence  = score;
    decision.elapsed_us  = (uint32_t)k_cyc_to_us_floor64(
                               k_cycle_get_64() - start);

    return decision;
}

void update_noise_floor(noise_tracker_t *tracker,
                        const energy_features_t *energy,
                        bool speech_detected)
{
    /* Only update during non-speech frames — keeps estimate at noise level. */
    if (!speech_detected) {
        /* EMA: noise = 0.9 * noise + 0.1 * current
         * Q15 coefficients: 0.9 * 32768 = 29491,  0.1 * 32768 = 3277     */
        tracker->noise_estimate = (q15_t)(
            (29491 * (int32_t)tracker->noise_estimate +
              3277 * (int32_t)energy->total_energy) >> 15
        );
    }
    tracker->frame_count++;
}
