#include "guardian/features/modulation.h"
#include <arm_math.h>
#include <string.h>

/* EMA alpha for 500ms time constant at 20ms/frame:
 * α = 1 - exp(-dt/τ) = 1 - exp(-20/500) = 0.03921                          */
#define MOD_ALPHA 0.03921f

void modulation_init(modulation_state_t *state)
{
    memset(state, 0, sizeof(*state));
    /* Start mean at a small positive value to prevent div-by-zero on first
     * frame if energy is 0 (silence at boot).                                */
    state->mean_ema = 1.0f;
}

float modulation_update(modulation_state_t *state, int16_t frame_energy_q15)
{
    float energy = (float)frame_energy_q15;

    /* Update mean EMA: μ[n] = α*x[n] + (1-α)*μ[n-1]                        */
    float new_mean = MOD_ALPHA * energy + (1.0f - MOD_ALPHA) * state->mean_ema;

    /* Update variance EMA: σ²[n] = α*(x[n]-μ[n-1])² + (1-α)*σ²[n-1]
     * Use the previous mean (not new_mean) so variance tracks actual spread. */
    float diff     = energy - state->mean_ema;
    float new_var  = MOD_ALPHA * (diff * diff)
                   + (1.0f - MOD_ALPHA) * state->var_ema;

    state->mean_ema = new_mean;
    state->var_ema  = new_var;

    /* Warmup: CV estimate is unreliable until EMA has converged (~5τ).
     * Return 0 during warmup so gate never fires on stale state.             */
    if (!state->valid) {
        state->warmup_n++;
        if (state->warmup_n >= MODULATION_WARMUP_FRAMES) {
            state->valid = true;
        }
        return 0.0f;
    }

    /* Coefficient of Variation = σ / μ
     * arm_sqrt_f32 uses the Cortex-M4 hardware sqrt instruction (~14 cycles). */
    float sigma;
    arm_sqrt_f32(state->var_ema, &sigma);

    float cv = (state->mean_ema > 1.0f) ? (sigma / state->mean_ema) : 0.0f;

    /* Clamp to [0, 2] — CV > 2 indicates impulse noise, not speech          */
    if (cv > 2.0f) cv = 2.0f;
    return cv;
}

void modulation_reset(modulation_state_t *state)
{
    modulation_init(state);
}
