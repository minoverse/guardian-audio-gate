#include "guardian/features/spectral_flatness.h"
#include <math.h>

float compute_spectral_flatness(const energy_features_t *energy)
{
    /* Convert Q15 channel energies to float for log/exp arithmetic.
     * +1 avoids log(0) when a channel is silent.                            */
    float e[NUM_RESONATORS];
    float arith_sum = 0.0f;
    float log_sum   = 0.0f;

    for (int i = 0; i < NUM_RESONATORS; i++) {
        e[i]      = (float)energy->channel_energy[i] + 1.0f;
        arith_sum += e[i];
        log_sum   += logf(e[i]);   /* FPU-accelerated on Cortex-M4F          */
    }

    float arith_mean = arith_sum / (float)NUM_RESONATORS;
    if (arith_mean < 2.0f) {
        /* Near-silence: all channels essentially zero.
         * Return 1.0 (flat/non-speech) to avoid divide-by-zero and ensure
         * silent frames do not pass the spectral flatness gate rule.         */
        return 1.0f;
    }

    /* Geometric mean via exp(mean(log(x)))                                  */
    float geom_mean = expf(log_sum / (float)NUM_RESONATORS);

    float sfm = geom_mean / arith_mean;

    /* Clamp to [0, 1] — floating point edge cases                           */
    if (sfm < 0.0f) sfm = 0.0f;
    if (sfm > 1.0f) sfm = 1.0f;

    return sfm;
}
