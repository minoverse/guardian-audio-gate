#include "guardian/features/coherence.h"
#include <arm_math.h>

void extract_coherence_features(const int16_t *signal, size_t len,
                               coherence_features_t *features)
{
    int16_t max_corr = 0;
    uint16_t max_lag = 0;
    
    /* Lag range covers pitch period 62-250 samples @ 16kHz = 64-258 Hz.
     * Original lag=80 only covered 64-200 Hz (male speech, f0=80-155Hz).
     * Extended to lag=62 to cover female speech (f0=165-255Hz, period 63-97).
     * Cost: +9 iterations per frame (~50us) — negligible vs 5ms budget.     */
    for (uint16_t lag = 62; lag < 250 && lag < len/2; lag += 2) {
        q63_t result;
        arm_dot_prod_q15(signal, &signal[lag], len - lag, &result);
        int32_t sum = (int32_t)(result >> 15);
        
        if (sum > max_corr) {
            max_corr = (int16_t)sum;
            max_lag = lag;
        }
    }
    
    features->autocorr_peak = max_corr;
    features->peak_lag = max_lag;
}
