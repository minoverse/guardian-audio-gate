#include "guardian/features/energy.h"
#include <arm_math.h>

void extract_energy_features(const int16_t outputs[NUM_RESONATORS][FRAME_SIZE],
                             energy_features_t *features)
{
    int32_t total = 0;
    
    for (int ch = 0; ch < NUM_RESONATORS; ch++) {
        int16_t rms;
        arm_rms_q15(outputs[ch], FRAME_SIZE, &rms);
        features->channel_energy[ch] = rms;
        total += rms;
    }
    
    features->total_energy = (int16_t)(total / NUM_RESONATORS);
}
