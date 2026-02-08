#include "guardian/features/flux.h"
#include <stdlib.h>

void compute_spectral_flux(flux_state_t *state, const energy_features_t *current)
{
    int32_t flux_sum = 0;
    
    for (int ch = 0; ch < NUM_RESONATORS; ch++) {
        int32_t diff = current->channel_energy[ch] - state->prev_energy[ch];
        flux_sum += abs(diff);
        state->prev_energy[ch] = current->channel_energy[ch];
    }
    
    state->flux = (int16_t)(flux_sum / NUM_RESONATORS);
}
