#ifndef GUARDIAN_FLUX_H
#define GUARDIAN_FLUX_H

#include <stdint.h>
#include "energy.h"

typedef struct {
    int16_t prev_energy[NUM_RESONATORS];
    int16_t flux;
} flux_state_t;

void compute_spectral_flux(flux_state_t *state, const energy_features_t *current);

#endif
