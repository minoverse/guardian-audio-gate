#ifndef GUARDIAN_ENERGY_H
#define GUARDIAN_ENERGY_H

#include <stdint.h>

#define NUM_RESONATORS 4
#define FRAME_SIZE 320

typedef struct {
    int16_t channel_energy[NUM_RESONATORS];
    int16_t total_energy;
} energy_features_t;

void extract_energy_features(const int16_t outputs[NUM_RESONATORS][FRAME_SIZE],
                             energy_features_t *features);

#endif
