#ifndef GUARDIAN_CORRELATION_H
#define GUARDIAN_CORRELATION_H

#include <stdint.h>

#define NUM_RESONATORS 4
#define FRAME_SIZE 320

typedef struct {
    int16_t corr_0_1;
    int16_t corr_1_2;
    int16_t corr_2_3;
    int16_t max_corr;
} correlation_features_t;

void extract_correlation_features(const int16_t outputs[NUM_RESONATORS][FRAME_SIZE],
                                  correlation_features_t *features);

#endif
