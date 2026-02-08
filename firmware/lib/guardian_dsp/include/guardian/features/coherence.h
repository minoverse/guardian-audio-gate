#ifndef GUARDIAN_COHERENCE_H
#define GUARDIAN_COHERENCE_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    int16_t autocorr_peak;
    uint16_t peak_lag;
} coherence_features_t;

void extract_coherence_features(const int16_t *signal, size_t len,
                               coherence_features_t *features);

#endif
