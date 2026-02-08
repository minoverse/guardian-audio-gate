#include "guardian/features/zcr.h"

uint16_t compute_zcr(const int16_t *signal, size_t len)
{
    uint16_t crossings = 0;
    
    for (size_t i = 1; i < len; i++) {
        if ((signal[i-1] >= 0 && signal[i] < 0) ||
            (signal[i-1] < 0 && signal[i] >= 0)) {
            crossings++;
        }
    }
    
    return crossings;
}
