#include "guardian/features/correlation.h"
#include <math.h>

static int16_t compute_correlation_q15(const int16_t *x, const int16_t *y, size_t len)
{
    if (len == 0) return 0;
    
    int32_t mean_x = 0, mean_y = 0;
    for (size_t i = 0; i < len; i++) {
        mean_x += x[i];
        mean_y += y[i];
    }
    mean_x /= (int32_t)len;
    mean_y /= (int32_t)len;
    
    int64_t cov = 0, var_x = 0, var_y = 0;
    for (size_t i = 0; i < len; i++) {
        int32_t dx = x[i] - mean_x;
        int32_t dy = y[i] - mean_y;
        cov   += (int64_t)dx * dy;
        var_x += (int64_t)dx * dx;
        var_y += (int64_t)dy * dy;
    }
    
    if (var_x <= 0 || var_y <= 0) return 0;
    
    float denom = sqrtf((float)var_x * (float)var_y);
    if (denom < 1.0f) return 0;
    
    float r = (float)cov / denom;
    if (r >  1.0f) r =  1.0f;
    if (r < -1.0f) r = -1.0f;
    
    return (int16_t)(r * 32767.0f);
}

void extract_correlation_features(const int16_t outputs[NUM_RESONATORS][FRAME_SIZE],
                                  correlation_features_t *features)
{
    features->corr_0_1 = compute_correlation_q15(outputs[0], outputs[1], FRAME_SIZE);
    features->corr_1_2 = compute_correlation_q15(outputs[1], outputs[2], FRAME_SIZE);
    features->corr_2_3 = compute_correlation_q15(outputs[2], outputs[3], FRAME_SIZE);
    
    int16_t max = features->corr_0_1;
    if (features->corr_1_2 > max) max = features->corr_1_2;
    if (features->corr_2_3 > max) max = features->corr_2_3;
    features->max_corr = max;
}
