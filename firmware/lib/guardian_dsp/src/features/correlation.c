#include "guardian/features/correlation.h"
#include <arm_math.h>

static int16_t compute_correlation_q15(const int16_t *x, const int16_t *y, size_t len)
{
    int32_t mean_x = 0, mean_y = 0;
    
    for (size_t i = 0; i < len; i++) {
        mean_x += x[i];
        mean_y += y[i];
    }
    mean_x /= len;
    mean_y /= len;
    
    int32_t cov = 0, var_x = 0, var_y = 0;
    
    for (size_t i = 0; i < len; i++) {
        int32_t dx = x[i] - mean_x;
        int32_t dy = y[i] - mean_y;
        cov += (dx * dy) >> 15;
        var_x += (dx * dx) >> 15;
        var_y += (dy * dy) >> 15;
    }
    
    cov /= len;
    var_x /= len;
    var_y /= len;
    
    int32_t denom_sq = (var_x * var_y) >> 15;
    if (denom_sq <= 0) return 0;
    
    int16_t denom;
    arm_sqrt_q15((int16_t)denom_sq, &denom);
    
    return (int16_t)((cov << 15) / denom);
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
