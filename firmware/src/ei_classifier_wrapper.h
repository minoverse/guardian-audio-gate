#ifndef EI_CLASSIFIER_WRAPPER_H
#define EI_CLASSIFIER_WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Best classification result from Edge Impulse keyword spotting model */
typedef struct {
    const char *label;  /* "yes", "no", "unknown", "noise" */
    float       score;  /* 0.0–1.0 confidence */
} ei_kws_result_t;

/* Run keyword spotting on the pre-roll ring buffer.
 *
 * ring_flat  : (const int16_t *)preroll_ring — flat view of 2D ring
 * head       : preroll_head  — next write slot (oldest frame index)
 * filled     : preroll_filled — number of valid frames (0..n_frames)
 * n_frames   : PREROLL_FRAMES
 * frame_size : FRAME_SIZE (320 samples)
 * out        : receives best label + confidence
 *
 * Returns 0 on success, negative on error.
 * Shorter context is zero-padded at the start (first call after boot).
 */
int ei_classify(const int16_t *ring_flat,
                uint32_t head, uint32_t filled,
                uint32_t n_frames, uint32_t frame_size,
                ei_kws_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* EI_CLASSIFIER_WRAPPER_H */
