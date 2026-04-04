/*
 * Guardian Audio Gate — Edge Impulse classifier wrapper
 *
 * Thin C++ shim so main.c (pure C) can call run_classifier().
 * Converts the int16_t pre-roll ring buffer → float32 normalized signal
 * on-the-fly via the EI signal callback — zero extra RAM.
 */

#include "ei_classifier_wrapper.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"
#include <zephyr/kernel.h>

/* C++ exception stubs — Zephyr disables exceptions but libstdc++ headers
 * still reference these symbols via std::function / std::vector.
 * Provide weak no-op implementations that halt on unexpected calls.       */
namespace std {
    void __throw_bad_function_call()              { k_panic(); }
    void __throw_length_error(const char *)       { k_panic(); }
    void __throw_out_of_range_fmt(const char *,
                                  ...)            { k_panic(); }
    void __throw_bad_alloc()                      { k_panic(); }
}

/* Module state filled before each inference */
static const int16_t *g_ring     = nullptr;
static uint32_t       g_head     = 0;
static uint32_t       g_filled   = 0;
static uint32_t       g_n_frames = 0;
static uint32_t       g_frame_sz = 0;

/*
 * Edge Impulse signal callback.
 * Maps a linear [0, EI_CLASSIFIER_RAW_SAMPLE_COUNT) offset to the circular
 * pre-roll ring.  Missing samples at the start are zero-padded (first wake
 * before ring is fully filled).
 */
static int get_signal_data(size_t offset, size_t length, float *out_ptr)
{
    uint32_t available   = g_filled * g_frame_sz;
    uint32_t needed      = EI_CLASSIFIER_RAW_SAMPLE_COUNT; /* 16000 */
    uint32_t pad_samples = (available < needed) ? (needed - available) : 0U;

    for (size_t i = 0; i < length; i++) {
        size_t abs_idx = offset + i;
        if (abs_idx < pad_samples) {
            out_ptr[i] = 0.0f; /* zero-pad silence at start */
        } else {
            size_t ring_idx   = abs_idx - pad_samples;
            size_t frame_idx  = ring_idx / g_frame_sz;
            size_t sample_idx = ring_idx % g_frame_sz;
            /* oldest slot = (head + 0) % n_frames */
            size_t ring_frame = (g_head + frame_idx) % g_n_frames;
            out_ptr[i] = (float)g_ring[ring_frame * g_frame_sz + sample_idx]
                         / 32768.0f;
        }
    }
    return 0;
}

extern "C" int ei_classify(const int16_t *ring_flat,
                           uint32_t head, uint32_t filled,
                           uint32_t n_frames, uint32_t frame_size,
                           ei_kws_result_t *out)
{
    if (!ring_flat || !out) { return -1; }

    g_ring     = ring_flat;
    g_head     = head;
    g_filled   = filled;
    g_n_frames = n_frames;
    g_frame_sz = frame_size;

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data     = get_signal_data;

    ei_impulse_result_t result = {};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    if (err != EI_IMPULSE_OK) { return -(int)err; }

    /* Find highest-confidence label */
    out->label = result.classification[0].label;
    out->score = result.classification[0].value;
    for (int i = 1; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > out->score) {
            out->label = result.classification[i].label;
            out->score = result.classification[i].value;
        }
    }
    return 0;
}
