/* biquad_df1.c — Portable Direct Form I biquad block processing
 *
 * Pure C99 — compiles on any platform with an IEEE 754 FPU.
 * On Cortex-M4F with -O2 the compiler auto-issues VMLA/VMLS instructions
 * for every iteration; no manual SIMD intrinsics needed.
 *
 * Copyright (c) 2026 Guardian Audio. All rights reserved.
 * SPDX-License-Identifier: Proprietary                                  */

#include "guardian/sdk/biquad_df1.h"

void gd_biquad_df1_block(gd_biquad_df1_t *f,
                         const float *in, float *out, size_t len)
{
    /* Cache state in locals — avoids repeated pointer dereferences inside
     * the hot loop, which can matter on in-order pipelines (M0/M0+).    */
    float b0  = f->b0,  b1  = f->b1,  b2  = f->b2;
    float na1 = f->na1, na2 = f->na2;
    float x1  = f->x1,  x2  = f->x2;
    float y1  = f->y1,  y2  = f->y2;

    for (size_t i = 0; i < len; i++) {
        float x = in[i];
        float y = b0 * x + b1 * x1 + b2 * x2
                + na1 * y1 + na2 * y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        out[i] = y;
    }

    f->x1 = x1; f->x2 = x2;
    f->y1 = y1; f->y2 = y2;
}
