#!/usr/bin/env python3
import numpy as np
import scipy.signal as sig

def generate_cmsis_resonator_coefs(center_freqs=[300, 800, 1500, 2500], fs=16000, Q=8.0):
    coefs = []
    for i, fc in enumerate(center_freqs):
        b, a = sig.iirpeak(fc, Q, fs=fs)
        b = b / a[0]
        a = a / a[0]
        
        # DF1 format: {b0, b1, b2, a1, a2} (NO negation)
        cmsis_coefs_float = [b[0], b[1], b[2], a[1], a[2]]
        cmsis_coefs_q15 = np.clip(np.round(np.array(cmsis_coefs_float) * 32768), -32768, 32767).astype(np.int16)
        
        coefs.append({'fc': fc, 'bandwidth': fc/Q, 'q15': cmsis_coefs_q15.tolist()})
        print(f"Ch{i}: {fc}Hz -> {cmsis_coefs_q15.tolist()}")
    return coefs

def export_cmsis_header(coefs, filename='resonator_coefs_cmsis.h'):
    with open(filename, 'w') as f:
        f.write("#ifndef RESONATOR_COEFS_CMSIS_H\n#define RESONATOR_COEFS_CMSIS_H\n#include <stdint.h>\n\n")
        f.write(f"#define NUM_RESONATORS {len(coefs)}\n#define NUM_STAGES 1\n\n")
        
        for i, c in enumerate(coefs):
            f.write(f"static const int16_t RESONATOR_{i}_COEFS[5] = {{{c['q15'][0]}, {c['q15'][1]}, {c['q15'][2]}, {c['q15'][3]}, {c['q15'][4]}}};\n")
        
        f.write("\nstatic const int16_t* const RESONATOR_COEFS[NUM_RESONATORS] = {")
        f.write(", ".join(f"RESONATOR_{i}_COEFS" for i in range(len(coefs))))
        f.write("};\n\n")
        
        f.write("static const uint16_t RESONATOR_CENTER_FREQS[NUM_RESONATORS] = {")
        f.write(", ".join(str(c['fc']) for c in coefs))
        f.write("};\n\n#endif\n")

if __name__ == '__main__':
    coefs = generate_cmsis_resonator_coefs()
    export_cmsis_header(coefs, '../../firmware/lib/guardian_dsp/include/resonator_coefs_cmsis.h')
    print("Done")
