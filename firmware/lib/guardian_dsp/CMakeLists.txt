zephyr_library_named(guardian_dsp)

zephyr_library_sources(
    src/resonator_df2t.c
    src/features/energy.c
    src/features/correlation.c
    src/features/coherence.c
    src/features/zcr.c
    src/features/flux.c
    src/features/spectral_flatness.c
    src/features/modulation.c
    src/audio/dma_pdm.c
    src/audio/preprocess.c
    src/audio/mic_cal.c
)

zephyr_include_directories(include)
