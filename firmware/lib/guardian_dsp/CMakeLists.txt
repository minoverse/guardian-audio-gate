zephyr_library_named(guardian_dsp)

zephyr_library_sources(
    src/resonator_df2t.c
    src/features/energy.c
    src/features/correlation.c
    src/features/coherence.c
    src/features/zcr.c
    src/features/flux.c
)

zephyr_include_directories(include)
