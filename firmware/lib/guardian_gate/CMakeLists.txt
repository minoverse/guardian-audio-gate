zephyr_library_named(guardian_gate)

zephyr_library_sources(
    src/decision.c
)

zephyr_include_directories(include)

# Needs guardian_dsp for types and feature functions
zephyr_library_link_libraries(guardian_dsp)
