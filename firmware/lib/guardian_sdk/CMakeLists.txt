# guardian_sdk — Portable Physics Pre-Filter SDK
#
# Pure C99. Zero CMSIS, Zephyr, or platform dependencies.
# Links math (m) for sqrtf/logf/expf — available on all hosted platforms;
# on Cortex-M4F Zephyr provides these via newlib (already linked).
#
# Integration:
#   Zephyr (in-tree):  add_subdirectory(lib/guardian_sdk)  in CMakeLists.txt
#                      target_link_libraries(app PRIVATE guardian_sdk)
#
#   Standalone CMake:  add_subdirectory(guardian_sdk)
#                      target_link_libraries(your_target PRIVATE guardian_sdk m)

cmake_minimum_required(VERSION 3.20)

# ── Detect build environment ──────────────────────────────────────────
if(DEFINED CONFIG_ZEPHYR_BASE OR DEFINED ZEPHYR_BASE)
    # ── Zephyr in-tree build ─────────────────────────────────────────
    zephyr_library_named(guardian_sdk)

    zephyr_library_sources(
        src/biquad_df1.c
        src/prefilter.c
    )

    zephyr_include_directories(include)

else()
    # ── Standalone CMake (host tests, Qualcomm SDK, etc.) ───────────
    project(guardian_sdk C)

    add_library(guardian_sdk STATIC
        src/biquad_df1.c
        src/prefilter.c
    )

    target_include_directories(guardian_sdk
        PUBLIC  include
        PRIVATE src      # for resonator_coefs_default.h
    )

    target_compile_features(guardian_sdk PRIVATE c_std_99)

    # math.h functions: linked automatically on most platforms.
    # Explicit -lm needed on Linux GCC.
    target_link_libraries(guardian_sdk PUBLIC m)

    # ── Optional: host unit tests ────────────────────────────────────
    option(GUARDIAN_SDK_BUILD_TESTS "Build SDK unit tests" OFF)
    if(GUARDIAN_SDK_BUILD_TESTS)
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()
