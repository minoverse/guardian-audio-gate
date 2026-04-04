cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(guardian_audio_gate)

# Add Guardian DSP library
add_subdirectory(lib/guardian_dsp)

# Add Guardian Gate library (Week 6 decision logic)
add_subdirectory(lib/guardian_gate)

# Add Guardian SDK — portable C99 pre-filter (no CMSIS/Zephyr in public API)
add_subdirectory(lib/guardian_sdk)

# Add Edge Impulse model (Week 15-16 real TinyML)
add_subdirectory(lib/ei_model)

# Edge Impulse needs C++ standard library headers (functional, algorithm, etc.)
# The Zephyr build system doesn't add them by default — find them from the compiler.
execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
    OUTPUT_VARIABLE GCC_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
get_filename_component(GCC_BIN_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)
get_filename_component(TOOLCHAIN_ROOT ${GCC_BIN_DIR} DIRECTORY)
set(CXX_STD_INCLUDE "${TOOLCHAIN_ROOT}/arm-zephyr-eabi/include/c++/${GCC_VERSION}")
target_include_directories(app PRIVATE
    ${CXX_STD_INCLUDE}
    ${CXX_STD_INCLUDE}/arm-zephyr-eabi
)

# ── Fault injection: compile-time guard ──────────────────────────────────────
# Off by default → test helper functions excluded from production binary.
# Enable for fault tolerance testing:
#   west build ... -- -DFAULT_INJECT=ON
option(FAULT_INJECT "Enable fault injection test helpers" OFF)
if(FAULT_INJECT)
    target_compile_definitions(app      PRIVATE FAULT_INJECT_ENABLED=1)
    target_compile_definitions(guardian_dsp PRIVATE FAULT_INJECT_ENABLED=1)
    message(STATUS "Guardian: FAULT_INJECT enabled — test helpers compiled in")
endif()

# Main application source
target_sources(app PRIVATE
    src/main.c
    src/ei_classifier_wrapper.cpp
)

# Link Guardian libraries
target_link_libraries(app PRIVATE guardian_dsp guardian_gate guardian_sdk)
