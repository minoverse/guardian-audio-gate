cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(guardian_audio_gate)

# Add Guardian DSP library
add_subdirectory(lib/guardian_dsp)

# Main application source
target_sources(app PRIVATE
    src/main.c
)

# Link Guardian library
target_link_libraries(app PRIVATE guardian_dsp)
