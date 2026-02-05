cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(guardian_audio_gate)

target_sources(app PRIVATE
    src/resonator_df2t.c
)

target_include_directories(app PRIVATE
    include/guardian
)
