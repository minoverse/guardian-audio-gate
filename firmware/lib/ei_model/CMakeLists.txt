cmake_minimum_required(VERSION 3.13.1)

if(NOT TARGET app)
    message(FATAL_ERROR "Please create a target named 'app' (ex: add_executable(app)) before adding this file")
endif()

include(edge-impulse-sdk/cmake/utils.cmake)

set(EI_SDK ${CMAKE_CURRENT_SOURCE_DIR}/edge-impulse-sdk)

# Collect EI SDK C++/CC sources — EXCLUDE CMSIS/ entirely.
# Zephyr already provides CMSIS-DSP via modules/lib/cmsis-dsp.
# Including EI's copy causes duplicate symbol linker errors.
RECURSIVE_FIND_FILE_EXCLUDE_DIR(EI_CPP_SOURCES "${EI_SDK}" "CMSIS" "*.cpp")
RECURSIVE_FIND_FILE_EXCLUDE_DIR(EI_CC_SOURCES  "${EI_SDK}" "CMSIS" "*.cc")

# TFLite micro requires this one C file (outside CMSIS)
set(EI_C_SOURCES "${EI_SDK}/tensorflow/lite/c/common.c")

# CMSIS-NN: neural-network ops used by TFLite micro.
# NOT provided by Zephyr's CONFIG_CMSIS_DSP — safe to include.
RECURSIVE_FIND_FILE_APPEND(EI_C_SOURCES "${EI_SDK}/CMSIS/NN/Source" "*.c")

# Generated tflite-model sources from Edge Impulse export
RECURSIVE_FIND_FILE_APPEND(EI_MODEL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/tflite-model" "*.cpp")

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(app PRIVATE
    ${EI_CPP_SOURCES}
    ${EI_CC_SOURCES}
    ${EI_C_SOURCES}
    ${EI_MODEL_SOURCES}
)
