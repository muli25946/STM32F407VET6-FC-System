cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FC_v2_freeRTOS)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined library search paths
)

# 设备源文件
file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/bsp/Src/*.c
    ${CMAKE_SOURCE_DIR}/modules/Src/*.c
    ${CMAKE_SOURCE_DIR}/devices/Src/*.c
)

# DSP sources
file(GLOB_RECURSE CMSIS_DSP_SOURCES
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user sources here
    ${SOURCES}
    ${CMSIS_DSP_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/bsp/Inc
    ${CMAKE_SOURCE_DIR}/modules/Inc
    ${CMAKE_SOURCE_DIR}/devices/Inc

    # CMSIS Core + DSP
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Core/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include

    # freeRTOS相关库
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/RTOS/Include
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined symbols
    ARM_MATH_CM4
    ARM_MATH_FLOAT32
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 添加数学库
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -ffunction-sections -fdata-sections
)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -Wl,--gc-sections
)
