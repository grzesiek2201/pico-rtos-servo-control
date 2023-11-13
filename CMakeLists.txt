# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.14)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

set(PICO_BOARD pico_w)

# Set name of project (as PROJECT_NAME) and C/CXX standards
project(serial_servo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Enable exceptions
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME}
    src/main.cpp
)

FILE(GLOB FreeRTOS_src FreeRTOS-Kernel/*.c)

add_library( FreeRTOS STATIC
	${FreeRTOS_src}
	FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
	FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

target_include_directories( FreeRTOS PUBLIC
	FreeRTOS-Kernel/include
	include/
	FreeRTOS-Kernel/portable/GCC/ARM_CM0
)

add_subdirectory(include/pca9685)

target_include_directories(${PROJECT_NAME} PRIVATE
    include/pca9685
)

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} PRIVATE
    pico_stdlib   # already in the pca9685 library with PUBLIC 
    hardware_i2c  # already in the pca9685 library with PUBLIC 
    pico_cyw43_arch_none
    FreeRTOS
    pca9685
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
