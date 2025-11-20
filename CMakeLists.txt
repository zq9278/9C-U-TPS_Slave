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
set(CMAKE_PROJECT_NAME SAT01_Ctrl_V1_0)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Application sources
    Application/24C02/24cxx.c
    Application/ads1248/ads1248.c
    # Application/apply/apply.c  # excluded: legacy state machine replaced by ControlTask
    Application/AppMain/AppMain.c
    Application/AppMain/mode_curves.c
    Application/Config/config.c
    Application/Tasks/comm_task.c
    Application/Tasks/app_task.c
    Application/Tasks/sensor_task.c
    Application/Tasks/control_task.c
    Application/Tasks/storage_task.c
    Application/Tasks/io_task.c
    Application/Tasks/safety_task.c
    Application/delay/delay.c
    Application/ds18b20/ds18b20.c
    Application/heat/heat.c
    Application/pid/pid.c
    Application/Pressure_sensor/Pressure_sensor.c
    Application/SYS/led.c
    Application/SYS/myiic.c
    Application/SYS/sys.c
    Application/SYS/myuart.c
    Application/Uart_Module/Uart_Communicate.c
    Application/Uart_Module/LOG.c
    Application/Uart_Module/uart_driver.c
    #Application/Uart_Module/Uart_Task.c
    Application/USER_TIM/USER_TIM.c
    Application/water/water.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Application include directories
    Application
    Application/24C02
    Application/ads1248
    Application/apply
    Application/AppMain
    Application/Config
    Application/Tasks
    Application/delay
    Application/ds18b20
    Application/heat
    Application/pid
    Application/Pressure_sensor
    Application/SYS
    Application/Uart_Module
    Application/USER_TIM
    Application/water
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Force-include typedefs to map legacy u8/u16/u32/s16 to stdint types
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-include ${CMAKE_SOURCE_DIR}/Application/SYS/types_compat.h>
)
# 启用 printf 的浮点格式支持（newlib-nano 默认禁用）
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)
