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
    # App / RTOS architecture
    App/System/app_main.c
    App/System/app_tasks.c
    App/System/app_rtos_hooks.c

    # BSP / drivers
    BSP/Gpio/bsp_gpio.c
    BSP/I2c/bsp_i2c.c
    BSP/I2c/bsp_i2c_hal_adapter.c
    BSP/I2c/bsp_i2c_soft_adapter.c
    BSP/Spi/bsp_spi.c
    BSP/Spi/bsp_spi_hal_adapter.c
    BSP/Spi/bsp_spi_soft_adapter.c
    BSP/Pwm/bsp_pwm.c
    BSP/Uart/bsp_uart.c
    UserDrivers/user_drivers_board.c
    UserDrivers/Ads1248/ads1248.c
    UserDrivers/Ds18b20/ds18b20_driver.c
    UserDrivers/Eeprom24cxx/eeprom24cxx.c
    UserDrivers/PressureSensor/pressure_sensor_driver.c
    UserDrivers/ValveControl/valve_control.c
    Components/Actuators/TreatmentActuators.c

    # Modules
    Modules/Log/module_log.c
    Modules/communication/ascii_processor.c
    Modules/communication/communication.c
    Modules/communication/Protocol/protocol_common.c
    Modules/communication/Protocol/rk3576_protocol.c
    Modules/Pid/pid_controller.c
    Modules/Pid/pid_debug_protocol.c
    Modules/Filter/target_window_filter.c
    Modules/Sensors/treatment_temperature_sensor.c
    Modules/Sensors/treatment_pressure_sensor.c
    Modules/EyeShield/eye_shield_status.c
    Modules/App/treatment_app_controller.c
    Modules/Pressure/treatment_pressure_control.c
    Modules/Heat/treatment_heating_control.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}

    # App / architecture include directories
    App/System

    # Modern architecture include directories
    BSP
    BSP/Gpio
    BSP/I2c
    BSP/Pwm
    BSP/Spi
    BSP/Uart
    UserDrivers
    UserDrivers/Ads1248
    UserDrivers/Ds18b20
    UserDrivers/Eeprom24cxx
    UserDrivers/PressureSensor
    UserDrivers/ValveControl
    Components
    Components/Actuators
    Modules
    Modules/Log
    Modules/communication
    Modules/communication/Protocol
    Modules/Pid
    Modules/Filter
    Modules/Sensors
    Modules/EyeShield
    Modules/App
    Modules/Pressure
    Modules/Heat
    RTOS
    Utils
    Config
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
)
# 启用 printf 的浮点格式支持（newlib-nano 默认禁用）
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)
