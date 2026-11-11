cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

option(TPS_BUILD_BOOT "Build standalone serial Bootloader" OFF)
option(TPS_LEGACY_APP "Build original SWD-only application at Flash base" OFF)
set(TPS_HW_REV 1 CACHE STRING "Production board hardware revision")
option(TPS_RTT "Enable non-blocking J-Link RTT diagnostics" ON)
include(Config/version.cmake)
if(NOT TPS_APP_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+(\\.[0-9]+)?$")
    message(FATAL_ERROR "Invalid TPS_APP_VERSION")
endif()
string(REPLACE "." ";" TPS_VERSION_PARTS "${TPS_APP_VERSION}")
list(LENGTH TPS_VERSION_PARTS TPS_VERSION_COUNT)
if(TPS_VERSION_COUNT EQUAL 3)
    list(APPEND TPS_VERSION_PARTS 0)
endif()
foreach(PART IN LISTS TPS_VERSION_PARTS)
    if(PART GREATER 255)
        message(FATAL_ERROR "Version component exceeds 255")
    endif()
endforeach()
list(GET TPS_VERSION_PARTS 0 TPS_VERSION_MAJOR)
list(GET TPS_VERSION_PARTS 1 TPS_VERSION_MINOR)
list(GET TPS_VERSION_PARTS 2 TPS_VERSION_PATCH)
list(GET TPS_VERSION_PARTS 3 TPS_VERSION_BUILD)
math(EXPR TPS_APP_VERSION_CODE "(${TPS_VERSION_MAJOR} << 24) | (${TPS_VERSION_MINOR} << 16) | (${TPS_VERSION_PATCH} << 8) | ${TPS_VERSION_BUILD}")
configure_file(Config/firmware_version.h.in ${CMAKE_BINARY_DIR}/generated/firmware_version.h @ONLY)
if(NOT TPS_HW_REV MATCHES "^[0-9]+$" OR TPS_HW_REV GREATER 65535)
    message(FATAL_ERROR "TPS_HW_REV must be 0..65535")
endif()
if(TPS_BUILD_BOOT)
    set(TPS_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/Bootloader/boot.ld")
elseif(TPS_LEGACY_APP)
    set(TPS_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32G070XX_FLASH.ld")
else()
    set(TPS_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/App/app.ld")
endif()
# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 9C-TPS100-Slave)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)
if(TPS_BUILD_BOOT)
    include(Bootloader/build.cmake)
    return()
endif()

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
    Shared/Upgrade/upgrade.c
    Shared/Upgrade/firmware_identity.c
    Modules/Debug/debug_rtt.c

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

    # Modules
    Modules/Log/module_log.c
    Modules/communication/communication.c
    Modules/communication/Protocol/protocol_common.c
    Modules/communication/Protocol/pid_debug_protocol.c
    Modules/communication/Protocol/rk3576_protocol.c
    Modules/Pid/pid_controller.c
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
    ${CMAKE_BINARY_DIR}/generated
    ThirdParty/SEGGER_RTT/RTT ThirdParty/SEGGER_RTT/Config
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
    UP_HW_REV=${TPS_HW_REV}
    TPS_RTT=$<BOOL:${TPS_RTT}>
)
if(TPS_RTT)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE ThirdParty/SEGGER_RTT/RTT/SEGGER_RTT.c)
endif()
if(NOT TPS_LEGACY_APP)
    target_compile_definitions(STM32_Drivers PRIVATE TPS_APP_OFFSET=0x4000U)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TPS_APP_UPGRADE=1)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_BINARY_DIR}/tps_app.bin)
endif()

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
