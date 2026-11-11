set(BOOT_HAL ${CMAKE_SOURCE_DIR}/Drivers/STM32G0xx_HAL_Driver)
add_executable(tps_boot
    Bootloader/boot_main.c Bootloader/boot_engine.c Shared/Upgrade/upgrade.c
    startup_stm32g070xx.s Core/Src/system_stm32g0xx.c)
foreach(module hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_uart hal_uart_ex hal_flash hal_flash_ex hal_pwr hal_pwr_ex hal_dma hal_dma_ex)
    target_sources(tps_boot PRIVATE ${BOOT_HAL}/Src/stm32g0xx_${module}.c)
endforeach()
target_include_directories(tps_boot PRIVATE ${CMAKE_SOURCE_DIR} Core/Inc ${BOOT_HAL}/Inc
    Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32G0xx/Include)
target_compile_definitions(tps_boot PRIVATE USE_HAL_DRIVER STM32G070xx UP_HW_REV=${TPS_HW_REV})
target_compile_options(tps_boot PRIVATE -Os -Wall -Wextra)
if(TPS_RTT)
    target_sources(tps_boot PRIVATE ThirdParty/SEGGER_RTT/RTT/SEGGER_RTT.c)
    target_include_directories(tps_boot PRIVATE ThirdParty/SEGGER_RTT/RTT ThirdParty/SEGGER_RTT/Config)
    target_compile_definitions(tps_boot PRIVATE TPS_RTT=1)
endif()
target_link_libraries(tps_boot PRIVATE m)
add_custom_command(TARGET tps_boot POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:tps_boot> ${CMAKE_BINARY_DIR}/tps_boot.bin)
