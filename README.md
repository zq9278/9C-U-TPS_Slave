# 9C-U-TPS_Slave Firmware (STM32G070 + FreeRTOS)

This repository contains the firmware for the SAT01 control board, targeting STM32G070 (Cortex‑M0+), built with FreeRTOS and a clean task‑oriented architecture. Business UART (USART2) communicates with the host (e.g., RK3576), while a debug UART (USART1) emits logs.

The codebase replaces legacy monolithic logic with clear responsibilities:
- CommTask: unified UART Rx parsing + Tx queuing
- AppTask: device state machine, command handling, config synthesis
- SensorTask: pressure and temperature sampling into a single writer buffer
- ControlTask: PID control and pressure waveform generation
- StorageTask: AT24C02 settings load/save and boot broadcast
- SafetyTask: basic over‑limit checks → alarm path
- IoTask: LED heartbeat (buttons can be added)

All new code and comments are UTF‑8.


## Quick Start

- Toolchain: STM32CubeIDE bundles GCC 13.3.1; Ninja + CMake (via cube‑cmake) are used by VS Code.
- Target: STM32G070xx, FreeRTOS provided by STM32Cube.

Build (VS Code):
1. Open the folder and select CMake preset `Debug`.
2. Build target `SAT01_Ctrl_V1_0`.
3. Flash with your usual STM32 method (ST‑Link, openocd, or CubeProgrammer).

Build (CLI example):
```
# From repository root
cube-cmake -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -S . -B build/Debug -G Ninja
cube-cmake --build build/Debug
```

Notes:
- If linking printf float fails, ensure the project does not set raw `-u _printf_float` directly in CMake command context (we already fixed the related parse issue). Use proper `target_link_options()` formatting if needed.
- This project uses FreeRTOS vTaskDelay (not vTaskDelayUntil). Ensure headers include FreeRTOS/task.h where used.


## Directory Layout

- `Application/AppMain/`
  - `AppMain.c`: sets up queues and creates tasks; initializes both UART ports
  - `system_app.h`: defines shared data, command structures, and queue handles
- `Application/Tasks/`
  - `comm_task.c`: drains UART Rx queue, calls parser, and sends Tx frames; drains debug log queue
  - `app_task.c`: translates app_cmd_t to control configurations; maintains app state; debounced save; handles safety events
  - `sensor_task.c`: samples pressure via I2C and temperature via SPI/RTD; writes `gSensorData`
  - `control_task.c`: PID and waveform; operates valves and pump; sends telemetry every 100 ms
  - `storage_task.c`: load defaults/EEPROM on boot and broadcast; saves parameters on request
  - `io_task.c`: LED heartbeat
  - `safety_task.c`: simple temperature/pressure threshold checks → `gSafetyQueue`
- `Application/Uart_Module/`
  - `uart_driver.c/.h`: UART abstraction for business/debug ports, frame packing/unpacking, CRC16, DMA+IDLE Rx
  - `Uart_Communicate.c/.h`: frame ID definitions and dispatch: frame → `app_cmd_t` (no direct hardware control)
  - `LOG.c/.h`: debug UART logging helper
- `Application/Config/`
  - `config.c/.h`: settings structure, defaults, CRC16, EEPROM load/save, boot broadcast to host
- `Application/` other legacy modules are retained for drivers (heat, PID, sensors, etc.)
- `Core/` and `Drivers/`: CubeMX generated HAL and FreeRTOS code


## Runtime Architecture

- Queues (declared in `Application/AppMain/system_app.h`):
  - `gCmdQueue` (Comm → App): host commands → `app_cmd_t`
  - `gCtrlCmdQueue` (App → Control): start/stop/update config → `ctrl_cmd_t`
  - `gTxQueue` (any → Comm): telemetry frames → `tx_frame_t`
  - `gStorageQueue` (App → Storage): load/save settings
  - `gSafetyQueue` (Safety → App): alarm flags

- Shared data:
  - `gSensorData` (single writer: SensorTask)
  - `gAppState` (App state; read by others if needed)

- Task responsibilities:
  - CommTask
    - Input: `rk3576_uart_port.rx_queue` (DMA+IDLE) → `parse_rk3576_uart_port_stream()`
    - Output: drains `gTxQueue` and sends via `rk3576_uart_port.sender()`
    - Debug: drains `debug_uart_port.tx_queue` and transmits
  - AppTask
    - Consumes `gCmdQueue` (host control), updates `g_settings`, and issues control configs
    - Debounced persistence (`STORAGE_CMD_SAVE_PARAM` after ~3s idle)
    - Responds to safety events (emergency stop, set `APP_STATE_ALARM`)
  - SensorTask (10 ms)
    - I2C pressure sensors → kPa; SPI ADS1248 → °C
    - Writes `gSensorData` and time stamp
  - ControlTask (10 ms)
    - Pressure waveform: ramp (T1), hold (T2), pulse (T3 Ton/Toff)
    - PID control for temperature and pressure; operates valves and pump (TIM15->CCR1)
    - Telemetry (100 ms): left/right pressures and temperatures
  - StorageTask
    - `Config_Init()`; if load fails → defaults, then save
    - `Settings_Broadcast()` after load
  - SafetyTask (100 ms)
    - Over‑temperature/pressure checks → `gSafetyQueue`
  - IoTask
    - LED heartbeat


## UART Protocol

- Frame layout (`Application/Uart_Module/uart_driver.h`):
  - Header: `0xAA 0x55`
  - `frame_id` (u16, LE)
  - `data_type` (u8): one of `{1=UINT8, 2=FLOAT, 3=TEXT, 4=UINT16, 5=UINT32}`
  - `data_length` (u16, LE)
  - `data` (0..32 bytes)
  - CRC16(Modbus) over: `frame_id | data_type | data_length | data`
  - Tail: `0x0D 0x0A`

- Direction and IDs (`Application/Uart_Module/Uart_Communicate.h`):
  - Host → Device (0x1000–0x10FF): control and configuration
  - Device → Host (0x1100–0x11FF): telemetry and status

- Core host commands (downlink):
  - `U8_HEARTBEAT_REQ` → device replies `U8_HEARTBEAT_ACK` (uint8=1)
  - Setpoints: `F32_PRESSURE_SET_KPA`, `F32_LEFT_TEMP_SET_C`, `F32_RIGHT_TEMP_SET_C`, `U8_USE_COMMON_TEMP`
  - Mode flow: `U8_MODE_SELECT` (1/2), `U8_START_TREATMENT`, `U8_STOP_TREATMENT`, `U8_SAVE_SETTINGS`
  - Mode1/2 profiles: target_kpa, t_rise_ms, t_hold_ms, t_pulse_total_ms, t_pulse_on_ms, t_pulse_off_ms, squeeze_mode

- Telemetry (uplink) examples:
  - `F32_LEFT_PRESSURE_VALUE`, `F32_RIGHT_PRESSURE_VALUE` (kPa)
  - `F32_LEFT_TEMP_VALUE`, `F32_RIGHT_TEMP_VALUE` (°C)
  - `U8_SYSTEM_STATE`, `U8_ALARM_STATE`, heater presence/fuse (reserved)

- Dispatching: `Application/Uart_Module/Uart_Communicate.c`
  - Only translates `frame_id + payload` into `app_cmd_t` over `gCmdQueue`.
  - No direct hardware control in this file.


## Settings and Persistence

- Structure (`Application/Config/config.h`):
  - `SystemSettings_t` with two `PressureProfile_t` profiles for modes 1 and 2
  - Key fields: `mode_select`, `left_temp_c`, `right_temp_c`, `use_common_temp`, per‑mode pressure targets and timings
  - CRC16(Modbus) and versioning; magic=0x9C71

- EEPROM (`Application/Config/config.c`):
  - AT24C02 read/write starting at `0x20`
  - On boot: load → if invalid, apply defaults then save
  - `Settings_Broadcast()` sends all active parameters to host


## Extending the Protocol

1. Add a new `FrameId_t` in `Application/Uart_Module/Uart_Communicate.h`.
2. In `Application/Uart_Module/Uart_Communicate.c`, map the new ID to an `app_cmd_t` and enqueue to `gCmdQueue`.
3. Handle the new command in `Application/Tasks/app_task.c` (update `g_settings` or trigger action).
4. If the device must publish something periodically or on change, push a `tx_frame_t` into `gTxQueue` and choose the corresponding uplink ID.

Keep dispatch files free from direct HAL control; use queues to reach the right task.


## Tuning and Safety

- Pressure and temperature PIDs are initialized in `Application/Tasks/control_task.c`; tune gains as needed.
- Safety limits (TEMP_MAX_C, PRESS_MAX_KPA) in `Application/Tasks/safety_task.c` can be surfaced into settings if required.
- AppTask responds to safety faults by issuing STOP and setting `APP_STATE_ALARM`.


## Troubleshooting

- Build parse errors for `-u _printf_float`: pass link options via proper `target_link_options()` rather than raw trailing arguments. This repo’s CMake already avoids the problematic pattern.
- `vTaskDelayUntil` unresolved: not enabled in current FreeRTOSConfig; use `vTaskDelay` or enable the feature.
- If UART business port is silent, confirm USART2 pins, DMA+IDLE Rx enabled, and queues are created in `Application/AppMain/AppMain.c`.


## Known Limitations / TODO

- SafetyTask provides only basic over‑limit checks; add sensor timeouts and stuck‑valve logic.
- Heater presence/fuse telemetry is reserved; implement actual detection or remove IDs.
- If both eyes share pump and valves, alternate mode splits effort; a dual‑pump design would allow independent control.
- Some legacy code remains for drivers; keep control confined to the ControlTask.


## License

Internal project. Do not distribute without permission.

