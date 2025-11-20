# UART Protocol Specification (Host ⇄ Device)

- MCU: STM32G070 (FreeRTOS)
- Business UART: USART2 (PD5 TX, PD6 RX), 115200 8N1, DMA + IDLE receive
- Debug UART: USART1 (logs only)
- Device telemetry period: 100 ms (pressure/temperature)

## Frame Format

- Byte layout: `0xAA 0x55 | frame_id(LE,2) | data_type(1) | data_len(LE,2) | payload | crc16(LE,2) | 0x0D 0x0A`
- `data_type` values:
  - `0x01` uint8 (1 byte)
  - `0x02` float (IEEE754, little‑endian, 4 bytes)
  - `0x03` text (ASCII/UTF‑8, <=32 bytes)
  - `0x04` uint16 (little‑endian, 2 bytes)
  - `0x05` uint32 (little‑endian, 4 bytes)
- `crc16` is Modbus CRC16 over the bytes from `frame_id` through the end of `payload` (i.e., `2 + 1 + 2 + data_len` bytes). Polynomial 0xA001, init 0xFFFF. CRC is little‑endian (low byte first).

## Frame IDs

### Host → Device (control)

- Heartbeat and utility
  - `0x1000` U8_HEARTBEAT_REQ — host sends any uint8; device replies `0x1100 = 1`
  - `0x10F0` FRAME_ID_TEXT — optional debug text, device logs only
- Treatment flow (single mode)
  - `0x10C0` U8_MODE_SELECT — ignored (single fixed mode); keep value=1
  - `0x10C1` U8_START_TREATMENT — uint8 1=start (0 is equivalent to stop)
  - `0x10C2` U8_STOP_TREATMENT — stop current treatment
  - `0x10C3` U8_SAVE_SETTINGS — persist current settings (temperatures, mode select)
- Temperatures
  - `0x1002` F32_LEFT_TEMP_SET_C — float left target °C (persisted)
  - `0x1003` F32_RIGHT_TEMP_SET_C — float right target °C (persisted)
  - `0x10C4` U8_USE_COMMON_TEMP — uint8 0/1, 1=use left temp for both
  - `0x1006` U8_LEFT_TEMP_ENABLE — uint8 0/1, runtime only (heater off when 0)
  - `0x1007` U8_RIGHT_TEMP_ENABLE — uint8 0/1, runtime only
- Pressure side enables and squeeze mode
  - `0x1004` U8_LEFT_PRESSURE_ENABLE — uint8 0/1, runtime only
  - `0x1005` U8_RIGHT_PRESSURE_ENABLE — uint8 0/1, runtime only
  - `0x100B` U8_SQUEEZE_MODE — uint8 0=normal/sync, 1=alternate, 2=sync (treated as 0)
- Reserved/ignored (kept for compatibility)
  - `0x1001` F32_PRESSURE_SET_KPA — ignored (device uses built‑in curves)
  - `0x1008` U8_PUMP_POWER_VALUE — ignored (closed‑loop control)
  - `0x1009` U32_HOLD_TIME_MS — ignored in read‑only curve build
  - `0x100A` U32_RELEASE_TIME_MS — ignored

### Device → Host (telemetry)

- `0x1100` U8_HEARTBEAT_ACK — value=1
- `0x1101` F32_LEFT_PRESSURE_VALUE — left pressure, kPa
- `0x1102` F32_RIGHT_PRESSURE_VALUE — right pressure, kPa
- `0x1103` F32_LEFT_TEMP_VALUE — left temperature, °C
- `0x1104` F32_RIGHT_TEMP_VALUE — right temperature, °C
- `0x1105` U8_SYSTEM_STATE — reserved
- `0x1106` U8_ALARM_STATE — reserved
- `0x1107..0x110A` heater presence/fuse — reserved

## Execution Curve (single mode)

The device runs a single built‑in curve (defaults in `Application/AppMain/mode_curves.c`) with six parameters. Host can update the runtime values via the setters below; no EEPROM persistence.

- target_kpa — pressure target (kPa)
- t1_rise_s — slow rise duration (seconds)
- t2_hold_s — constant hold duration (seconds)
- t3_pulse_s — total pulse window (seconds)
- pulse_on_ms — pulse on (milliseconds)
- pulse_off_ms — pulse off (milliseconds)

Phases:
- Slow rise (t1): linear ramp to target_kpa
- Constant hold (t2): maintain target_kpa
- Pulse (t3): alternate on/off windows; squeeze_mode=1 alternates eyes, else sync

Runtime setters (optional):
- `0x10D0` F32_MODE1_TARGET_KPA (key=0)
- `0x10D1` U32_MODE1_T_RISE_MS  (key=1)
- `0x10D2` U32_MODE1_T_HOLD_MS  (key=2)
- `0x10D3` U32_MODE1_T_PULSE_TOTAL_MS (key=3)
- `0x10D4` U16_MODE1_T_PULSE_ON_MS (key=4)
- `0x10D5` U16_MODE1_T_PULSE_OFF_MS (key=5)
Notes: updates take effect immediately if running (device issues CTRL_CMD_UPDATE_CFG internally).

Valve/Pump:
- Single pump (TIM15 CH1 PWM 0..255)
- Two solenoid valves: right (AirValve1), left (AirValve2)
- When only one side demands pressure, open only that side’s valve; when both demand, open both; otherwise close both and pump off.

Side enables:
- Pressure enables (U8_LEFT/RIGHT_PRESSURE_ENABLE): when 0, that side’s setpoint is forced to 0 (release)
- Temperature enables (U8_LEFT/RIGHT_TEMP_ENABLE): when 0, heater PWM=0 for that side

## Typical Flow

1) Host → `0x1000` heartbeat; Device → `0x1100 = 1`
2) (Optional) set temperatures: `0x1002`/`0x1003`, and `0x10C4` for common temp
3) (Optional) tweak runtime curve with 0x10D0..0x10D5
4) Start treatment `0x10C1 = 1`
5) Monitor telemetry every ~100 ms: `0x1101..0x1104`
6) Adjust runtime switches as needed: `0x1004/0x1005/0x1006/0x1007/0x100B`
7) Stop `0x10C2` or `0x10C1 = 0`

## Examples (hex)

- Select mode 1
```
AA 55  C0 10  01  01 00  01  CRClo CRChi  0D 0A
```
- Start (value=1)
```
AA 55  C1 10  01  01 00  01  CRClo CRChi  0D 0A
```
- Set left temp = 38.0°C (float LE = 00 00 19 42)
```
AA 55  02 10  02  04 00  00 00 19 42  CRClo CRChi  0D 0A
```

## Data/Constraints

- Max payload length: 32 bytes
- Multi‑byte values are little‑endian
- CRC16(Modbus), poly 0xA001, init 0xFFFF
- Device discards frames with CRC mismatch or invalid length

## Implementation Pointers

- Business port framing and CRC: `Application/Uart_Module/uart_driver.[ch]`
- Command dispatch: `Application/Uart_Module/Uart_Communicate.c`
- Control loop (10 ms tick): `Application/Tasks/control_task.c`
- Built‑in curves: `Application/AppMain/mode_curves.c`
- Settings persistence (EEPROM): `Application/Config/config.[ch]`
