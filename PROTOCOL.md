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

- `0x1000` U8_HEARTBEAT_REQ — 心跳请求 (host keeps sending 1)
- `0x1001` F32_PRESSURE_SET_KPA — 眼盾压力设定 (float kPa)
- `0x1002` F32_LEFT_TEMP_SET_C — 眼盾温度设定 (float °C，左右共用)
- `0x10C0` U8_MODE_SELECT — 曲线模式选择 (1/2/3/4)
- `0x10C1` U8_START_TREATMENT — 启动治疗 (1=start)
- `0x10C2` U8_STOP_TREATMENT — 停止治疗
- `0x10C3` U8_SAVE_SETTINGS — 保存当前参数
- `0x1004` U8_LEFT_EYE_ENABLE — 左眼功能开关 (0关/1开)
- `0x1005` U8_RIGHT_EYE_ENABLE — 右眼功能开关 (0关/1开)

### Device → Host (telemetry)

- `0x1100` U8_HEARTBEAT_ACK — 心跳应答 (固定 1，每 100 ms)
- `0x1101` F32_LEFT_PRESSURE_VALUE — 左眼压力反馈 (kPa)
- `0x1102` F32_RIGHT_PRESSURE_VALUE — 右眼压力反馈 (kPa)
- `0x1103` F32_LEFT_TEMP_VALUE — 左眼温度反馈 (°C)
- `0x1104` F32_RIGHT_TEMP_VALUE — 右眼温度反馈 (°C)
- `0x1107` U8_LEFT_HEATER_PRESENT — 左眼盾存在 (0/1)
- `0x1108` U8_RIGHT_HEATER_PRESENT — 右眼盾存在 (0/1)
- `0x1109` U8_LEFT_HEATER_FUSE — 左眼盾熔断状态 (0/1)
- `0x110A` U8_RIGHT_HEATER_FUSE — 右眼盾熔断状态 (0/1)

## Control Behaviour

- 目标温度: `F32_LEFT_TEMP_SET_C` (°C)
- 目标压力: `F32_PRESSURE_SET_KPA` (kPa)
- 左/右眼功能: `U8_LEFT_EYE_ENABLE` / `U8_RIGHT_EYE_ENABLE` (0=关闭, 1=开启)
- 治疗开始: `U8_START_TREATMENT = 1`，停止: `U8_STOP_TREATMENT` 或左右眼均关闭
- 下位机维持设定压力（单泵 + 双阀），并根据开关状态控制加热/气压
- 心跳: 设备每 100 ms 主动发送 `U8_HEARTBEAT_ACK = 1`

## Typical Flow

1) Host → `0x1000` heartbeat; Device → `0x1100 = 1`
2) (Optional) set temperatures: `0x1002`/`0x1003`, and `0x10C4` for common temp
3) 设置温度 `0x1002`、压力 `0x1001`
4) 分别打开左/右眼功能 `0x1004` / `0x1005`
5) `0x10C1 = 1` 启动
6) 倒计时结束时发送 `0x1004/0x1005 = 0` 关闭对应眼盾，或发送 `0x10C2` 全部停止
7) 如需保存本次设定，发送 `0x10C3`

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
