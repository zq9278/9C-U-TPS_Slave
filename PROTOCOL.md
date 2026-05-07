# UART Protocol Summary

## Scope

This document summarizes the UART protocol currently implemented in the slave firmware, based on the code as it exists today. It is intended as a refactor aid, so it documents both the intended protocol and the actual runtime behavior.

## Physical Ports

- Business UART: `USART3`
  - Pins: `PD8` TX, `PD9` RX
  - Baud rate: `115200`
  - Format: `8N1`
  - RX mode: DMA + `ReceiveToIdle`
  - Main usage: host business protocol
- Debug UART: `USART1`
  - Pins: `PC4` TX, `PC5` RX
  - Baud rate: `115200`
  - Format: `8N1`
  - Main usage: logs and debug commands

## Binary Frame Format

The business UART uses a fixed binary frame:

```text
+------------+------------+---------------+-----------+-----------------+---------+-----------+
| 0xAA       | 0x55       | frame_id      | data_type | data_len        | payload | crc16     |
| 1 byte     | 1 byte     | 2 bytes, LE   | 1 byte    | 2 bytes, LE     | N bytes | 2 bytesLE |
+------------+------------+---------------+-----------+-----------------+---------+-----------+
+-----------+-----------+
| 0x0D      | 0x0A      |
| 1 byte    | 1 byte    |
+-----------+-----------+
```

- Header: `0xAA 0x55`
- Tail: `0x0D 0x0A`
- `frame_id`: little-endian `uint16_t`
- `data_len`: little-endian `uint16_t`
- Maximum payload length: `32` bytes
- Multi-byte numbers are little-endian
- `float` uses 4-byte IEEE754 layout as stored in STM32 memory

## `data_type` Values

| Value | Name | Payload meaning |
| --- | --- | --- |
| `0x01` | `DATA_UINT8_T` | `uint8_t` |
| `0x02` | `DATA_FLOAT` | `float` |
| `0x03` | `DATA_TYPE_TEXT` | text bytes, no trailing `\0` in the frame |
| `0x04` | `DATA_UINT16_T` | `uint16_t` |
| `0x05` | `DATA_UINT32_T` | `uint32_t` |

## CRC

- CRC algorithm: `CRC16/Modbus`
- Initial value: `0xFFFF`
- Polynomial: `0xA001`
- CRC range:
  - starts at `frame_id`
  - includes `frame_id + data_type + data_len + payload`
  - does not include frame header, CRC field itself, or frame tail
- CRC is stored little-endian

In code, the CRC input length is:

```text
2 (frame_id) + 1 (data_type) + 2 (data_len) + payload_length
```

## RX Parsing Rules

On the business UART, the parser:

- scans the received byte stream for `0xAA 0x55`
- reads `frame_id`, `data_type`, `data_len`
- rejects frames with `data_len > 32`
- checks that the complete frame is present
- verifies tail bytes `0x0D 0x0A`
- verifies `CRC16/Modbus`
- if valid, dispatches by `frame_id`

Important actual behavior:

- `data_type` is parsed but not validated against `frame_id`
- the dispatch layer only reads payload bytes by expected length
- if the sender gives the wrong `data_type` but the payload layout happens to match, the command may still work

## Protocol IDs

### Host -> Device

| Frame ID | Symbol | Type | Actual handling |
| --- | --- | --- | --- |
| `0x1000` | `U8_HEARTBEAT_REQ` | `uint8` | Device replies with `0x1100 = 1` |
| `0x1001` | `F32_PRESSURE_SET_KPA` | `float` | Sets target pressure, then clamps to `0.0 .. 400.0 kPa` |
| `0x1002` | `F32_LEFT_TEMP_SET_C` | `float` | Sets target temperature; current app logic applies the same target to left and right |
| `0x1004` | `U8_LEFT_EYE_ENABLE` | `uint8` | `0=disable`, nonzero=`enable` |
| `0x1005` | `U8_RIGHT_EYE_ENABLE` | `uint8` | `0=disable`, nonzero=`enable` |
| `0x1006` | `U16_TREAT_TIME_MIN` | `uint16` | Sets treatment duration; current app logic treats this value as count of 60-second cycles, not literal minutes |
| `0x1007` | `U8_LEFT_HEATER_FUSE_BLOW_CMD` | `uint8` or ignored | Triggers heater shield fuse-blow request |
| `0x1008` | `U8_RIGHT_HEATER_FUSE_BLOW_CMD` | `uint8` or ignored | Triggers heater shield fuse-blow request |
| `0x10C0` | `U8_MODE_SELECT` | `uint8` | Selects mode `1..4`; storage currently only has 2 profile slots |
| `0x10C1` | `U8_START_TREATMENT` | `uint8` or ignored | Starts treatment |
| `0x10C2` | `U8_STOP_TREATMENT` | `uint8` or ignored | Stops treatment |
| `0x10C3` | `U8_SAVE_SETTINGS` | `uint8` or ignored | Requests parameter save |
| `0x10C4` | `U8_PAUSE_RESUME_TREATMENT` | `uint8` | `0=pause`, nonzero=`resume` |

### Device -> Host

| Frame ID | Symbol | Type | Actual source |
| --- | --- | --- | --- |
| `0x1100` | `U8_HEARTBEAT_ACK` | `uint8` | Reply to heartbeat request, payload fixed to `1` |
| `0x1101` | `F32_LEFT_PRESSURE_VALUE` | `float` | Pressure telemetry from `gSensorData.pressL` |
| `0x1102` | `F32_RIGHT_PRESSURE_VALUE` | `float` | Defined in enum, but currently not sent |
| `0x1103` | `F32_LEFT_TEMP_VALUE` | `float` | Left temperature telemetry |
| `0x1104` | `F32_RIGHT_TEMP_VALUE` | `float` | Right temperature telemetry |
| `0x1107` | `U8_LEFT_HEATER_PRESENT` | `uint8` | Left heater shield present status |
| `0x1108` | `U8_RIGHT_HEATER_PRESENT` | `uint8` | Right heater shield present status |
| `0x1109` | `U8_LEFT_HEATER_FUSE` | `uint8` | Left heater fuse status |
| `0x110A` | `U8_RIGHT_HEATER_FUSE` | `uint8` | Right heater fuse status |
| `0x110B` | `U8_MODE_CURVES` | `uint8` | Current phase character: `'r'`, `'h'`, `'p'`, `'i'`, sometimes `'v'` by comment |

## Command Mapping Into Application

Business UART frames are not handled directly by control logic. The path is:

```text
USART3 RX -> uart_driver parser -> UartFrame_Dispatch()
         -> HostProtocol::dispatch()
         -> app_event_t(APP_EVT_HOST_COMMAND)
         -> TreatmentApp
         -> ctrl_cmd_t
         -> ControlTask
```

Current frame-to-app command mapping:

| Frame ID | App command | Notes |
| --- | --- | --- |
| `0x1001` pressure set | `APP_CMD_SET_PRESSURE_KPA` | value is clamped in protocol layer |
| `0x1002` temp set | `APP_CMD_SET_TEMP` | one shared target for both sides |
| `0x1004` left enable | `APP_CMD_LEFT_ENABLE` | nonzero becomes `1` |
| `0x1005` right enable | `APP_CMD_RIGHT_ENABLE` | nonzero becomes `1` |
| `0x1006` treatment time | `APP_CMD_SET_TREATMENT_TIME` | semantic mismatch: not true minutes in runtime |
| `0x10C0` mode select | `APP_CMD_MODE_SELECT` | valid mode range later normalized to `1..4` |
| `0x10C1` start | `APP_CMD_START` | payload ignored |
| `0x10C2` stop | `APP_CMD_STOP` | payload ignored |
| `0x10C3` save | `APP_CMD_SAVE_PARAM` | payload ignored |
| `0x10C4` pause/resume | `APP_CMD_PAUSE_RESUME` | `0=pause`, nonzero=`resume` |

## Device Telemetry Behavior

### 1. Heartbeat reply

- Triggered only when host sends `0x1000`
- Device enqueues one `0x1100` frame with payload `1`
- This is request-response, not periodic push

### 2. Pressure telemetry

- Sent only while treatment is running
- Current code only sends `F32_LEFT_PRESSURE_VALUE (0x1101)`
- Send rate: `100` points/second
- Period: `10 ms`

### 3. Temperature telemetry

- Sent only while treatment is running
- Sends both:
  - `0x1103` left temp
  - `0x1104` right temp
- Send rate: `50` points/second
- Period: `20 ms`

Actual extra behavior:

- temperature display offset currently is `0.0`
- invalid instantaneous temperature samples are filtered out for TX
- if a new sample is invalid, the last valid transmitted temperature is reused

### 4. Phase telemetry

- Sent together with temperature telemetry
- Frame ID: `0x110B`
- Type: `uint8`
- Payload is the first character of phase name:
  - `'r'`: rise
  - `'h'`: hold
  - `'p'`: pulse
  - `'i'`: idle
- Comments also mention `'v'` for vent, but whether it appears depends on the wave-control path

### 5. Heater shield status telemetry

- Sent by `HeaterShieldStatus_Process()`
- Check period: about `100 ms`
- Reports:
  - left present `0x1107`
  - right present `0x1108`
  - left fuse `0x1109`
  - right fuse `0x110A`

Value meaning:

- present: `1=present`, `0=not present`
- fuse: `1=normal`, `0=blown`

## Configuration Synchronization Behavior

The firmware reuses some host command IDs as device-to-host configuration broadcast IDs.

When settings are loaded or saved, `Settings_Broadcast()` sends:

| Direction | Frame ID | Meaning |
| --- | --- | --- |
| Device -> Host | `0x10C0` | current selected mode |
| Device -> Host | `0x1002` | current target temperature |
| Device -> Host | `0x1001` | current target pressure |

This is important for refactor:

- these IDs are defined in the enum as host-to-device commands
- but the implementation also uses them as host UI sync notifications
- so the protocol is not direction-strict today

## Treatment State Related Behavior

### Start

When host sends `0x10C1`:

- if both left/right channels are disabled, app logic auto-enables both sides first
- then treatment enters running state

### Stop

When host sends `0x10C2`:

- treatment stops
- pause state is cleared

### Pause/Resume

When host sends `0x10C4`:

- `0` means pause
- nonzero means resume

Paused treatment:

- shuts off outputs
- freezes treatment timer position
- resumes from the same wave position later

### Time field semantic mismatch

`0x1006` is named `U16_TREAT_TIME_MIN`, but current runtime meaning is:

- value = number of 60-second treatment cycles
- `0` is normalized to `1`

So a host sending `5` should currently expect about `5 * 60 s = 300 s`, not necessarily 5 literal minutes by a separate minute clock abstraction.

## Important Implementation Quirks

These are worth keeping during refactor planning because they may be accidental dependencies.

### 1. Right fuse-blow command currently affects both sides

Current code:

- `0x1007` calls `HeaterShieldStatus_RequestFuseBlow(1, 1)`
- `0x1008` also calls `HeaterShieldStatus_RequestFuseBlow(1, 1)`

That means both commands request blowing both fuse outputs, not left-only/right-only.

This looks suspicious and may be a bug, but it is the current behavior.

### 2. `APP_CMD_READ_PARAM` exists but has no current UART entry

Application layer supports `APP_CMD_READ_PARAM -> Settings_Broadcast()`, but there is no current `frame_id` in `HostProtocol::dispatch()` that generates this app command.

So, from the current UART protocol implementation, there is no dedicated host frame for "read current parameters".

### 3. `F32_RIGHT_PRESSURE_VALUE` is defined but not emitted

The enum contains `0x1102`, but `ControlTask` only sends left pressure.

### 4. `U8_STOP_TREATMENT` is reused as a device notification

When treatment finishes by timer, firmware sends:

- `0x10C2`
- payload `1`

So this frame ID is used in both directions:

- Host -> Device: stop command
- Device -> Host: treatment finished notification

This is another direction-overloaded ID and should probably be made explicit during refactor.

### 5. Parser accepts concatenated frames in one DMA chunk

The parser iterates through the RX buffer and can dispatch multiple valid frames from one received block.

### 6. Parser may stop on incomplete trailing frame

If the last frame in the current RX block is incomplete, parsing stops and waits for the next DMA callback. There is no separate stream reassembly buffer in this layer.

Because the RX source is `ReceiveToIdle`, the sender should preferably transmit one full frame per burst.

## Recommended Refactor Split

If you are going to reconstruct this cleanly, the current protocol can be split into:

1. Transport/frame layer
   - header/tail
   - length
   - CRC
   - serialization/deserialization
2. Message schema layer
   - `frame_id`
   - direction
   - payload type
   - payload meaning
3. Application command layer
   - host command -> app command
   - telemetry source -> outgoing message
4. Compatibility layer
   - reused IDs
   - current semantic mismatches
   - current accidental behaviors that old host software may rely on

## Code Reference

- Frame IDs: `Application/Uart_Module/Uart_Communicate.h`
- Host frame dispatch: `Application/Uart_Module/Uart_Communicate.cpp`
- Frame build / parse / CRC: `Application/Uart_Module/uart_driver.cpp`
- UART type definitions: `Application/Uart_Module/uart_driver.h`
- Communication task scheduling: `Application/Tasks/comm_task.cpp`
- App command handling: `Application/AppMain/TreatmentApp.cpp`
- Telemetry emission: `Application/Tasks/control_task.c`
- Heater shield status telemetry: `Application/HeaterShieldStatus/heater_shield_status.c`
- Config broadcast: `Application/Config/config.c`
- Host stop notification: `Application/Services/AppServices.cpp`

