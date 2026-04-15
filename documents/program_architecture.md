# Program Architecture

This document summarizes the runtime architecture and UART data flow of the
9C-U-TPS Slave firmware (STM32G070 + FreeRTOS).

## System Overview (Tasks + Queues)

```mermaid
flowchart LR
  subgraph Init[Startup]
    main[main.c: HAL init] --> freertos[MX_FREERTOS_Init]
    freertos --> appinit[AppMain_FreeRTOS_Init]
  end

  subgraph Queues[Global Queues]
    qCmd[gCmdQueue<br/>Comm -> App]
    qCtrl[gCtrlCmdQueue<br/>App -> Control]
    qTx[gTxQueue<br/>Any -> Comm]
    qSafe[gSafetyQueue<br/>Safety -> App]
    qStore[gStorageQueue<br/>App -> Storage]
  end

  subgraph Tasks[FreeRTOS Tasks]
    Comm[CommTask<br/>Comms I/O + protocol]
    App[AppTask<br/>System orchestration]
    Ctrl[ControlTask<br/>Closed-loop actuation]
    Sensor[SensorTask<br/>Sensor acquisition]
    Safe[SafetyTask<br/>Safety supervision]
    Store[StorageTask<br/>Persistent settings]
    IO[IoTask<br/>Status indication]
  end

  appinit --> Comm
  appinit --> App
  appinit --> Ctrl
  appinit --> Sensor
  appinit --> Safe
  appinit --> Store
  appinit --> IO

  Comm --> qCmd --> App
  App --> qCtrl --> Ctrl
  Ctrl --> qTx --> Comm
  Sensor --> Ctrl
  Safe --> qSafe --> App
  App --> qStore --> Store
```

## UART Data Path (Business Port)

```mermaid
flowchart TD
  UART[USART3 DMA + IDLE] --> ISR[HAL_UARTEx_RxEventCallback]
  ISR --> RXQ[rx_queue]
  RXQ --> Parse[parse_rk3576_uart_port_stream]
  Parse --> Dispatch[UartFrame_Dispatch]
  Dispatch --> CmdQ[gCmdQueue]
  CmdQ --> App[AppTask]
  App --> CtrlQ[gCtrlCmdQueue]
  CtrlQ --> Ctrl[ControlTask]
  Ctrl --> TxQ[gTxQueue]
  TxQ --> Send[send_rk3576_uart_port_frame]
  Send --> UARTTX[USART3 DMA TX]
```

## Detailed Command Flow (Host -> App -> Control)

```mermaid
flowchart TD
  Host[Host] --> Frame[UART frame: AA55 | id | type | len | payload | crc | 0D0A]
  Frame --> DMA[USART3 DMA + IDLE]
  DMA --> RXISR[HAL_UARTEx_RxEventCallback]
  RXISR --> RXQ[rx_queue (UartRxMessage_t)]
  RXQ --> Parse[parse_rk3576_uart_port_stream]
  Parse --> CRC{CRC OK?}
  CRC -- no --> Drop[Drop frame]
  CRC -- yes --> Dispatch[UartFrame_Dispatch]

  Dispatch -->|APP_CMD_*| CmdQ[gCmdQueue]
  Dispatch -->|Heartbeat| Ack[enqueue U8_HEARTBEAT_ACK -> gTxQueue]
  Dispatch -->|Fuse Blow Cmd| IO[GPIO pulse fuse pin]

  CmdQ --> App[AppTask]
  App --> Update[Update settings/state]
  Update --> CtrlQ[gCtrlCmdQueue]
  CtrlQ --> Ctrl[ControlTask]
  Ctrl --> Actuators[Pump/Valves/Heaters]
  Ctrl --> Telemetry[tx_frame_t -> gTxQueue]
  Telemetry --> CommTX[CommTask -> send_rk3576_uart_port_frame]
  CommTX --> UARTTX[USART3 DMA TX]
```

## Command Examples (Mermaid Sequence)

```mermaid
sequenceDiagram
  participant Host
  participant Comm as CommTask
  participant App as AppTask
  participant Ctrl as ControlTask
  participant HW as Pump/Valves/Heaters

  Host->>Comm: U8_START_TREATMENT (frame 0x10C1)
  Comm->>App: app_cmd_t { APP_CMD_START }
  App->>Ctrl: ctrl_cmd_t { CTRL_CMD_START, config }
  Ctrl->>HW: Start pressure cycle + enable heaters
  Ctrl-->>Comm: tx_frame_t telemetry (pressure/temp/mode)
  Comm-->>Host: Frames (0x1101/0x1102/0x1103/0x1104)
```

```mermaid
sequenceDiagram
  participant Host
  participant Comm as CommTask
  participant App as AppTask
  participant Ctrl as ControlTask

  Host->>Comm: F32_PRESSURE_SET_KPA (frame 0x1001)
  Comm->>App: app_cmd_t { APP_CMD_SET_PRESSURE_KPA, value }
  App->>Ctrl: ctrl_cmd_t { CTRL_CMD_UPDATE_CFG, new target }
  Ctrl-->>Comm: updated telemetry (new pressure target)
  Comm-->>Host: Frames (0x1101/0x1102 ...)
```

## Key Source Files

- UART framing and DMA RX/TX: `Application/Uart_Module/uart_driver.c`
- Command dispatch: `Application/Uart_Module/Uart_Communicate.c`
- Communication task: `Application/Tasks/comm_task.c`
- App orchestration: `Application/Tasks/app_task.c`
- Control loop and state machine: `Application/Tasks/control_task.c`
- Sensor sampling: `Application/Tasks/sensor_task.c`
- Safety checks: `Application/Tasks/safety_task.c`
- Settings storage: `Application/Tasks/storage_task.c`
- Protocol specification: `PROTOCOL.md`
