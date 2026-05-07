# PID Tuner

用于通过 `UART1` 调试口实时调节和观察多路 PID：

- 压力 PID 三段：`PR` / `PH` / `PP`
- 左眼加热 PID：`HL`
- 右眼加热 PID：`HR`

## 安装依赖

```powershell
cd D:\Project\9C-U-TPS\software\9C-U-TPS_Slave\Tools\PidTuner
python -m pip install -r requirements.txt
```

## 运行

```powershell
python pid_tuner.py
```

也可以双击 `run_pid_tuner.bat`。

## UART1 调试协议

### 1. ASCII 调参命令

```text
PID STREAM 1
PID STREAM 0
PID SET PR kp ki kd
PID SET PH kp ki kd
PID SET PP kp ki kd
PID SET HL kp ki kd
PID SET HR kp ki kd
```

说明：

- `PID STREAM 1`：打开 PID 调试数据流
- `PID STREAM 0`：关闭 PID 调试数据流
- `PR`：压力升压段 PID
- `PH`：压力保压段 PID
- `PP`：压力脉动段 PID
- `HL`：左眼加热 PID
- `HR`：右眼加热 PID

### 2. 二进制遥测帧

UART1 会混发普通文本日志和 PID 二进制调试帧。调试帧格式如下：

- 帧头：`C5 5C`
- 版本：`1 byte`
- 目标：`1 byte`
- 时间戳：`uint32`
- `kp, ki, kd, p, i, d, error, output, setpoint, measurement`：共 `10 * float`
- CRC16(Modbus)：`uint16`
- 帧尾：`0D 0A`

当前目标编号：

- `0`：`PR`
- `1`：`PH`
- `2`：`PP`
- `3`：`HL`
- `4`：`HR`
