# PID Tuner

通过 `UART1` 调试口实时调节和观察多路 PID：
- 压力 PID 三段：`PR / PH / PP`
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

## UART1 PID 二进制协议

UART1 现在不再使用 ASCII 调参命令，改为二进制帧协议。

### 1. 设置 PID 参数帧

帧格式：

```text
C5 5C | 01 | 01 | 0D | target | kp(float) | ki(float) | kd(float) | crc16
```

字段说明：
- `C5 5C`：帧头
- 第 1 个 `01`：协议版本
- 第 2 个 `01`：命令字，表示设置 PID
- `0D`：payload 长度 13 字节
- `target`：目标编号
- `kp/ki/kd`：`float32 little-endian`
- `crc16`：Modbus CRC，计算范围从 `version` 开始到 `payload` 结束

目标编号：
- `0`：`PR`
- `1`：`PH`
- `2`：`PP`
- `3`：`HL`
- `4`：`HR`

### 2. PID Stream 开关帧

帧格式：

```text
C5 5C | 01 | 02 | 01 | enable | crc16
```

字段说明：
- `02`：命令字，表示 PID Stream 开关
- `enable`：`0x00` 关闭，`0x01` 打开

### 3. PID 遥测帧

UART1 会发送 PID 二进制遥测帧，格式如下：

- 帧头：`C5 5C`
- 版本：`1 byte`
- 目标：`1 byte`
- 时间戳：`uint32`
- `kp, ki, kd, p, i, d, error, output, setpoint, measurement`
  - 共 `10 * float32`
- CRC16(Modbus)：`uint16`
- 帧尾：`0D 0A`
