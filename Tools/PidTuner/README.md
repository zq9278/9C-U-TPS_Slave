# PID Tuner

用于通过调试串口实时调节下位机压力波形 PID。

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

## 串口命令

客户端会发送下位机已经支持的 ASCII 命令：

```text
R kp ki kd
H kp ki kd
P kp ki kd
```

其中：

`R` 表示 `RISE` 阶段，`H` 表示 `HOLD` 阶段，`P` 表示 `PULSE` 阶段。
