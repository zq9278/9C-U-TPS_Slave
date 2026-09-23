# J-Link 压力 PID 调试台

通过 J-Link 的 SWD 后台内存读写调节压力 PID，无需串口接线。界面显示左右压力、目标压力、控制反馈、阶段、P/I/D、PID 原始输出和电机 PWM；支持单双眼 × 升压/保压/脉冲共六组参数、JSON 保存/加载、CSV 导出和连续记录。

## 启动

1. 编译并烧录本次修改后的固件，让 MCU 正常运行。默认 ELF 为 `build/Debug/9C-TPS100-Slave.elf`。
2. 安装 SEGGER J-Link Software and Documentation Pack。接好 VTref、GND、SWDIO、SWCLK，给目标板供电。
3. 首次运行 `setup.bat` 安装 Python 依赖（需 Python 3.10+）；随后双击 `run_jlink_pid_tuner.bat`。本机若已创建 `.venv`，可直接运行。
4. 结束占用同一探针的 IDE/GDB Server、J-Link Commander、RTT Viewer 等会话。确认退出调试后 MCU 继续运行。
5. 选择与板上固件对应的 ELF，默认芯片 `STM32G070RBTx`、SWD 1000 kHz，点击“连接”。多探针时填写序列号。
6. 如果 DLL 自动查找失败，在 DLL 栏填写 SEGGER 安装目录中的 `JLink_x64.dll` 完整路径（需 64 位 Python）。

界面芯片仍填写 `STM32G070RBTx`，程序内部使用 `Cortex-M0` 内核附加，跳过 STM32 芯片专用连接钩子。本机 J-Link DLL 在按 `STM32G070RBTx` 连接时会复位并暂停目标，这是旧版调试台连接失败的原因。内核附加已在本机实测可读取运行中的邮箱。

程序只连接并读写调参邮箱，不烧录、复位或暂停 CPU，并禁用失败后复位重连以及退出时自动运行 CPU。检测到 CPU 已暂停会报错；需先恢复运行。治疗的启动、停止和目标压力仍通过原设备界面设置；断开调试台不会停止治疗。连接日志保存在同目录 `jlink_pid_tuner.log`，修改程序后需要关闭旧窗口并重新打开。

## 调参

- 连接后编辑框自动读取设备六组参数。选对应模式和阶段，修改 Kp/Ki/Kd，点击该行“应用”。显示“固件已确认，回读一致”才表示写入完成。
- 修改非当前模式/阶段的参数会保留，切换到该模式/阶段时生效。参数版本变化会在控制周期中重载 PID 并清空积分/微分历史。
- 参数只保存在 RAM，复位/断电恢复 `treatment_pressure_control.c` 的默认值。可以保存 JSON，重新连接后加载并逐行应用；加载文件本身不写设备。
- 右侧“设备回读”持续显示固件实际参数，不覆盖正在编辑的输入框。“设备参数 → 编辑框”可重新同步。
- 参数取值为 0～1000 的有限数。范围校验不代表范围内所有增益都适合具体气路，实际增益仍需台架调试。
- 曲线缓存最多 30000 点（50Hz 下约 10 分钟）。长时间测试使用“开始连续记录”，CSV 包含阶段、压力、PWM、P/I/D 和六组增益。

## 固件控制行为

RK3576 主界面在升压阶段显示模拟参考压力：以 0→目标值的斜坡为基础，叠加平滑伪随机起伏，单眼、双眼均生效。每轮更换随机种子，每 500ms 平滑过渡到新的随机偏移，偏差不超过目标值的 3.5%，起点和终点扰动收敛到零，显示值限制在 0～目标值内。可在 `App/System/app_tasks.c` 调整 `RK3576_RISE_SIM_NOISE_RATIO` 和 `RK3576_RISE_SIM_NOISE_PERIOD_MS`。此曲线不是传感器实测结果，不能用于判断实际跟踪效果。保压、脉冲和周期末泄气仍上传原实测滤波压力。开关 `RK3576_RISE_PRESSURE_DISPLAY_SIM_ENABLE` 设为 0 可恢复全部阶段实测显示。

模拟只替换现有 RK3576 压力帧 `0x1101` 的升压显示负载，不修改 `gSensorData`、PID 反馈、安全保护或 J-Link 遥测；J-Link 左右压力、反馈、CSV 均保留实测数据。查看实际升压效果请使用本调试台。升压结束切回实测值时，若实际压力与目标不同，主界面会出现相应跳变。RK3576 界面源码不在本工程内，若需要“模拟参考”文字标识，需在其界面端同步添加。

所有加压阶段使用 `PidController_ComputeDt()`：升压目标线性上升，保压目标恒定，脉冲 ON 目标为设定压力；脉冲 OFF 继续停泵放气。单眼反馈为启用侧压力，双眼为左右较大值。

每个 60 秒周期的 58～60 秒固定为“周期末泄气”：PWM=0、波形阀=0、保持当前单双眼气路，压力提前归零也不提前结束。正常配置下只截短脉冲段尾部 2 秒，升压和保压时长不变；默认时序为升压 10s、保压 20s、脉冲 28s、泄气 2s（配置宏中的脉冲 30s 包含末尾泄气 2s）。若配置的脉冲段不足 2 秒，末尾泄气仍优先，会截断前一阶段。下一周期从 0 秒直接开始升压，不再额外执行旧的跨周期最多 1 秒泄气。整个治疗结束后仍保持停泵泄气。

```c
#define PRESS_PID_DEFAULT_DT_S 0.002f  // 实际 ControlTask 传入 2ms
#define TREATMENT_CYCLE_VENT_MS 2000U // treatment_app_controller.h
#define PRESS_PUMP_MIN_PWM     10U
#define PRESS_PUMP_MAX_PWM     100U
#define PRESS_PUMP_RAW_MAX     25.0f
```

删除了原来四个 `*_RATED_PWM` 固定输出宏及 rise/hold 固定输出分支。原 `PRESS_VENT_ZERO_KPA` / `PRESS_VENT_MAX_MS` 跨周期放气逻辑已由固定周期末泄气窗口替代。

PID 原始输出仍限幅到 0～25，保留原参数量纲：输出 ≤0 时关闭；输出 >0 时按 `MIN + output * (MAX - MIN) / 25` 映射，整数截断。上下限以 `treatment_pressure_control.c` 中的配置和设备回读为准，当前为 10U～100U。单位为千分比（1%～10%），不是瓦特或百分比；关闭输出为 0U。原六组默认 PID 增益未重新整定。

## 通信与故障处理

`g_pid_jlink` 是 180 字节、32 位对齐的 SRAM 邮箱。上位机从 ELF 符号表定位地址，并核对调试模块构建标识、magic、协议版本、大小后才允许写入。始终使用当前烧录固件的 ELF，构建标识不是整份镜像的加密校验。

主机先写参数负载，读回验证，再写请求序号；固件在 ControlTask 中校验并应用参数，最后发布确认序号。一次仅允许一个未确认请求；超时结果未知时断开，不自动重发。遥测每 20ms 发布，通过前后相同且为偶数的序号保证读取过程中未更新，避免混合两帧数据。

MCU 复位、数据超过 2 秒不更新、SWD 出错均会断开连接。重新连接后以设备回读为准。采样采用轮询最新快照，没有固件历史环形缓冲，USB/主机延迟时允许丢帧，CSV 时间戳来自设备时钟。

SEGGER 文档说明 Cortex-M 支持运行中的后台内存访问：[Memory accesses](https://kb.segger.com/Memory_accesses)。接口使用 [PyLink SDK 包装](https://pylink.readthedocs.io/en/latest/pylink.html) 的 `memory_read32` / `memory_write32`。

## 开发验证

```powershell
.venv\Scripts\python.exe -m unittest discover -s tests -v
```

协议测试使用模拟探针，不连接硬件。固件行为测试需要先构建 Debug ELF 并安装 `pip install unicorn`，然后运行 `test_firmware.py`；它在 ARM 模拟器中执行本次编译的控制函数，并截获 PWM/阀硬件调用，不驱动真实设备。
