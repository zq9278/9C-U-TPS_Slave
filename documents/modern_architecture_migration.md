# 现代嵌入式架构迁移路线

本文档用于指导当前 STM32 医疗热脉动设备固件，从“能运行的 HAL + FreeRTOS 工程”逐步迁移到
CMake + C++17 + 分层模块化架构。

迁移原则：

- 现有功能稳定优先。
- 每一阶段都必须能编译、能回滚、能验证。
- HAL 最终收敛到 BSP 层。
- 业务模块优先使用 C++ class。
- FreeRTOS 任务只负责调度，不直接承载复杂业务。

## 当前问题判断

### 代码耦合

当前项目已经开始 C++ 化，但历史代码仍然存在明显耦合：

- `ControlTask` 同时包含状态机、PID、硬件输出、遥测上报、调试输出。
- 旧 `Application/heat`、`ValveControl`、`HeaterShieldStatus` 等模块仍直接调用 HAL。
- 全局队列和共享状态仍集中在 `system_app.h`，例如 `gAppEventQueue`、`gCtrlCmdQueue`、`gTxQueue`、`gSensorData`。
- 上位机协议已经拆到 `HostProtocol`，但协议 payload 和业务命令仍共用部分老结构。

### HAL 调用分布

HAL 目前分布在：

- `Core/Src/*`：CubeMX 生成代码，允许保留。
- `Application/Uart_Module/uart_driver.cpp`：串口 DMA/IDLE 驱动。
- `Application/Tasks/control_task.c`：PWM、定时器 CCR 直接访问。
- `Application/heat`、`ValveControl`、`HeaterShieldStatus`、`water`、`ds18b20`：GPIO/PWM 直接操作。

目标是逐步把这些 HAL 调用收敛到 `BSP/`。

### 中断、任务、业务逻辑

当前已经改善：

- UART ISR 只入队。
- `CommTaskRunner` 统一处理业务串口 TX/RX 和调试串口 TX。
- `TreatmentApp` 已经承担部分治疗业务状态。

仍需改善：

- `ControlTask` 仍是最大复杂点。
- 安全、耗材、治疗流程的状态机还没有完全抽象。
- `DebugLogTask` 后续可以改成 `DebugCommandTask` 或合并到通信 runner 的 RX 分支。

### 是否适合直接迁移到 C++

适合，但不能一次性全部重写。

推荐策略：

- 新代码全部 C++。
- 旧 C 模块先包一层 C++ facade。
- 每次只替换一个清晰边界，例如 UART、PWM、加热、压力、耗材。

## 目标目录结构

```text
Project/
├── Core/              # CubeMX/HAL 生成代码，尽量少改
├── Drivers/           # CMSIS / HAL Driver，尽量少改
├── BSP/               # STM32 外设封装，只允许这里直接碰 HAL
├── Components/        # 器件驱动，例如 EEPROM、ADS1248、DS18B20、屏幕
├── Modules/           # 功能模块，例如加热、压力、耗材、安全
├── App/               # 治疗业务流程、应用状态机
├── RTOS/              # 任务 runner、队列适配、调度入口
├── Utils/             # CRC、滤波、限幅、环形缓冲、时间工具
├── Config/            # 编译配置、运行参数结构、默认参数
├── cmake/             # CMake 工具链和构建脚本
└── CMakeLists.txt
```

### `Core/`

放：

- CubeMX 生成的初始化代码。
- `main.c`、`gpio.c`、`usart.c`、`tim.c` 等。

不放：

- 业务状态机。
- PID 控制策略。
- 协议解析。

### `BSP/`

放：

- `GpioPin`
- `PwmChannel`
- `UartPort`
- `SpiBus`
- `I2cBus`
- `AdcChannel`

不放：

- 压力算法。
- 温控算法。
- 治疗流程。

当前已新增：

- `BSP/Gpio/GpioPin.hpp/.cpp`
- `BSP/Pwm/PwmChannel.hpp/.cpp`
- `BSP/Uart/UartPort.hpp/.cpp`

### `Components/`

放具体器件驱动：

- EEPROM / 24C02
- ADS1248
- DS18B20
- OLED / UI 屏
- 压力传感器芯片驱动

特点：

- 可以依赖 BSP。
- 不应该依赖 App。

### `Modules/`

放可复用功能模块：

- `HeatingController`
- `PressureController`
- `ConsumableManager`
- `SafetyManager`
- `AlarmManager`
- `LifetimeManager`

特点：

- 可以使用 Components。
- 尽量不直接使用 HAL。
- 尽量不直接使用 FreeRTOS。

当前已新增：

- `Modules/Heating/HeatingController`
- `Modules/Pressure/PressureController`
- `Modules/Consumable/ConsumableManager`

### `App/`

放业务流程：

- 治疗状态机
- 开始/暂停/停止流程
- PreCheck 流程
- 错误处理流程

当前已新增：

- `App/Treatment/TreatmentStateMachine`

### `RTOS/`

放：

- 任务 runner。
- 队列适配器。
- FreeRTOS 和 App/Modules 的桥接。

不放：

- 复杂业务逻辑。
- HAL 直接调用。

## 渐进式迁移路线

### 阶段 1：不改业务，只建立 CMake 构建系统

目标：

- 工程可以用 CMake 稳定构建。
- 支持 C + C++17 混合编译。

当前状态：

- 已完成。
- 已启用 C++17。
- 已关闭异常和 RTTI。

风险：

- C/C++ 符号名不兼容。
- CubeMX 生成文件和手写 CMake 冲突。

验证方法：

```text
cmake --build --preset Debug
```

回滚方案：

- 回退 `CMakeLists.txt`。
- 保留原 CubeIDE 工程。

### 阶段 2：整理目录结构，不改变函数逻辑

目标：

- 建立 `BSP/Components/Modules/App/RTOS/Utils/Config`。
- 新目录先参与编译，但不接管旧功能。

当前状态：

- 已开始。
- 已新增 BSP 和核心医疗模块骨架。

风险：

- CMake 漏加 include/source。
- 新类命名和旧 C 符号冲突。

验证方法：

- 编译通过。
- 烧录后确认原有启动、停止、遥测、PID 调试仍正常。

回滚方案：

- 从 `CMakeLists.txt` 移除新增源文件。
- 删除新增目录。

### 阶段 3：封装 UART/SPI/I2C/ADC/PWM/GPIO 到 BSP

目标：

- HAL 调用逐步收敛到 BSP。
- 业务模块只使用 `bsp::GpioPin`、`bsp::PwmChannel` 等接口。

要移动/修改的文件：

- `Application/Uart_Module/uart_driver.cpp` -> 后续迁移到 `BSP/Uart`
- `Application/ValveControl/valve_control.c` -> 使用 `bsp::GpioPin`
- `Application/heat/heat.c` -> 使用 `bsp::PwmChannel` 和 `bsp::GpioPin`
- `Application/Tasks/control_task.c` 中直接访问 `TIM15->CCR1` 的位置 -> 使用 PWM BSP

示例：

```cpp
bsp::PwmChannel pumpPwm(&htim15, TIM_CHANNEL_1);
pumpPwm.start();
pumpPwm.setCompare(pwm);
```

风险：

- PWM channel 映射错误。
- GPIO 极性错误。
- DMA UART 生命周期管理错误。

验证方法：

- 每迁移一个外设，只验证一个功能。
- 用示波器/逻辑分析仪确认 GPIO/PWM 输出。
- UART 验证收发、停止命令、PID 曲线。

回滚方案：

- 保留旧 C API。
- 新 BSP 先包旧 API，确认稳定后再反向替换。

### 阶段 4：器件驱动封装成 Components

目标：

- 把具体芯片/器件驱动从 Application 中抽到 Components。

候选：

- `Application/24C02`
- `Application/ads1248_v2`
- `Application/ds18b20`
- `Application/Pressure_sensor`

示例：

```cpp
class Eeprom24C02 {
public:
    bool read(uint16_t addr, void *data, uint16_t len);
    bool write(uint16_t addr, const void *data, uint16_t len);
};
```

风险：

- 时序类驱动容易被抽象破坏。
- EEPROM 写入延时和页边界处理不能丢。

验证方法：

- 读写固定 pattern。
- 断电重启读取参数。

回滚方案：

- 保留旧 C 驱动函数。
- C++ component 内部先调用旧 C 函数。

### 阶段 5：抽出 Heating/Pressure/Consumable/Safety Modules

目标：

- `ControlTask` 不再直接承载全部控制算法。
- 医疗业务规则从任务代码中抽离。

重点模块：

- `HeatingController`
- `PressureController`
- `ConsumableManager`
- `SafetyManager`

耗材规则：

- 插入耗材只进入 `Inserted`。
- 只有点击开始治疗，并且开始请求被系统接受后，才标记 `InUse` 或增加使用次数。
- 拔插本身不能报废耗材。

风险：

- 控制闭环时序改变。
- 耗材计数时机错误属于医疗安全风险。

验证方法：

- 单元测试状态机。
- 手动测试插入、拔出、开始、停止、异常断电。

回滚方案：

- TreatmentApp 仍保留旧逻辑入口。
- 新模块先旁路计算，不直接驱动硬件。

### 阶段 6：整理 App 层治疗状态机

目标状态：

```text
Idle
Inserted
Ready
PreCheck
Heating
PressureRunning
TreatmentRunning
Finish
Error
```

当前已新增：

- `App/Treatment/TreatmentStateMachine`

后续动作：

- 让 `TreatmentApp` 内部持有 `TreatmentStateMachine`。
- 把开始/暂停/停止/故障逻辑逐步交给状态机。

风险：

- 状态转移遗漏。
- 暂停/继续和倒计时恢复逻辑变化。

验证方法：

- 每个事件写状态转移表。
- 用日志打印状态变化。
- 对照 Qt 上位机流程测试。

回滚方案：

- 保留 `TreatmentApp::updateControlState()` 旧逻辑。
- 新状态机先只记录状态，不参与决策。

### 阶段 7：整理 FreeRTOS 任务

目标：

- 任务只负责调度。
- 复杂逻辑全部进入 Runner/App/Modules。

当前建议：

- `CommTask`：保留通信 TX/RX，已经适合放调试串口 TX。
- `DebugLogTask`：后续可改名为 `DebugCommandTask`，只处理调试串口 RX 命令。
- `ControlTask`：最终变薄，只调用 `PressureController` 和 `HeatingController`。
- `SensorTask`：最终只采集传感器并发布快照。

风险：

- 优先级和队列阻塞时间改变。
- 调试串口阻塞发送影响业务串口任务。

验证方法：

- 停止命令延迟测试。
- PID 曲线持续输出测试。
- 运行 30 分钟观察任务栈和 heap。

回滚方案：

- 保留旧任务入口。
- 每次只迁移一个任务内部的小分支。

### 阶段 8：核心模块 C++ class 化

目标：

- 新模块全部 C++。
- 旧 C 模块只保留稳定驱动或兼容层。

风险：

- C/C++ ABI。
- 静态初始化顺序。
- 不小心引入动态内存。

约束：

- 不使用异常。
- 不使用 RTTI。
- 不使用堆创建核心对象。
- HAL 回调和任务入口保持 `extern "C"`。

验证方法：

- 编译。
- 静态 RAM/FLASH 变化。
- 上板功能测试。

