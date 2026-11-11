# TPS130 Studio 2.1：调试平台与设备适配

## 交付对象

本次重构的是原Windows Electron烧录调试工具，产品名改为TPS130 Studio；不修改ARM Linux治疗界面，也不改变MCU业务协议。保留原SWD烧录、回读校验、RTT、Mender发布能力与立即关闭行为。

界面采用固定侧栏、公共连接头、设备工作台、公共任务日志和成功弹窗。六个页面：设备概览、调试工作台、固件与发布、检查与诊断、日志与记录、连接与设置。

## 模块边界

| 层 | 路径 | 职责 |
|---|---|---|
| 公共平台模型 | Tools/Flasher/platform/core.js | 流式行解码、受限历史、数据过期、断线/重启断点、记录验证、CSV |
| 公共曲线控件 | Tools/Flasher/platform/chart.js | 真实数据绘制、颜色与显示开关、时间窗口、缺失数据断线 |
| TPS130能力定义 | Tools/Flasher/devices/tps130/manifest.json | 型号、测点、单位、颜色、参数、能力开关和限制说明 |
| TPS130协议适配 | Tools/Flasher/devices/tps130/adapter.js | 将现有RTT info/status回复映射成identity/sample事件 |
| 首个设备工作台 | Tools/Flasher/index.html、renderer.js | TPS130温压、参数区、实时/回放联动；公共导航和任务交互 |
| 主进程服务 | main.cjs、backend.cjs、publisher.cjs | 资源互斥、烧录、文件校验、发布、文件对话框和导出 |
| RTT连接 | rtt.cjs、rtt_bridge.py | 独占J-Link后台RTT，校验诊断与PID命令，不直接操作治疗输出 |
| 品牌资源 | assets/tps130.svg、png、ico | TPS130独立眼形+脉冲图标；窗口、任务栏和EXE图标 |

这是首个设备适配版本，不宣称任意设备已即插即用。新增设备时先定义自己的manifest、协议适配器与工作台布局，再接入对应板卡升级/发布服务；TPS130的Flash、Mender类型、泵/加热语义不应作为其他设备默认值。当前打包目标只有TPS130，其他设备需各自配置名称、AppUserModelID、EXE和图标，不能仅复制TPS130参数。

后续增加设备注册器、不同工作台模板和可选串口/TCP传输时，应保持页面通过服务接口调用，不允许专用页面直接使用Node/读写任意硬件地址。当前Electron renderer保持contextIsolation、sandbox且不开放Node。

## 当前真实数据能力

Release固件0.1.2开放help/info/status与PID指令。工作台约每200ms请求status，解析左右温压、设备Tick、治疗标志、剩余堆和日志丢弃；info提供设备身份。PID按需开启10Hz数据流，具体协议见下文。

这是5Hz请求的诊断状态快照，不等于原始传感器采样；实际更新率取决于RTT链路。断线、暂停采集或超过1.5s没有状态回复时，值标记为历史/过期，不能继续显示“实时正常”。数据缺失超过1.5s、断线后重连、设备Tick回退时曲线不跨段连接。

参数区域提供设备身份查询与8组PID增益读写，使用请求ID、范围检查、执行回执和实际存储值回读。目标温压与治疗时序不通过调试接口写入。PID增益仅RAM生效，具体协议见下文。

## 录制、导出与回放

- 实时历史最多18000点；显式录制最多50000点，达到上限停止录制并提示保存。
- JSON记录schema=1，标明device=tps130、设备身份、时间和sample数组。导入检查产品、数量、时间顺序与有限数值；文件不超过20MB。
- CSV列包含主机ISO时间、设备Tick、治疗标志和带单位的测点。记录仅用于诊断，传感器快照不保证各字段同一采样时刻。
- 回放提供时间滑块与按记录时间播放；回放时暂停实时轮询，禁止设备查询/新连接操作，返回实时时恢复原有连接的采集。
- 暂停查看曲线只冻结显示，不暂停设备，也不停止后台采集；暂停采集仅停止只读状态请求。
- 工作台支持 PID 参数写入；不提供治疗启动、急停或执行器测试按钮。

## 图标与软件身份

产品名TPS130 Studio、EXE名TPS130-Studio.exe、Windows AppUserModelID=Sanitlook.TPS130.Studio。图标由代码原生矢量几何生成，非沿用Electron默认图标；scripts/make_icon.py生成多分辨率ICO及PNG，SVG用于界面。打包用rcedit写入EXE资源和产品信息，窗口也显式指定ICO。

## 构建与验收

```powershell
npm ci --prefix Tools/Flasher
python scripts/build_release.py
python scripts/test.py
python scripts/package_flasher.py --skip-build --electron <electron/dist目录>
```

默认打包输出out/distribution/TPS130-Studio-2.1.0-时间戳，LATEST.txt指向最新目录。需要本机安装J-Link；分发包会携带运行时及原厂许可，不包含管理凭据、私钥或用户录制数据。

主机测试包括：设备描述校验、真实协议格式的分块解析、无效数值拒绝、数据过期与断线、记录格式/设备匹配、CSV、六页导航、工作台与离线回放联动、成功弹窗，以及原烧录/发布/固件恢复测试。GUI冒烟注入的数据是明确的测试fixture，测试后清空，发布程序启动无模拟曲线。

尚未实板验证J-Link采集稳定性、状态更新率、长期录制与实际设备安全状态；仍保留release.json中的hardware_validation=NOT_RUN。本次实现 RTT PID 调参及被动稳定性测试；传感器校准、本机串口 OTA 与其他设备适配未实现。


## 2.1 PID 与稳定性测试

App 0.1.2 增加 RTT PID 文本指令。设备执行后返回 request ID 回执，上位机等待3.5秒；超时不显示成功、不自动重发，需读取确认。操作无二次确认，写入成功有明显弹窗。

- `pid get <req> <target> <profile>` 读取指定组。
- `pid set <req> <target> <profile> <kp> <ki> <kd>` 写入并返回实际存储值。
- `pid stream <req> <0|1>` 开关调试流。
- req为1..1000000；target为0升压、1保压、2脉冲、3左温度、4右温度；profile为0单眼、1双眼、2温度。压力仅允许profile 0或1，温度仅允许2。
- 增益为有限浮点数，协议边界0..1000；此边界不代表经过验证的治疗参数范围。旧UART1写入也增加有限数值和范围检查。
- 全部读写经过ControlTask队列。压力模式显式选择，不依赖当前治疗模式。不启动治疗，不更改目标温压。参数仅RAM生效，重启恢复编译默认，无Flash保存命令。
- 成功示例：`RTT> PID ack=1 ok=1 target=3 profile=2 kp=0.7 ki=0.3 kd=0 volatile=1`。失败返回ok=0及reason。升级期间控制任务不执行调参，客户端按超时处理。
- PID流10Hz：两个温度组和当前压力阶段。通道1保留56字节二进制帧；通道0增加文本镜像，含profile、tick、增益、P/I/D、误差、目标、反馈、输出、映射输出。现有离线回放记录不含PID流，长时间无数据会明确标记历史状态。

稳定性测试在“测试与诊断”页，最长24小时。用户设置四路温压上下限和1..60秒快照超时。必须有设备身份和新鲜快照才能开始。运行期间锁定配置，禁用PID写入和导入回放，持续5Hz状态采集；不自动启停治疗。

报告记录最小/最大/均值/总体标准差、越限快照数、最大数据间隔、超时、断连、Tick回退/停滞、缺失测点和日志丢弃。正常32位Tick回绕不判为重启。任何异常保留为FAIL，恢复正常不能清除；手动结束为ABORTED，不算通过。达到设定时长、有有效完整快照且无记录异常才PASS并弹窗。

JSON报告包含身份、配置、结论、统计、最多1000条事件和每秒一条完整快照。事件总数单独保存，截断会标注。CSV导出测点统计。关闭程序即停止测试，未导出的报告不自动保存。测试只覆盖已配置的观测条件，不替代硬件保护或整机验证。
