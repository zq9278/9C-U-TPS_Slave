# TPS130 Studio 2.2.0

本工具已从单页烧录器重构为设备调试平台：设备概览、TPS130专属工作台、固件与发布、检查与诊断、日志与记录、连接设置。运行TPS130-Studio.exe；新产品图标已写入窗口、任务栏和EXE。

新增独立“电源板烧录”模块：PowerManage / STM32F030F4，Flash16KiB、RAM4KiB、烧录地址0x08000000。使用电源板自身SWD，探针序列号和速度沿用连接设置。提供烧录、仅校验和打开固件目录，与Slave烧录、RTT、发布任务互斥。点击直接执行，完整16KiB回读SHA256匹配后弹窗；校验也会暂停CPU并复位。运行前停止治疗，结束后核对电源板及整机供电。

PowerManage无已声明的固件版本接口，界面展示源码构建标识，不当作设备实测版本。内置电源板文件在resources/payload/firmware-powermanage；Slave文件仍在resources/payload/firmware。电源板仅SWD烧录，不套用Slave OTA分区、协议或服务器发布流程。构建说明见documents/powermanage_flasher.md。

调试工作台把实时数值、温压曲线和设备参数放在同页。连接RTT后自动轮询真实状态快照；无设备时显示空值，不生成模拟数据。支持曲线通道开关、30秒/1分钟/5分钟/10分钟窗口、暂停查看、数据录制、JSON离线回放及CSV导出。

App 0.1.2 支持8组PID增益读取与写入：左右温度，以及单眼/双眼的升压、保压、脉冲。进入控制参数页选择温度或压力，先读取再应用；设备执行回执通过才弹出成功提示。参数仅RAM临时生效，重启恢复默认。增益范围0–1000是协议边界，非验证后的治疗参数范围。状态快照约5Hz，PID流约10Hz，曲线显示目标、实测、误差及P/I/D与输出。

“测试与诊断”新增稳定性测试，设置时长、四路温压范围和快照超时后开始。持续统计最小/最大/均值/标准差，记录越限、超时、断连、Tick回退及日志丢弃。到时无异常为PASS；有异常为FAIL；手动结束为ABORTED。支持JSON完整报告及CSV统计导出。测试不自动启动治疗，运行期间禁止PID写入。报告关闭后不保留，请先导出。

稳定性测试压力统一使用 mmHg（1 kPa ≈ 7.50062 mmHg），温度保持 °C。阈值、实时卡片、统计表、事件和导出均采用同一单位；设备通信和调试工作台仍用原有 kPa。测试页显示全程统计及本次测试实时温压曲线，可选30秒/1分钟/5分钟/10分钟窗口；结束后保留曲线。曲线历史最多18000点，全程统计不受历史裁剪影响。JSON报告schema=2，明确标注每个测点的单位、源单位与换算系数。

框架与TPS130设备适配包说明见同包debug_platform_architecture.md。新增设备需自己的测点、参数、协议、工作台和软件图标；不照搬TPS130升级地址。

目标：TPS130 / 9C-U-TPS Slave / STM32G070RBT6。解压完整文件夹后运行 TPS130-Studio.exe，不要单独复制 EXE。

关闭窗口立即退出，不等待烧录、上传或RTT；会结束本工具启动的子进程。中途关闭时当前操作结果未确认，重新打开后先核对设备/服务器状态。

## 首次安装 / SWD 救援

1. 停止治疗。连接 J-Link 的 SWDIO、SWCLK、GND、VTref，使用正常目标板电源。
2. 关闭占用探针的 IDE、RTT Viewer 等程序；多探针时填写探针序列号。
3. 核对 App 版本、HW 修订、Target 0x9C01 及 Factory SHA256。
4. 点击“烧录完整固件”立即写入 0x08000000，包含 Boot + 偏移 App + 两份有效元数据，不再二次确认。
5. 工具执行 J-Link 校验和完整128KiB回读SHA256比较，成功后请求复位运行。只表示Flash与文件一致；App启动需通过业务UART版本与心跳验证。

“仅回读校验”不写Flash，但会暂停CPU并在结束后复位，不应在治疗中使用。无效探针/损坏固件/回读不匹配不能显示成功。

## RTT 调试

断开其他调试会话，点击“连接 RTT”。RTT后台连接固定1000kHz，不主动halt/reset，不访问任意变量。连接本身不修改参数；明确点击应用才发送PID增益写入。若CPU已暂停则报错，需用户在原调试器中恢复。

日志通道0；下行通道0接收help、info、status以及带请求ID的PID get/set/stream命令。PID指令由控制任务执行后回执，3.5秒超时不自动重发。暂停日志显示仍读取数据。日志非阻塞，读取过慢可能丢失，status返回log_drops。

PID二进制采样保留独立上行通道1，通道0提供文本镜像用于工作台解析。PID参数可临时调整，工具不启停治疗。Boot也输出RTT日志，有效App上电约500ms后跳转。

## 后续 OTA

## 上传到 Mender Releases

“上传并创建更新任务”选择新的 `.tpsfw` App包后直接上传，不再二次确认；名称、实际版本及哈希核对结果保留在日志和记录中。工具查询服务器已有 `tps-mcu-1.0.*`，从用户指定的 `tps-mcu-1.0.2-20261111` 起递增patch（1.0.3、1.0.4…），日期段沿用20261111。不会覆盖现有Artifact；上传前再次查重，遇到并发新版本会要求重新准备。烧录、校验、构建及上传成功后显示明显的绿色成功弹窗，失败不会显示成功。

固定兼容类型 `eyemeter-thermal-aarch64`、模块 `eyemeter-tps-mcu`、负载名 `firmware.tpsfw`，与服务器既有包一致。`d0b82a98-300b-41ae-b5d6-530f80fe87cd` 是9C-TPS-130设备ID，仅用于核对device_type。按钮上传核对后，向服务器当前全部兼容 TPS130 分配任务；设备端手动开始并确认写入。

发布名独立于MCU版本：外层名称1.0.2不代表负载App也变成1.0.2；包内 `eyemeter.tps-mcu.version` 永远来自.tpsfw真实版本。需要更新MCU程序版本时先修改Config/version.cmake重新构建。

管理员凭据仅在本机从 `D:/Technology_stack/Web/mender/admin-credentials.json` 读取，绝不打包或记录用户名、密码、JWT。其他发布电脑可在Electron用户数据目录的publisher.json配置credentials路径。服务器依次使用aliyun、hk、oregon、oracle1、oracle2入口，强制验证TLS，上传POST不因断线盲目重发。

按用户最新要求，默认沿用现有TPS未签名格式，无需配置密钥。“发布设置”可切换未签名模式；以后如需签名，可选择TPS私钥和设备已信任公钥，独立验签通过后发布。上传核对会检查服务器实际签名状态与所选模式一致。设备接收策略仍由ARM客户端执行。

发布文件与upload.json记录保存在Windows“文档/TPS130-Published/<发布名>”；原烧录器的TPS100-Published历史文件仍保留，不会删除。其中包括外层Artifact SHA256、负载哈希、服务器Artifact ID和核对结果。上传响应丢失时按同名及哈希查询结果，不直接换入口重复POST；不确定结果必须人工核查后处理本地记录。

“打开固件目录”中的 tps_app.tpsfw 交给ARM上位机工程师，由Mender封装分发；USART3 PD8/PD9，115200/8N1，ARM设备节点现场确认（当前约定/dev/ttyS1）。不要把tps_factory.bin作为UART载荷。

软件包已内置本次Release固件、RTT桥接程序、J-Link运行时和原厂许可，不需要Python/编译器。使用合法SEGGER探针，保留原厂许可；外部分发按SEGGER许可办理。

源码开发：Tools/Flasher 执行 npm ci、npm start。构建固件：python scripts/build_release.py；打包：python scripts/package_flasher.py。详见同包交接文档。当前实机验证标记以release.json为准，不把模拟测试当成硬件验收。


## 上传后创建更新任务（2.1.3）

上传按钮直接执行：核对App、分配发布名、上传核对负载、查询全部设备、筛选device_type=eyemeter-thermal-aarch64、创建批量更新任务并核对目标清单。覆盖发布时已登记的兼容TPS130，包括离线设备；不向服务器其他产品分配任务。不再限制为单个UUID，也不依赖某一参考设备在线。新增设备在下一次发布时纳入，已有任务不会自动扩大。

机器端仍需手动开始并确认写入。成功弹窗显示任务状态和设备数量，日志及upload.json保存目标ID列表。已有同包且目标清单完全一致的任务复用。

upload.json记录deployment_scope=all-compatible及发布时的deployment_devices快照。重试使用同一快照，不随库存增加而扩大；目标兼容类型发生变化时拒绝执行。任务POST响应丢失先查询，不盲目重发。“补建/核对更新任务”选择已有upload.json继续，无需重新上传。旧版单设备发布记录仍按其原始范围恢复，不静默扩大历史任务。
