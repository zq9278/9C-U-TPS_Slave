# TPS130 Studio 2.0.0

本工具已从单页烧录器重构为设备调试平台：设备概览、TPS130专属工作台、固件与发布、检查与诊断、日志与记录、连接设置。运行TPS130-Studio.exe；新产品图标已写入窗口、任务栏和EXE。

调试工作台把实时数值、温压曲线和设备参数放在同页。连接RTT后自动轮询真实状态快照；无设备时显示空值，不生成模拟数据。支持曲线通道开关、30秒/1分钟/5分钟/10分钟窗口、暂停查看、数据录制、JSON离线回放及CSV导出。

当前设备参数仅支持读取身份信息；PID/治疗参数写入尚无确认协议，明确显示未接入，不发送控制命令。实时值来自现有固件RTT状态快照，约5Hz请求，不代表传感器原始采样率。数据过期、断线和回放模式明确区分。

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

断开其他调试会话，点击“连接 RTT”。RTT后台连接固定1000kHz，不主动halt/reset，不改治疗参数、不访问任意变量。若CPU已暂停则报错，需用户在原调试器中恢复。

日志通道0；下行通道0接收只读文本命令：help、info、status。界面的版本/运行状态按钮只发送对应命令，固件回复显示在日志区；“命令已发送”不表示已收到固件回复。暂停显示仍会读取日志，避免阻塞目标。日志采用非阻塞模式，未连接或读取过慢可能丢失；status返回log_drops。

PID二进制采样使用独立上行通道1；此工具不修改PID，不启停治疗。Boot也可输出RTT日志，但有效App上电约500ms后跳转，连接时通常看到App。

## 后续 OTA

## 上传到 Mender Releases

“上传新包到服务器”选择新的 `.tpsfw` App包后直接上传，不再二次确认；名称、实际版本及哈希核对结果保留在日志和记录中。工具查询服务器已有 `tps-mcu-1.0.*`，从用户指定的 `tps-mcu-1.0.2-20261111` 起递增patch（1.0.3、1.0.4…），日期段沿用20261111。不会覆盖现有Artifact；上传前再次查重，遇到并发新版本会要求重新准备。烧录、校验、构建及上传成功后显示明显的绿色成功弹窗，失败不会显示成功。

固定兼容类型 `eyemeter-thermal-aarch64`、模块 `eyemeter-tps-mcu`、负载名 `firmware.tpsfw`，与服务器既有包一致。`d0b82a98-300b-41ae-b5d6-530f80fe87cd` 是9C-TPS-130设备ID，仅用于核对device_type。按钮仅上传Releases，不创建部署或触发安装。

发布名独立于MCU版本：外层名称1.0.2不代表负载App也变成1.0.2；包内 `eyemeter.tps-mcu.version` 永远来自.tpsfw真实版本。需要更新MCU程序版本时先修改Config/version.cmake重新构建。

管理员凭据仅在本机从 `D:/Technology_stack/Web/mender/admin-credentials.json` 读取，绝不打包或记录用户名、密码、JWT。其他发布电脑可在Electron用户数据目录的publisher.json配置credentials路径。服务器依次使用aliyun、hk、oregon、oracle1、oracle2入口，强制验证TLS，上传POST不因断线盲目重发。

按用户最新要求，默认沿用现有TPS未签名格式，无需配置密钥。“发布设置”可切换未签名模式；以后如需签名，可选择TPS私钥和设备已信任公钥，独立验签通过后发布。上传核对会检查服务器实际签名状态与所选模式一致。设备接收策略仍由ARM客户端执行。

发布文件与upload.json记录保存在Windows“文档/TPS130-Published/<发布名>”；原烧录器的TPS100-Published历史文件仍保留，不会删除。其中包括外层Artifact SHA256、负载哈希、服务器Artifact ID和核对结果。上传响应丢失时按同名及哈希查询结果，不直接换入口重复POST；不确定结果必须人工核查后处理本地记录。

“打开固件目录”中的 tps_app.tpsfw 交给ARM上位机工程师，由Mender封装分发；USART3 PD8/PD9，115200/8N1，ARM设备节点现场确认（当前约定/dev/ttyS1）。不要把tps_factory.bin作为UART载荷。

软件包已内置本次Release固件、RTT桥接程序、J-Link运行时和原厂许可，不需要Python/编译器。使用合法SEGGER探针，保留原厂许可；外部分发按SEGGER许可办理。

源码开发：Tools/Flasher 执行 npm ci、npm start。构建固件：python scripts/build_release.py；打包：python scripts/package_flasher.py。详见同包交接文档。当前实机验证标记以release.json为准，不把模拟测试当成硬件验收。
