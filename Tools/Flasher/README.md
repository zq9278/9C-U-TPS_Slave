# TPS100-Flasher 1.0.0

目标：9C-U-TPS Slave / STM32G070RBT6。解压完整文件夹后运行 TPS100-Flasher.exe，不要单独复制 EXE。

## 首次安装 / SWD 救援

1. 停止治疗。连接 J-Link 的 SWDIO、SWCLK、GND、VTref，使用正常目标板电源。
2. 关闭占用探针的 IDE、RTT Viewer 等程序；多探针时填写探针序列号。
3. 核对 App 版本、HW 修订、Target 0x9C01 及 Factory SHA256。
4. 点击“烧录完整固件”，二次确认后写入 0x08000000，包含 Boot + 偏移 App + 两份有效元数据。
5. 工具执行 J-Link 校验和完整128KiB回读SHA256比较，成功后请求复位运行。只表示Flash与文件一致；App启动需通过业务UART版本与心跳验证。

“仅回读校验”不写Flash，但会暂停CPU并在结束后复位，不应在治疗中使用。无效探针/损坏固件/回读不匹配不能显示成功。

## RTT 调试

断开其他调试会话，点击“连接 RTT”。RTT后台连接固定1000kHz，不主动halt/reset，不改治疗参数、不访问任意变量。若CPU已暂停则报错，需用户在原调试器中恢复。

日志通道0；下行通道0接收只读文本命令：help、info、status。界面的版本/运行状态按钮只发送对应命令，固件回复显示在日志区；“命令已发送”不表示已收到固件回复。暂停显示仍会读取日志，避免阻塞目标。日志采用非阻塞模式，未连接或读取过慢可能丢失；status返回log_drops。

PID二进制采样使用独立上行通道1；此工具不修改PID，不启停治疗。Boot也可输出RTT日志，但有效App上电约500ms后跳转，连接时通常看到App。

## 后续 OTA

“打开固件目录”中的 tps_app.tpsfw 交给ARM上位机工程师，由Mender封装分发；USART3 PD8/PD9，115200/8N1，ARM设备节点现场确认（当前约定/dev/ttyS1）。不要把tps_factory.bin作为UART载荷。

软件包已内置本次Release固件、RTT桥接程序、J-Link运行时和原厂许可，不需要Python/编译器。使用合法SEGGER探针，保留原厂许可；外部分发按SEGGER许可办理。

源码开发：Tools/Flasher 执行 npm ci、npm start。构建固件：python scripts/build_release.py；打包：python scripts/package_flasher.py。详见同包交接文档。当前实机验证标记以release.json为准，不把模拟测试当成硬件验收。
