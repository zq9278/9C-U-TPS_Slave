# OTA 改造需求对照与交付记录（2026-11-11）

依据用户提供的《下位机OTA改造需求.md》，在 codex/bootloader-uart-update 上继续实现。该文档的协议、地址、Target、版本编码全部保留，未要求ARM客户端修改接口。

## 已完成的软件改造

| 需求 | 实现/验证 |
|---|---|
| G070RBT6，Boot16K/App108K/Metadata4K，mailbox32B | 独立链接脚本；编译器强制容量；App启动设置VTOR；factory镜像校验 |
| USART3 PD8/PD9 115200 8N1，Target9C01 | 原业务/OTA实现保留；ARM节点/dev/ttyS1需现场确认 |
| 版本查询、心跳、停治疗、进入Boot | ID/type/载荷不变；App版本由Config/version.cmake唯一生成 |
| 运行/暂停/熔断拒绝升级、安全输出握手 | App/Control任务现有握手保留；真实电平/气路待实测 |
| TPSU帧，CRC32，HELLO..ABORT，重传，末包 | 真实C状态机模拟测试及跨语言固定向量；不改变帧 |
| 新nonce接管遗留会话 | 清RAM接收状态，重新BEGIN；无持久续传 |
| END有效标记、元数据冗余、启动CRC | 模拟139个擦写边界中断与恢复；真实掉电/ECC待实测 |
| RUN ACK完成后才复位 | 只有当前RUN成功响应已发送才复位，防止其他响应触发遗留reset标志 |
| App .tpsfw + factory +版本/SHA/source/toolchain | scripts/build_release.py自动构建；out/Release/release.json与SHA256SUMS.txt |
| 包版本必须与App回复一致 | App内固定TPSID001编译身份，打包器读取实际版本；手工错误--version被拒绝 |
| 日志/调试与OTA分离 | App与Boot非阻塞SEGGER RTT；App只读help/info/status；不占USART3 |
| 一体烧录调试工具 | TPS100-Flasher.exe，内置当前Release与RTT桥；SWD烧录/完整128K回读SHA、RTT互斥 |

App版本在本轮设为 **0.1.1**；Boot保持 **0.1.0.0**（00010000）。硬件修订沿用构建默认1，需工程师核对实板；可用--hw-rev显式构建，包硬件范围自动等于该修订号。不要通过修改包头绕过板型核对。

## 可复现命令（仓库根目录 PowerShell）

```powershell
python scripts/build_release.py --hw-rev 1
python -m unittest discover -s Tools/FirmwarePack -v
python -m unittest discover -s Tools/Flasher -p test_rtt_bridge.py -v
node --test Tools/Flasher/backend.test.cjs
python scripts/test.py
python scripts/package_flasher.py --skip-build
```

首次准备Electron：Tools/Flasher目录运行npm ci；Python打包依赖见Tools/Flasher/requirements.txt；也可给package_flasher.py传--electron指向现成electron/dist。默认J-Link安装目录C:/Program Files/SEGGER/JLink_V954，可传--jlink。源码快照发生变化时不能--skip-build复用旧固件；软件测试未通过或报告对应旧源码也拒绝打包。

`out/Release/`：tps_app.tpsfw用于Mender分发/UART；tps_factory.bin从0x08000000首次SWD安装；Boot/App ELF和BIN供调试；release.json记录版本、硬件、GCC版本、源码提交、dirty标记及完整源码快照hash；SHA256SUMS.txt记录各文件校验值。

`out/distribution/`：完整可运行Windows文件夹和同名ZIP，内置同一次Release的全部文件；LATEST.txt指向最新文件夹。工具只允许内置完整镜像固定地址烧录，不支持任意地址写入、擦option bytes或修改读写保护。

软件验证结果：Python容器/身份3项、RTT环形缓冲/只读命令3项、Node烧录后端6项通过；实际C Boot状态机139个擦写边界、11个跨语言帧向量、factory验镜像通过；实际SEGGER RTT源码与固件只读命令测试通过（10000次满缓冲写不阻塞）。启用ASan/UBSan执行原生C测试。Boot、Release及Debug App均编译成功。桌面GUI无探针冒烟已通过；这些结果不会把hardware_validation改为已验收。

## 软件验证与实机验收边界

已执行的编译、C状态机测试、Python打包/RTT环形缓冲测试、Node烧录后端测试、桌面程序无探针冒烟和截图由交付报告记录。模拟器证明软件路径行为，不能证明实物输出安全、Flash电气特性或5s内App上线。

以下本轮均为 **NOT_RUN / 待工程师实机验收**，不得填“通过”：

1. 首次SWD真实烧录、偏移App所有中断、FreeRTOS任务/计时器。
2. /dev/ttyS1节点、接线、电平、UART独占；正常OTA及5s内版本+心跳。
3. 运行/暂停/熔断拒绝升级；泵/加热/阀门安全电平和泄压实测。
4. 真实ACK丢失、线缆中断、CRC/offset/seq/目标包错误。
5. 擦除、DATA、END、两份元数据commit中真实掉电与完整重刷。
6. J-Link实物驱动/探针、FLASH回读、RTT持续运行及无探针日志负载。

后端测试的模拟“回读成功”不是实板烧录记录。界面烧录成功也仅证明写入文件和Flash回读一致，业务启动成功仍按上位机版本/心跳判定。

## 上位机交接

串口协议无差异。只提供新的、版本可追溯的tps_app.tpsfw与校验值；Mender不要封装factory作为UART载荷。完整字段及超时见uart_upgrade_handoff.md；RTT约定见rtt_debug.md。源需求副本保留于input_ota_requirements_20261111.md。
