# Slave Bootloader、App 与 Qt 串口升级设计 v1

状态：已实现第一版固件与打包器，已通过交叉编译和模拟 Flash 测试，待实机联调。适用 STM32G070RBT6；不适用 PowerManage。上位机实施以 `uart_upgrade_handoff.md` 与共享 C 头文件为准。

## 1. 已核实的项目约束与方案

- `.ioc`：STM32G070RBT6；现有链接脚本：128 KiB Flash、36 KiB RAM。
- 当前实际业务通信：`Modules/communication/communication.c` 使用 USART3，PD8 TX / PD9 RX，115200、8N1。USART1 PC4/PC5 用于日志和调试。README 中旧 USART2 描述不能作为实现依据。
- 当前业务帧 AA55、32 字节载荷继续使用；升级数据使用独立帧和独占通信状态。
- 已有 `build/Debug` ELF 的 text=125004、data=568；Flash 约 125572 字节。它是已有产物，不代表当前源码重建结果。108 KiB App 容量必须通过新的 Release 构建验证；不允许超限后继续发布。
- 改造前 Release 基线为 Flash81596字节。加入升级逻辑并偏移链接后，GCC14.3.1构建：Release App Flash82444 / RAM24448字节；Debug（-Og -g3）App Flash90388 / RAM24464字节；Boot Flash7696 / RAM3136字节。容量均满足分区限制。已验证构建和模拟Flash，尚未实机验证；编译仍有原有告警和Boot RAM代码段RWX链接提示，不代表零告警或安全验收。
- v1 采用固定 Boot + 单 App 原地升级。断电可回到 Boot 从头重传；不提供旧版本自动回滚，不承诺断点续传。
- Qt 在原有串口链路上执行升级；若 Qt 运行在 RK3576，就直接使用已连接 USART3 的系统串口。升级时该串口只能有一个拥有者。若 Qt 在 PC，则必须通过实际连接到 USART3 的串口链路，不能默认 USB 可直连 MCU。

## 2. Flash 与 RAM 布局

地址范围均含首尾；Flash 擦除页 2 KiB，常规编程单位为对齐的 8 字节 double-word。

| 区域 | 地址 | 大小 | 页号 | 用途 |
|---|---|---:|---|---|
| Boot | 0x08000000–0x08003FFF | 16 KiB | 0–7 | 上电入口、协议、校验、烧写、跳转 |
| App | 0x08004000–0x0801EFFF | 108 KiB | 8–61 | 当前业务固件；向量表从区首开始 |
| Metadata A | 0x0801F000–0x0801F7FF | 2 KiB | 62 | 镜像有效记录 |
| Metadata B | 0x0801F800–0x0801FFFF | 2 KiB | 63 | 镜像有效记录冗余副本 |

Boot 与 App 独立链接、独立产物；二者共享协议定义，不互相链接业务目标。App FLASH ORIGIN=0x08004000、LENGTH=108K。App 在 SystemInit 中显式设置 SCB->VTOR=0x08004000，不能被生成代码重新写回 Flash 起点。

RAM 顶部预留 32 字节 `0x20008FE0–0x20008FFF` 为 BootMailbox，Boot/App 链接脚本均从普通 RAM 中扣除，`_estack=0x20008FE0`；该区 NOLOAD，不受 data/bss 初始化影响。Mailbox 字段：magic u32、反码 u32、reason u32、反码 u32、保留 16 字节。仅在软件复位标志有效且 magic/反码匹配时接受升级请求，并立即清除。上电/掉电后不依赖 SRAM 内容。

当前芯片单 Bank，擦写时可能暂停 Flash 取指，因此采用一问一答，擦写期间主机绝不连续灌包。Boot 不启动 RTOS、传感器、治疗控制或浮点日志，优先轮询 UART 与超时计时；实测 Boot 超过 16 KiB 时先优化，不能静默挤占 App。

## 3. 上电、App 进入升级、跳转

Boot 首先把加热、气泵、熔断驱动置关闭，并把阀门设为经过硬件核实的安全泄压组合；不能以“所有 GPIO 拉低”替代阀门安全定义。电平表须依据 `bsp_gpio`、`bsp_pwm`、`valve_control` 和硬件实际极性落实。

上电流程：安全输出 → 最小时钟/UART → 检查 Mailbox 与元数据/镜像 → 有有效 App 且无请求时提供 500 ms HELLO 窗口 → 跳 App。成功 HELLO 后保持 Boot；App 无效或升级请求有效时持续等待，不能超时跳入残缺镜像。Boot 不执行任何治疗命令。

App 新增原业务协议命令，保留 ID（实现时再次检查冲突）：

| ID | 方向 | 数据类型/载荷 |
|---|---|---|
| 0x10F0 GET_FW_INFO | Qt→App | UINT8，值 0 |
| 0x11F0 FW_INFO | App→Qt | TEXT，ASCII 版本字符串，不超过 32 字节 |
| 0x10F1 ENTER_BOOT | Qt→App | UINT32，固定确认值 0x31504455 |
| 0x11F1 ENTER_BOOT_ACK | App→Qt | UINT8：0=接受，1=忙，2=安全停机失败，3=参数错误 |

治疗运行、暂停或熔断操作中拒绝 ENTER_BOOT。Qt 先通过原流程停止治疗，再请求进入升级。App 接受后锁住新的启动命令，由控制任务确认停机与安全输出；失败不得复位。成功后 ACK，等待 UART 的 TC（不能仅等待 DMA 完成），停止遥测，写 Mailbox，执行 NVIC_SystemReset。不能从 RTOS 任务直接跳 Boot。

Boot 跳转 App 前必须校验：元数据合法、目标/地址/大小合法、全镜像 CRC 正确、初始 MSP 在 App 普通 SRAM 范围且 8 字节对齐、Reset_Handler 为 Thumb 地址且落在镜像范围。关闭 IRQ、SysTick、DMA、UART 中断和其他 Boot 使用的外设，清除 NVIC enable/pending 与 PendSV/SysTick pending，恢复约定复位时钟，设置 VTOR，DSB/ISB。最终用无栈访问的汇编 trampoline 设置 CONTROL=0、MSP 并跳 Reset_Handler；明确 PRIMASK 交接：最后设置 MSP 后恢复 IRQ 再 BX（所有中断源已经清除），App 正常从启动代码运行。

## 4. 镜像文件 .tpsfw

Qt 使用版本化容器，不允许操作者任意指定 MCU 写入地址。容器=固定 64 字节头+App 原始 bin。App bin 首字节对应 0x08004000，不含 Boot 和元数据。

全部整数小端，逐字段序列化，禁止直接传输 C/Qt 结构体内存。

| 偏移 | 字段 | 类型/规定 |
|---:|---|---|
| 0 | magic | 8 字节 ASCII `TPSFW001` |
| 8 | header_size | u16=64 |
| 10 | format_version | u16=1 |
| 12 | target_id | u32=0x00009C01，项目产品标识，打包器与 Boot 共用 |
| 16 | hw_min | u16，支持的最低硬件修订号 |
| 18 | hw_max | u16，支持的最高硬件修订号 |
| 20 | app_base | u32=0x08004000 |
| 24 | image_size | u32，原始 bin 字节数，范围 8..110592 |
| 28 | image_crc32 | u32，原始 bin CRC |
| 32 | firmware_version | u32，major<<24 / minor<<16 / patch<<8 / build |
| 36 | min_boot_version | u32，同上版本编码 |
| 40 | flags | u32=0，未知标志拒绝 |
| 44 | reserved | 16 字节，全 0 |
| 60 | header_crc32 | u32，覆盖头字节 0..59 |

文件大小必须恰好为 64+image_size；当前板硬件修订号必须在构建/生产配置中有可信来源，不能让 Qt 任意覆盖。CRC32 均使用 CRC-32/ISO-HDLC：反射多项式 0xEDB88320、init=0xFFFFFFFF、xorout=0xFFFFFFFF；`123456789` 校验值 0xCBF43926。最后不足 8 字节用 FF 补齐编程，但镜像 CRC/长度不包含填充。

CRC 用于传输与存储检错，不是签名认证。v1 不提供来源认证或防降级；如产品要求签名验证，应在发布前另行定义密钥、签名容器及 Boot 容量预算。

## 5. Boot 串口帧 v1

固定 115200、8N1、无流控。使用独立同步头，避免与业务 AA55 混淆；Boot 模式不夹杂文本日志。

```text
offset  size  field
0       4     magic = 54 50 53 55 (ASCII TPSU)
4       1     protocol_version = 1
5       1     command
6       2     seq (LE)
8       4     session_id (LE)
12      2     payload_length (LE, 0..264)
14      N     payload
14+N    4     CRC32 (LE, 覆盖 offset 4 起的 10+N 字节)
```

最大帧 282 字节，必须使用独立升级缓冲，不能直接塞入当前业务 32 字节帧或 160 字节 TX 缓冲。根据长度解析，无尾字符，无转义；载荷可包含任意同步头。解析器支持任意拆包/粘包，200 ms 字节间超时丢弃残帧。超长、版本错误、CRC 错误帧丢弃并重新扫描同步头；只有通过完整 CRC 才能响应/执行。正常响应 command=request|0x80，回显 seq 和 session。

Qt 停等发送，同一时刻只有一条未完成请求。seq 从 1 开始递增且会话内不能复用；最大镜像只需 432 个 256 字节数据包。HELLO 的 session=0；Qt 提供非零随机 u32 nonce，Boot 回显该 nonce 作为本次 session（仅隔离旧消息，不用于认证）。相同 nonce/seq HELLO 重发缓存响应；相同 nonce 换 seq 返回 BAD_SEQUENCE。新 nonce HELLO 接管遗留会话，清接收偏移和请求缓存，进入 READY，但不擦除 Flash；之后 BEGIN 从头升级，旧 session 数据被拒绝。这允许上位机进程重启后恢复，不依赖再次掉电。

所有正常响应公共前缀：`status:u16 + boot_state:u8 + reserved:u8(0) + next_offset:u32`，后接命令专用响应。错误时不推进偏移。

| 命令 | 值 | 请求载荷 | 响应附加字段/行为 |
|---|---:|---|---|
| HELLO | 0x01 | nonce:u32 | target:u32, hw_rev:u16, max_chunk:u16(256), boot_ver:u32, app_base:u32, app_capacity:u32, app_valid:u8, reserved:3, app_ver:u32；响应 session=nonce |
| BEGIN | 0x02 | 完整 64 字节镜像头 | 完成头/兼容检查后，先使所有旧有效记录失效并读回确认，再擦除全部 App 页，全部成功才 ACK；next_offset=0 |
| DATA | 0x03 | offset:u32 + 1..256 字节镜像数据 | 检查、编程、读回一致后才 ACK；next_offset=已确认连续字节数 |
| END | 0x04 | 空 | 必须已收到 image_size；Flash 全量 CRC 与向量检查通过后提交元数据，进入 VERIFIED |
| RUN | 0x05 | 空 | 仅 VERIFIED 或原有有效镜像可执行；ACK 等 TC 后系统复位，由 Boot 正常验镜像跳 App |
| STATUS | 0x06 | 空 | 无附加字段；返回状态和当前 RAM 接收偏移 |
| ABORT | 0x07 | 空 | 清除当前接收状态；已擦写后镜像仍无效，保持 Boot；不恢复旧镜像 |

Boot 状态编码：0=WAIT、1=READY、2=RECEIVING、3=VERIFIED、4=ERROR。HELLO 后 WAIT→READY；BEGIN 后 READY→RECEIVING；END 后 RECEIVING→VERIFIED。BEGIN 也允许 VERIFIED 状态开始新升级；ABORT→READY。ERROR 仅允许 STATUS、ABORT、HELLO 重放；复位清会话。

状态码：0 OK、1 BAD_STATE、2 BAD_LENGTH、3 BAD_SESSION、4 BAD_SEQUENCE、5 BAD_TARGET、6 BAD_VERSION、7 OUT_OF_RANGE、8 BAD_OFFSET、9 FLASH_ERROR、10 IMAGE_CRC_ERROR、11 BAD_VECTOR、12 BUSY、13 BAD_HEADER。

数据规则：offset 必须等于 next_offset 且为 8 字节对齐；除最后一个包外固定 256 字节；最后一个包长度恰好填满 image_size。范围计算使用 `length <= image_size-offset`，先验证 offset 不超过 image_size，防止整数溢出；不能写 Boot/元数据。每个 double-word 仅编程一次，不对已经写过的 Flash 重编程，包括全 FF 内容的 ECC 编程。

重传规则：Boot 在 RAM 缓存最近一条已完成的改变状态请求的完整内容与响应（最多 282 字节请求）。相同 session/seq/命令/载荷重复到达时只重发缓存响应，不能重复擦写；相同 seq 内容不同返回 BAD_SEQUENCE。STATUS 不清除此缓存，也不改变接收偏移。BEGIN ACK 丢失不能再次擦除；END ACK 丢失不能再次写元数据。设备复位后会话失效，Qt 必须重新 HELLO/BEGIN 并从 0 传输。

Qt 默认普通命令 1 s、DATA 1 s、BEGIN 10 s、END 3 s 超时；同一请求原样重传最多 3 次，之后报错并保留恢复入口。BEGIN 超时必须沿用同 seq 重试，禁止换 seq 并发开始新擦除。实际超时需结合最低电压与最差擦写时延上板确认。

## 6. 元数据提交与断电恢复

每页仅使用页首 64 字节记录，字段与容器头分开定义：magic:u32、schema:u16、record_size:u16、generation:u32、target_id:u32、app_base:u32、image_size:u32、image_crc32:u32、firmware_version:u32、hw_min:u16、hw_max:u16、reserved:16、record_crc32:u32（覆盖前52字节），最后 offset56 为独立 8 字节 commit 常量 `54 50 53 56 41 4C 49 44`（TPSVALID）。

BEGIN 的破坏性操作顺序不可改变：检查容器 → 擦除 Metadata A/B → 确认两份均无有效记录 → 擦除 App。任一步失败不继续。此后任何复位只能留在 Boot。旧 App 尚未擦除但元数据无效时也不自动启动。

END：从 Flash 读镜像并验 CRC/向量 → 写 A 前56字节并读回 → 最后单独写 commit double-word → 同样提交 B → 验证镜像和记录 → ACK。若 A 有效但 B 失败则报告 FLASH_ERROR，复位仍可能用 A 启动；原 seq END 重试只重发缓存错误，需新 HELLO/BEGIN 从头重传，不重编程已写 double-word。启动检查发现 Flash ECC 读错误时保持 Boot；完整擦除后才清除旧镜像的错误锁存。

启动时每份记录须同时满足 commit、schema、保留字段、record CRC、地址范围、产品及硬件兼容，再验证实际镜像。两份内容相同或仅一份有效可启动；两份有效但内容冲突则留在 Boot，不能凭 generation 选一个未经定义的镜像。序列号/generation 仅诊断使用，不用于回滚或防降级。

因此断电最多丢失 App，不会要求运行残缺 App；Boot 完整时可重新升级。首次量产通过 SWD 同时烧 Boot、偏移后的 App、有效元数据；仅烧 App bin 将不会被 Boot 启动。后续串口升级不允许更新 Boot。量产可对 Boot 页启用 WRP，不能误保护 App 页；更改 option bytes 由生产流程独立处理。

## 7. Qt 升级流程

```text
选文件/本地验头与 CRC → 查询 App 版本 → 停止治疗
→ ENTER_BOOT/等待 ACK → 暂停业务发送和遥测解析
→ 重试 HELLO 等待 Boot → 验产品/硬件/Boot版本/容量
→ BEGIN → DATA(逐包等 ACK) → END → RUN
→ 等待 App 重新上线与版本核对 → 升级成功
```

Qt 使用 QSerialPort 的异步 readyRead 和 QTimer 驱动状态机，不阻塞 GUI；串口接收统一维护 QByteArray 流缓冲；明确处理 partial write、bytesWritten、端口断开和迟到响应。进度按 Boot ACK 的 next_offset 计算，不按 write() 接受的字节计算。

END 成功仅表示镜像提交成功；RUN 后 App 版本查询与业务心跳均恢复才显示“升级成功”。超时显示“镜像已写入，尚未确认应用启动”，不能直接认为固件无效或重新擦除。Boot 的 CRC 检查不能证明 App 的业务功能正确。

取消只在当前命令完成后发送 ABORT；提示需重新升级。Qt 退出或线缆断开后，再次连接先 HELLO/查询状态。设备在 Boot 无有效 App 时允许直接升级，不依赖 App ENTER_BOOT 命令。有效但故障的 App 可通过重新上电的 500 ms 窗口持续尝试 HELLO，SWD 保留为维护恢复入口。

## 8. 实施拆分和验收

建议目录：`Bootloader/`（独立 CMake/启动/板级安全输出/UART/Flash/状态机）；`Shared/Upgrade/`（纯 C 定义、LE 编解码、CRC、容器定义）；`Modules/Upgrade/`（App 停机握手/Mailbox/复位）；`Tools/FirmwarePack/`（bin 打包及元数据/合并镜像）；Qt 工程接入共享协议规范，不依赖 MCU HAL。

实施顺序：
1. 重建当前 Release，记录 Flash/RAM/栈余量；确认 App <=108 KiB；不满足先评估 -Os/LTO、浮点 printf 与日志体积，不能直接裁掉功能。
2. 实现共享编解码与打包器，生成确定的跨语言测试向量；测试 CRC、LE、空帧、任意拆包粘包、错误长度、同步头嵌入载荷、超时恢复。
3. 建 Boot/App 独立构建与链接检查，保留现有普通构建目标直到新链路通过；验证 VTOR、FreeRTOS SVC/PendSV/SysTick、DMA、中断均可在偏移后工作。
4. 实现烧写、读回、元数据，覆盖 BEGIN/DATA/END ACK 丢失、重复包、乱序、跨界、最后非8字节数据、错误产品和硬件版本。
5. 在元数据擦除、App 擦除、double-word 编程、两个 commit 前后逐点断电；每次应只启动完整已验镜像或停留 Boot，重传可恢复。
6. 对照实物验证整个升级期间泵/加热/熔断不动作、阀门安全；验证 App 停机失败时拒绝升级，以及 Qt 重连/重启/取消/成功后心跳版本恢复。

来源：本仓库 `.ioc`、链接脚本、实际通信实现与 HAL；芯片 Flash 页/编程要求参考 [ST RM0454](https://www.st.com/resource/en/reference_manual/rm0454-stm32g0b0-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) 及 [ST HAL Flash 实现](https://github.com/STMicroelectronics/stm32g0xx-hal-driver/blob/master/Src/stm32g0xx_hal_flash.c)。
