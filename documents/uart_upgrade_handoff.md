# 给 Qt 上位机工程师：Slave 串口升级交接 v1

本文件描述当前已实现的固件接口。Qt 界面与升级客户端由上位机工程师实现；仓库未包含 Qt 工程修改。

## 交付与边界

- 分支：`codex/bootloader-uart-update`，目标 STM32G070RBT6。
- 串口：USART3 / PD8 TX、PD9 RX，115200、8N1、无流控；Qt 仍使用当前与下位机连接的串口。USART1 是日志口，不接收升级。
- Boot 起点 0x08000000 / 16 KiB；App 起点 0x08004000 / 108 KiB；元数据 0x0801F000 和 0x0801F800，各 2 KiB。
- 单 App 原地升级：开始擦除后旧 App 丢失；断电、取消、断线后可重新传输，不支持断点续传或回滚。Boot 不接受串口自更新。
- 打包文件 `.tpsfw` 才是 Qt 输入，不能把 ELF/HEX/合并烧录文件当作升级载荷。
- 首次通过 SWD 在 0x08000000 烧 `tps_factory.bin`；串口升级使用 `tps_app.tpsfw`。不能直接把偏移 App bin 烧到 0x08000000。
- 新 App 保留原业务协议，升级过程中串口必须独占，暂停心跳定时发送、参数同步、遥测消费等普通通信。恢复业务前重新初始化普通接收状态机。
- 当前 Boot 版本编码 0x00010000，即 **0.1.0.0**；App 当前为 **0.1.1**，从Config/version.cmake生成回复版本和二进制身份。打包器读取实际编译身份，禁止包头和App回复不同。
- 硬件修订号由 Boot 构建参数 `TPS_HW_REV` 指定，当前默认 1；生产时须对照实际板型确认。上位机不能修改设备返回的硬件编号来绕过检查。

## 1. App 进入 Boot

继续使用原业务帧：`AA 55 | id:u16 | type:u8 | len:u16 | payload | CRC16:u16 | 0D 0A`。
整数小端；CRC16/Modbus 覆盖 id..payload，init FFFF、反射多项式 A001。

| ID | type | payload | 行为 |
|---|---:|---|---|
| 0x10F0 | 1 UINT8 | 00 | 读取 App 版本 |
| 0x11F0 | 3 TEXT | ASCII 版本字符串，如 `0.1.1` | 无末尾 NUL，长度 <=31 |
| 0x10F1 | 5 UINT32 | `55 44 50 31`（0x31504455 小端） | 请求进入 Boot |
| 0x11F1 | 1 UINT8 | 00 / 01 / 02 / 03 | 接受 / 忙 / 安全输出确认失败 / 参数错误或普通非升级构建 |

升级前请让设备停止治疗：现有 `0x10C2`（type=1，payload=01），再发送 ENTER_BOOT。运行、暂停、熔断中拒绝升级。不要用直接断电代替治疗停止。

固件在控制任务中确认泵、加热和阀门的空闲输出后 ACK=0，并等待 UART 最后停止位发完再软件复位。ACK=1 时恢复普通通信并提示先停止操作；ACK=2 时输出保持锁定，允许重试 ENTER_BOOT，不能显示进入 Boot 成功。

ENTER_BOOT 等待 ACK 建议 2 s（内部安全握手最多 1 s）；超时可重发请求。ACK 丢失时设备可能已进入 Boot，因此也应尝试 HELLO，不要只等业务帧。Boot 无有效 App 时可直接 HELLO，无需 ENTER_BOOT。

## 2. Boot 帧与接收器

```text
54 50 53 55 | ver:u8=01 | cmd:u8 | seq:u16 | session:u32 | len:u16 | data[len] | crc32:u32
```

| 字节偏移 | 内容 |
|---:|---|
| 0..3 | ASCII TPSU |
| 4 | 协议版本 1 |
| 5 | 命令 |
| 6..7 | 序号，小端 |
| 8..11 | 会话，小端 |
| 12..13 | 载荷长度，小端，0..264 |
| 14..13+N | 载荷，允许任意二进制内容 |
| 14+N..17+N | CRC32，小端 |

总长度 18+N，最大 282。没有帧尾、没有转义。CRC32 仅覆盖 offset4 起 10+N 字节。算法为 CRC-32/ISO-HDLC：反射多项式 EDB88320、init FFFFFFFF、xorout FFFFFFFF；`123456789` → CBF43926。等价 Python `zlib.crc32(data)`。Qt 显式读写 LE，不能 memcpy C++ struct。

readyRead 可能拿到半帧、多帧或任意拆分。请使用流缓冲按 magic/长度/CRC 拆帧，200 ms 字节间超时清残帧，长度超过264先拒绝；CRC错误或版本错误帧丢弃后继续找头。载荷中出现 TPSU 不能被当作新帧头。Boot 对不完整/CRC坏帧不回 NACK。

## 3. 请求与响应

每次只发一个请求，等待响应后再发下一个。请求 seq 从1递增（不回绕），重传使用完全相同的 seq、session、载荷。响应 cmd=request_cmd|80，回显 seq 和 session。仅接受当前请求对应的响应；迟到/其他会话响应丢弃。

所有响应 data 前8字节：`status:u16 | state:u8 | reserved:u8=0 | next_offset:u32`。next_offset 是已完成编程并读回的连续字节数。错误响应偏移不前进。除 HELLO 外当前响应长度都是8。

| cmd | 请求 data | 成功响应/要求 |
|---:|---|---|
| 01 HELLO | nonce:u32，非零随机值；帧session=0 | 帧session=nonce；额外28字节信息，见下表 |
| 02 BEGIN | 文件前64字节原样发送 | 擦除完成后 ACK；next_offset=0 |
| 03 DATA | offset:u32 + 1..256字节 App 数据 | ACK 后按 next_offset 发下一包 |
| 04 END | 空 | 全镜像验证与元数据提交完成后 ACK |
| 05 RUN | 空 | ACK 后设备复位，重新启动 App |
| 06 STATUS | 空 | 状态/偏移，仅诊断，无持久续传功能 |
| 07 ABORT | 空 | 回到 READY，已擦除的旧 App 无法恢复 |

HELLO 返回 data：

| data偏移 | 类型 | 内容 |
|---:|---|---|
| 0..7 | 公共前缀 | status/state/reserved/next_offset |
| 8 | u32 | target_id=0x9C01 |
| 12 | u16 | hw_rev，当前默认1 |
| 14 | u16 | max_chunk=256 |
| 16 | u32 | boot_version=0x00010000 |
| 20 | u32 | app_base=0x08004000 |
| 24 | u32 | app_capacity=110592 |
| 28 | u8 | app_valid=0/1 |
| 29..31 | 3字节 | 保留0 |
| 32 | u32 | 已有效 App 的版本编码，无有效 App 时0 |

HELLO 注意：相同 nonce/seq 重发会获得最初 HELLO 的缓存响应；新 nonce 开始新会话，丢弃旧接收偏移和缓存、不擦除 Flash，必须重新 BEGIN。该行为供 Qt 重启/重连恢复；旧 session DATA 会被拒绝。禁止两份升级程序同时控制同一串口。

DATA 的 offset 相对于 App bin（文件偏移64），首包0。除末包外严格256字节；末包恰好为剩余字节数（可不是8的倍数）。固件自行以FF补足8字节编程，Qt 不要补齐文件、长度或CRC。例：521字节镜像，DATA 依次 offset0/256字节、256/256字节、512/9字节。

BEGIN/DATA/END 等改变状态的最近请求会在 RAM 缓存；响应丢失后原样重发只回缓存，不重擦写。相同seq改内容=BAD_SEQUENCE。STATUS不覆盖该缓存。新的会话或设备复位后缓存丢失，必须从头传。

状态：0 WAIT、1 READY、2 RECEIVING、3 VERIFIED、4 ERROR。

错误码：0 OK；1 BAD_STATE；2 BAD_LENGTH；3 BAD_SESSION；4 BAD_SEQUENCE；5 BAD_TARGET；6 BAD_VERSION；7 OUT_OF_RANGE；8 BAD_OFFSET；9 FLASH_ERROR；10 IMAGE_CRC_ERROR；11 BAD_VECTOR；12 BUSY（保留）；13 BAD_HEADER。

status非0时停止正常发送；向用户显示阶段和错误码。Flash/CRC/序号/会话异常可重新 HELLO（新nonce）→BEGIN，从头传。不要把错误响应当成功ACK继续增加offset。

超时建议：HELLO/DATA/STATUS/ABORT/RUN为1 s，BEGIN为10 s，END为3 s；同包最多重传3次（首次+3次=4次）。串口 bytesWritten 仅代表本地写入，不是设备确认。写入不足时继续发送剩余字节，不能在旧包尚未交完时重排整包。

## 4. 文件格式与成功判定

`.tpsfw`=64字节头+App bin。完整字段见设计文档第4节及 `Tools/FirmwarePack/pack_firmware.py`。

Qt 选文件时先验证：magic TPSFW001、header_size64、format_version1、文件大小=64+image_size、header CRC(offset60，覆盖0..59)、image CRC(offset28，仅App bin)、保留位0、App地址与容量。HELLO 后再比较 target_id、hw_min..hw_max、min_boot_version、设备容量，不兼容不得发 BEGIN。

版本编码 major<<24 | minor<<16 | patch<<8 | build。不要将版本编码按十进制字符串比较。CRC 仅检错，不验证发行来源，当前无签名/防降级。

顺序：文件检查 → 停治疗 → ENTER_BOOT → HELLO → BEGIN → DATA逐包 → END → RUN → App版本查询0x10F0及心跳0x1000。

只有 App 回复0x11F0的版本与文件头offset32一致，且0x1100心跳ACK=1，才提示升级成功。END ACK仅证明镜像写入验证成功。RUN ACK丢失时也应尝试重新识别 App；不能因此直接重新擦除。建议 App重连确认窗口5 s，超时报“镜像已写入，尚未确认应用启动”。

进度=DATA ACK 的 next_offset/image_size；另显示擦除、校验、重启、确认等阶段。取消在当前请求完成后发 ABORT；END已成功提交则继续RUN与启动确认，避免界面显示取消但设备已更新。

Boot上电无升级请求且App有效时监听HELLO约500 ms再跳App；App无效或接受ENTER_BOOT后持续等待。故障App恢复可在重新上电时发送HELLO；这个窗口应实测串口打开延迟。保留SWD维护入口。

## 5. 构建、打包与测试

在仓库根目录 PowerShell：

```powershell
cmake --preset Release
cmake --build --preset Release -j 8
cmake -S . -B build/Boot -G Ninja '-DCMAKE_BUILD_TYPE=Release' '-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake' '-DTPS_BUILD_BOOT=ON'
cmake --build build/Boot -j 8
python Tools/FirmwarePack/pack_firmware.py build/Release/tps_app.bin --output build/Release/tps_app.tpsfw --boot build/Boot/tps_boot.bin --factory-output build/Release/tps_factory.bin
python -m unittest discover -s Tools/FirmwarePack -v
```

普通旧地址固件可在独立构建目录设置 `-DTPS_LEGACY_APP=ON`，该模式拒绝ENTER_BOOT。默认构建现在是偏移App；Debug使用 `-Og -g3`，两种配置都受108KiB链接限制。

本轮OTA+RTT的GCC14.3.1构建：Boot Flash8360 / RAM4584字节；Release App Flash84012 / RAM26488字节；Debug App Flash92100 / RAM26504字节。均满足分区容量。RAM含链接脚本预留堆栈，不能替代运行时高水位检查。统一交付可执行 `python scripts/build_release.py`，产物在out/Release。

跨语言固定测试向量：`Tools/FirmwarePack/protocol_vectors.json`；生成脚本 `make_vectors.py`。每条包含完整wire_hex和字段说明，可与 Qt编解码进行逐字节比对。

纯C模拟Flash测试 `Tools/FirmwarePack/test_boot.c` 使用真实Boot状态机与CRC代码，覆盖包重复、偏移、拆包、坏CRC、元数据冗余与擦写中断恢复。编译命令（Linux/WSL，仓库根目录）：

```sh
gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g -I. Shared/Upgrade/upgrade.c Bootloader/boot_engine.c Tools/FirmwarePack/test_boot.c -o build/tps_boot_test
./build/tps_boot_test
python3 Tools/FirmwarePack/make_vectors.py --check-binary ./build/tps_boot_test
./build/tps_boot_test --factory build/Release/tps_factory.bin
```

必须实机联调：首次factory烧录、偏移App所有中断/FreeRTOS、UART口电平与波特率、控制安全输出实测、ACK丢包/断线/取消/重连、每类擦写/commit阶段掉电。模拟测试不覆盖真实掉电过程、Flash ECC或阀门气路。未完成这些验证前不作为生产已验收固件。
