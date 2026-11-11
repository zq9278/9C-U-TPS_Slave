# 下位机 OTA 改造需求（供嵌入式工程师自行实现）

上位机 ARM 软件已接入 Mender 应用更新及下位机串口升级客户端。本交接仅提供接口和验收要求，不提供编译固件或源码补丁；请嵌入式工程师在自己的下位机工程中完成改造、编译、首次烧录和硬件验证。

以下是当前上位机客户端使用的接口约定。若下位机需要更换芯片、串口、Flash 地址、Target ID 或协议字段，请先同步上位机实现，不能只改一端。

1. **总体职责与结构**

   - Mender 运行在 ARM Linux 上位机；下位机通过业务 UART 接收 App 固件，由下位机 Bootloader 擦写 Flash。
   - 请实现 Bootloader 与可独立链接的 App，保留 SWD 恢复入口。首次安装 Bootloader 由嵌入式工程师通过 SWD 完成，后续通过 UART 更新 App。
   - 当前目标为 STM32G070RBT6。Boot 从 `0x08000000` 开始占 16 KiB；App 从 `0x08004000` 开始占 108 KiB；元数据 A/B 分别在 `0x0801F000`、`0x0801F800`，各 2 KiB。
   - RAM `0x20008FE0` 起的 32 B 为 Boot mailbox，Boot/App 链接和启动代码保持一致。
   - 当前按单 App 区设计，不支持固件回滚和持久断点续传。上位机 AppImage 回滚能力不能用于下位机。

2. **串口及身份约定**

   - MCU：USART3，PD8 TX / PD9 RX，115200、8N1、无流控。日志口与升级口区分。
   - ARM 当前串口配置 `/dev/ttyS1`，现场确认设备节点、接线与电平。
   - Target ID `0x9C01`，硬件修订号由实际板型确定；当前客户端不把返回的硬件修订号写死为 1，而是与固件包声明的允许范围比较。
   - App 版本由固件工程师维护，格式 `major.minor.patch[.build]`，每段 0..255；三段格式按 build=0 处理。程序回复的版本与固件包头必须一致。
   - 版本编码为 `major<<24 | minor<<16 | patch<<8 | build`，不能按版本文本或十进制数字字符串比较。

3. **普通业务协议需要提供的接口**

   帧：`AA 55 | id:u16 | type:u8 | len:u16 | payload | CRC16:u16 | 0D 0A`，整数小端。CRC16/Modbus 覆盖 id 至 payload，init `FFFF`、反射多项式 `A001`。

   | 功能 | 请求 | 回复 |
   | --- | --- | --- |
   | 停治疗 | `0x10C2`, type=1, payload=`01` | 沿用原业务停机流程 |
   | 读固件版本 | `0x10F0`, type=1, payload=`00` | `0x11F0`, type=3，ASCII 版本字符串，长度 1..31，无末尾 NUL |
   | 心跳 | `0x1000`, type=1, payload=`01` | `0x1100`, type=1, payload=`01` |
   | 进入升级模式 | `0x10F1`, type=5, payload=`55 44 50 31` | `0x11F1`, type=1，单字节状态 |

   进入升级状态：0 接受；1 忙；2 安全输出确认失败；3 参数错误或不支持升级。

   治疗运行、暂停、熔断时拒绝进入升级。请在控制任务中确认泵、加热、阀门进入安全状态，ACK 发送完成后再复位进入 Boot；不能仅停止通信任务后直接复位。

4. **Bootloader 升级帧**

   ```text
   TPSU | ver:u8=1 | cmd:u8 | seq:u16 | session:u32 | len:u16 | data[len] | crc32:u32
   ```

   整数小端，CRC32/ISO-HDLC 覆盖 ver 至 data；与 Python `zlib.crc32` 相同。最大 data 264 B、最大整帧 282 B；按流处理拆包、粘包和载荷内的 magic，200 ms 字节间超时清理残帧。

   响应 cmd 为请求 cmd OR `0x80`，回显 seq/session。响应 data 前 8 B：`status:u16 | state:u8 | reserved:u8=0 | next_offset:u32`。

   | 命令 | 请求载荷 | 要求 |
   | --- | --- | --- |
   | 1 HELLO | nonce:u32，非零；请求 session=0 | 以 nonce 建立会话，返回型号、硬件、Boot 版本、App 地址/容量/有效性/版本 |
   | 2 BEGIN | `.tpsfw` 前 64 B 原始包头 | 全部兼容性检查通过后才擦 App；擦除完成回复 offset=0 |
   | 3 DATA | offset:u32 + 1..256 B App 数据 | offset 相对 App bin；写入并读回后 ACK 下一个 offset |
   | 4 END | 空 | 检查完整镜像 CRC、向量并提交有效元数据 |
   | 5 RUN | 空 | ACK 发送完成后复位启动有效 App |
   | 6 STATUS | 空 | 返回当前状态/偏移，用于诊断 |
   | 7 ABORT | 空 | 结束当前接收，不恢复已擦除的旧 App |

   HELLO 响应长度 36 B：公共前缀 8 B；偏移 8 target:u32，12 hw:u16，14 max_chunk:u16=256，16 boot_version:u32，20 app_base:u32，24 app_capacity:u32，28 app_valid:u8，29..31 为 0，32 app_version:u32。

   状态：0 WAIT、1 READY、2 RECEIVING、3 VERIFIED、4 ERROR。错误码：0 OK、1 BAD_STATE、2 BAD_LENGTH、3 BAD_SESSION、4 BAD_SEQUENCE、5 BAD_TARGET、6 BAD_VERSION、7 OUT_OF_RANGE、8 BAD_OFFSET、9 FLASH_ERROR、10 IMAGE_CRC_ERROR、11 BAD_VECTOR、12 BUSY、13 BAD_HEADER。

   相同 seq/session/载荷的重发只返回原结果，不能重复擦写；同 seq 改内容返回 BAD_SEQUENCE。新 nonce 开新会话后从 BEGIN 重新开始。末包按真实长度发送，固件自行补齐 Flash 编程单位，不能改变包头镜像长度和 CRC。

   上位机请求串行发送，首次加重传最多 4 次；BEGIN 等待 10 s，END 3 s，其他 Boot 请求 1 s。RUN ACK 丢失后上位机会尝试确认 App，不能因此再次擦除。

5. **请输出可校验的 App 固件包**

   `.tpsfw` 为 64 B 包头加偏移 App bin。包头约定如下，整数小端：

   | 偏移 | 字段 |
   | --- | --- |
   | 0..7 | magic `TPSFW001` |
   | 8 | header_size:u16=64 |
   | 10 | format_version:u16=1 |
   | 12 | target:u32=`0x9C01` |
   | 16 / 18 | hw_min:u16 / hw_max:u16 |
   | 20 | app_base:u32=`0x08004000` |
   | 24 | image_size:u32，最大 110592 |
   | 28 | image_crc32:u32，仅覆盖 App bin |
   | 32 | app_version:u32 |
   | 36 | min_boot_version:u32 |
   | 40..59 | 保留为 0 |
   | 60 | header_crc32:u32，覆盖包头 0..59 |

   文件大小必须等于 `64+image_size`，初始 SP/复位向量必须符合保留 mailbox 后的 RAM 和偏移 App 地址。当前客户端最低 Boot 版本取自包头；现有打包工具填写 `0x00010000`（0.1.0.0）。

   请同时准备首次 SWD 所需的 Boot+App+元数据 factory 镜像，并注明烧录地址、版本、Target、硬件范围、编译工具链、源码提交及 SHA256。Mender 服务器只封装/分发 App 升级容器，不直接把 factory 镜像作为 UART 载荷。

6. **与上位机配合和成功判定**

   - 工程用户进入设置更新页查询任务并手动开始。下位机包在写入前还需要二次确认。
   - 上位机获取治疗独占锁并暂停治疗程序，释放串口给独立升级任务；退出写入流程后释放串口并恢复治疗程序。
   - END ACK 代表镜像写入、校验及元数据提交成功。RUN 后 5 s 内，目标 App 版本和心跳均确认，才标记升级成功。
   - END 已成功而启动未确认时，报告“镜像已写入，尚未确认应用启动”，不能自动再擦除或显示成功。
   - Boot 无有效 App 时等待完整升级；App 有效、普通上电时保留约 500 ms 的 HELLO 恢复窗口。首次 SWD 安装完成前，原固件不能仅靠上位机更新软件自动获得 Bootloader。

7. **请工程师完成并反馈的验收结果**

   - 首次 SWD 烧录和偏移 App 的正常启动、全部中断、FreeRTOS 任务/定时器。
   - 普通版本查询/心跳、USART3 实际接线与串口独占。
   - 正常升级、重启后的目标版本/心跳确认。
   - 运行/暂停拒绝升级；进入 Boot 前的泵、加热和阀门安全输出实测。
   - 丢 ACK、断线、坏 CRC、错误 offset、重复 seq、错误硬件/型号包的处理。
   - 擦除、DATA、END 和元数据提交期间真实掉电后的 Boot 恢复与完整重刷。
   - Boot 不被 UART 升级覆盖；保留 SWD 救援。

请交回：修改后的源码/提交号、接口差异、可复现构建命令、本人编译的 App 升级容器和首次烧录镜像、版本与校验值、实机验证记录。收到后由上位机侧核对协议和容器，再封装为 Mender 更新包。
