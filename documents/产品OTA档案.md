# TPS MCU 发布端档案

- 产品：9C-U-TPS Slave，STM32G070RBT6；板硬件默认1，按实际.tpsfw范围核对。
- 源码：本仓库 codex/bootloader-uart-update；桌面工具 Tools/Flasher。
- 网关设备：9C-TPS-130，Mender UUID `d0b82a98-300b-41ae-b5d6-530f80fe87cd`；这是设备ID，不是Artifact ID。
- 设备类型：`eyemeter-thermal-aarch64`，已从服务器设备库存核对。
- 模块：`eyemeter-tps-mcu`；负载必须命名 `firmware.tpsfw`；清除provides `eyemeter.tps-mcu.*`。
- provides：`eyemeter.tps-mcu.target=0x9C01`；`eyemeter.tps-mcu.version`取包内真实major.minor.patch.build，不能取发布名伪装。
- 原服务器参考Artifact：`de5a68d5-a45a-4d71-a3f3-24e9fcc29f31` / `tps-mcu-1.0.0-20261111`，未签名；已读取头部和真实负载结构。
- 用户最新命名要求：首个新名称 `tps-mcu-1.0.2-20261111`，后续按服务器最大1.0.patch递增，日期固定20261111；不覆盖旧包。
- Boot/App分区、mailbox、UART/CRC/握手见uart_upgrade_handoff.md。MCU只支持单App重传恢复，不支持回滚；CRC不是签名。
- 管理服务器：五入口同一后端，依次aliyun/hk/oregon/oracle1/oracle2.sanitlook.dpdns.org:19443。正常TLS证书校验。当前只读查询aliyun TLS失败后使用hk成功。
- 发布凭据文件（只读路径）：`D:/Technology_stack/Web/mender/admin-credentials.json`。不进入源码、日志或软件包。JWT仅进程内存。
- 发布模式：用户最新指定未签名，默认沿用服务器现有TPS未签名格式，无需密钥。未来可在发布设置启用TPS签名；禁止复用SND-100密钥。配置仅保存在本机Electron用户数据publisher.json中。
- 上传位置：`POST /api/management/v1/deployments/artifacts` multipart artifact。上传后核对列表中的name/device_type/module/负载SHA256和大小。
- 本次任务范围：实现桌面上传按钮、版本递增与上传核对，不创建Deployment，不运行设备安装。发布默认未签名；测试密钥仅用于主机测试，不用于产品发布。
- 发布记录：Windows文档/TPS100-Published/<name>，保留.mender和upload.json。软件包不包含用户凭据或私钥。
- ARM Linux安装器/签名策略/健康检查内部源码：本机未找到，参考服务器现有模块及用户OTA需求；实机安装和回滚未验证。
- 验证：主机测试覆盖命名、坏CRC、签名公钥匹配、并发版本冲突、POST响应丢失后哈希核对、仅指定设备定向部署。新外层包已被原厂mender-artifact工具解析；不是实机安装验证。


## Studio 2.1.2 发布流程修正

依据下位机OTA改造需求第6节，工程用户在机器更新页查询任务并手动开始，MCU写入前二次确认。Studio上传按钮现在上传核对后创建定向部署，不能将“只上传Releases”当作机器可发现任务。

创建接口POST /api/management/v1/deployments/deployments，body含name、artifact_name、devices。devices固定为单个设备d0b82a98-300b-41ae-b5d6-530f80fe87cd。发布前核对设备类型、Artifact ID、模块、负载大小及SHA256。创建后读取详情，验证artifact_name及filter中唯一identity.id的$in目标，仅含指定设备。现有精确目标任务复用；创建结果不确定时查询核对，不重复POST。

本机upload.json记录任务ID和状态；“补建/核对更新任务”可选历史upload.json继续处理，无需重新上传。机器端手动安装策略来自已交接的ARM客户端，不由此工具绕过。
