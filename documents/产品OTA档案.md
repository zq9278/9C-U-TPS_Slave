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
- TPS签名信任锚：用户指定使用TPS产品密钥签名，路径尚待提供；必须配置签名私钥与设备原有公钥，不提供未签名发布。禁止复用SND-100密钥。配置仅保存在本机Electron用户数据publisher.json中。
- 上传位置：`POST /api/management/v1/deployments/artifacts` multipart artifact。上传后核对列表中的name/device_type/module/负载SHA256和大小。
- 本次任务范围：实现桌面上传按钮、版本递增、签名与上传核对，不创建Deployment，不运行设备安装。首次GUI功能测试不真实POST；取得真实TPS信任锚前不能把测试密钥用于产品发布。
- 发布记录：Windows文档/TPS100-Published/<name>，保留.mender和upload.json。软件包不包含用户凭据或私钥。
- ARM Linux安装器/签名策略/健康检查内部源码：本机未找到，参考服务器现有模块及用户OTA需求；实机安装和回滚未验证。
- 验证：主机测试覆盖命名、坏CRC、签名公钥匹配、并发版本冲突、POST响应丢失后哈希核对、禁止部署。新外层包已被原厂mender-artifact工具解析；不是实机安装验证。
