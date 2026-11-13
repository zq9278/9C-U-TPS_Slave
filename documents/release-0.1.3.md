# App 0.1.3

启用耗材主动熔断：PRODUCT_FEATURE_CONSUMABLE_FUSE_BLOW_ENABLE=1。

USART3业务命令0x1007请求左眼熔断，0x1008请求右眼熔断；由原有控制任务服务输出熔断脉冲。保留现有脉冲时长和硬件映射。本版本不改变熔断异常拦截、OTP或其他安全配置。

版本由Config/version.cmake统一生成，业务版本回复、二进制身份和OTA包版本一致为0.1.3。沿用既有未签名Mender Artifact方式上传Releases，不创建部署，不自动操作设备；由用户手动更新。外层发布名称按服务器既有tps-mcu序号递增，独立于App版本。

软件编译和自动测试不能证明实物耗材已经熔断；实际脉冲、熔断检测与硬件结果仍需实板验证。
