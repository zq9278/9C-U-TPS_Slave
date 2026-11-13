# App 0.1.4 / Studio 2.2.3

按用户要求关闭耗材主动熔断，PRODUCT_FEATURE_CONSUMABLE_FUSE_BLOW_ENABLE=0。收到左右熔断命令时不产生熔断GPIO脉冲；状态检测、上报及其他保护配置保持不变。

App统一版本升为0.1.4。Studio 2.2.3内置该版本用于热脉动控制板PCBA烧录，电源控制板PCBA沿用当前PowerManage源码构建。保留失败/成功弹窗和完整Flash回读核对。

此版本作为本地生产包交付；本次不自动上传服务器或创建更新任务。尚未进行实物烧录与耗材验证。
