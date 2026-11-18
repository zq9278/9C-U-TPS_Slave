# Studio 2.3.0 / App 0.1.5 发布记录

- GitHub Release：[studio-v2.3.0-app-v0.1.5](https://github.com/zq9278/9C-U-TPS_Slave/releases/tag/studio-v2.3.0-app-v0.1.5)
- 完整生产包：TPS130-Studio-2.3.0-20261116-175146.zip
- 固件源码与构建来源：dd64f26（标签指向该构建提交；本记录提交位于其后）。
- 热脉动控制板 App：0.1.5，主动熔断关闭，升压/保压/脉冲加压均由PID输出控制泵速，周期末固定2秒放气。
- PowerManage：独立STM32F030F4电源控制板固件，构建标识及源快照见PowerManage-release.json。

附件包括完整ZIP及其校验文件、Slave Boot/App/OTA/factory/ELF及交付清单、电源控制板镜像/ELF及交付清单。PowerManage附件使用PowerManage-前缀区分所属板卡，不与Slave镜像混用。各附件字节数、SHA256和下载链接见同目录studio-2.3.0.json。

GitHub附件上传后核对服务器返回的SHA256与本地一致，再公开发布。完整ZIP必须解压并保留目录结构。热脉动控制板SWD使用tps_factory.bin，OTA封装使用tps_app.tpsfw；电源控制板使用PowerManage-power_flash.bin。

编译、ARM仿真、主机自动测试、GUI冒烟及包完整性检查已通过；尚未实板确认烧录与功能。GitHub发布不会创建Mender设备升级任务；此前已下发的Mender任务单独管理。
