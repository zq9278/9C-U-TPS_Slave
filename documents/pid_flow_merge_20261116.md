# 远程PID控制流程选择性合入

来源：origin/master的6f38a72、0fbd240；按0fbd240的最终压力控制实现合入本地codex/bootloader-uart-update，不整体合并master。

- 升压、保压、脉冲加压统一由PID输出驱动泵，取消升压/保压固定额定PWM旁路。
- 泵PID原始输出范围0..25，正输出映射到10..100千分比PWM（1%..10%），零/负值及非有限输出停泵。
- 60秒周期默认升压10秒、保压20秒、脉冲30秒；每周期最后2秒固定放气，从脉冲尾部截取，不侵占下一周期升压。即使阶段配置异常挤满周期，末尾仍优先放气。
- 保留单双眼压力配置选择、双眼取较大值反馈、阶段切换/跨周期积分复位。默认增益采用远程最终值。
- 保留本地RTT八组调参、事务回执、Boot/App升级及Studio功能；ProfileGains包装接入远程经过范围检查的读写接口。

明确不纳入：master的主动熔断开启、超压阈值/确认时长改动、RK3576模拟压力曲线、独立J-Link内存邮箱及其GUI。主动熔断继续为0，超压保护继续使用本地原值；传感器、业务上报和RTT继续真实采样链路。

App版本提升为0.1.5，避免与旧控制流程的0.1.4混淆。本次仅本地分支提交和构建，不上传服务器、不创建部署、不改变既有生产ZIP。

验证：真实Debug ARM ELF在Unicorn中运行控制函数，截获PWM/阀门硬件调用；覆盖三阶段PID影响输出、零输出/最小输出/饱和、单侧及双侧反馈、脉冲放气、周期末2秒放气和下一轮重启、异常时序放气优先、RTT参数组默认值及读写。回归继续覆盖Boot/OTA/RTT和Studio。仿真不证明真实气路、压力动态或治疗安全。

```powershell
python -m pip install -r Tools/FirmwarePack/requirements-test.txt
python scripts/build_release.py
python scripts/test.py
```

新增压力流程测试基于远程Tools/JLinkPidTuner/tests/test_firmware.py的ARM执行夹具，去除本次未引入的模拟显示与邮箱用例，使用当前分支的Debug交付ELF。
