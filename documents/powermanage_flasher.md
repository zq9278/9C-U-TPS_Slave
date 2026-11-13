# PowerManage独立烧录模块

来源：同级9C-U-TPS_PowerManage工程。IOC指定STM32F030F4Px，实际链接脚本STM32F030XX_FLASH.ld指定Flash16KiB/RAM4KiB，程序从0x08000000启动。保持PowerManage源码和现有.clangd修改不变，构建目录放在Slave/build/powermanage/Release，交付放在Slave/out/PowerManage。

```powershell
python scripts/build_powermanage.py
python scripts/build_release.py
python scripts/test.py
python scripts/package_flasher.py --skip-build --electron <electron/dist目录>
```

正常不带--skip-build打包会构建两块板。电源板输出power_app.elf、power_app.bin、power_flash.bin、release.json和SHA256SUMS.txt。power_flash.bin在真实程序末尾填充FF至完整16384字节。清单包含芯片、容量、地址、源提交/完整快照、构建标识和文件大小/哈希；源码在构建期间发生变化则拒绝。PowerManage尚无独立版本字段，此模块不杜撰版本号，也不声称读取了实机版本。

Tools/Flasher/powermanage.cjs负责独立镜像校验及SWD流程，main.cjs统一管理任务互斥。界面独立侧栏入口“电源板烧录”，操作使用公共探针序列号/速度、任务日志和明显成功弹窗。RTT连接期间禁用电源板烧录，主进程也拒绝冲突任务。窗口关闭继续按用户要求立即退出，无二次确认。

烧录固定J-Link目标STM32F030F4，loadfile/verifybin地址0x08000000，savebin完整0x4000字节。镜像校验检查文件哈希、16KiB填充和SP/复位向量范围。程序执行后验证回读大小及SHA256一致，才确认成功并显示复位已请求。仅校验不下发loadfile或erase，但会暂停CPU并在结束时复位。

分发包电源板镜像位于resources/payload/firmware-powermanage，Slave镜像继续在firmware目录。打包检查PowerManage当前源快照与交付清单一致，避免交付旧文件。电源板镜像不进入Slave Mender上传接口；不含Slave Boot/App分区。

软件验证包括真实交叉编译、错误芯片/镜像/SP拒绝、准确J-Link目标和完整回读、缺失回读失败、七页导航与RTT互斥。未连接实物烧录，电源、电池、充电和启动行为均需实板验证。
