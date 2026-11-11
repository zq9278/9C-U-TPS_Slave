# TPS130 Studio 2.0：调试平台与设备适配

## 交付对象

本次重构的是原Windows Electron烧录调试工具，产品名改为TPS130 Studio；不修改ARM Linux治疗界面，也不改变MCU业务协议。保留原SWD烧录、回读校验、RTT、Mender发布能力与立即关闭行为。

界面采用固定侧栏、公共连接头、设备工作台、公共任务日志和成功弹窗。六个页面：设备概览、调试工作台、固件与发布、检查与诊断、日志与记录、连接与设置。

## 模块边界

| 层 | 路径 | 职责 |
|---|---|---|
| 公共平台模型 | Tools/Flasher/platform/core.js | 流式行解码、受限历史、数据过期、断线/重启断点、记录验证、CSV |
| 公共曲线控件 | Tools/Flasher/platform/chart.js | 真实数据绘制、颜色与显示开关、时间窗口、缺失数据断线 |
| TPS130能力定义 | Tools/Flasher/devices/tps130/manifest.json | 型号、测点、单位、颜色、参数、能力开关和限制说明 |
| TPS130协议适配 | Tools/Flasher/devices/tps130/adapter.js | 将现有RTT info/status回复映射成identity/sample事件 |
| 首个设备工作台 | Tools/Flasher/index.html、renderer.js | TPS130温压、参数区、实时/回放联动；公共导航和任务交互 |
| 主进程服务 | main.cjs、backend.cjs、publisher.cjs | 资源互斥、烧录、文件校验、发布、文件对话框和导出 |
| RTT连接 | rtt.cjs、rtt_bridge.py | 独占J-Link后台RTT，白名单只读命令，不直接操作治疗输出 |
| 品牌资源 | assets/tps130.svg、png、ico | TPS130独立眼形+脉冲图标；窗口、任务栏和EXE图标 |

这是首个设备适配版本，不宣称任意设备已即插即用。新增设备时先定义自己的manifest、协议适配器与工作台布局，再接入对应板卡升级/发布服务；TPS130的Flash、Mender类型、泵/加热语义不应作为其他设备默认值。当前打包目标只有TPS130，其他设备需各自配置名称、AppUserModelID、EXE和图标，不能仅复制TPS130参数。

后续增加设备注册器、不同工作台模板和可选串口/TCP传输时，应保持页面通过服务接口调用，不允许专用页面直接使用Node/读写任意硬件地址。当前Electron renderer保持contextIsolation、sandbox且不开放Node。

## 当前真实数据能力

现有Release固件0.1.1的RTT协议开放help/info/status。工作台连接成功后约每200ms请求status，自动解析左右温度、左右压力、设备Tick、治疗运行标志、RTOS剩余堆、日志丢弃计数；info提供App、HW、Target、App基址、最低Boot。

这是5Hz请求的诊断状态快照，不等于原始传感器采样；实际更新率取决于RTT链路。断线、暂停采集或超过1.5s没有状态回复时，值标记为历史/过期，不能继续显示“实时正常”。数据缺失超过1.5s、断线后重连、设备Tick回退时曲线不跨段连接。

参数区域当前提供设备身份只读查询。PID/目标值/治疗时序写入没有带应答的已确认诊断接口，因此明确禁用，不构造假参数或发送未验证控制指令。添加参数写入需要后续固件支持参数ID、范围、事务号、应用应答、停机/权限约束和回读。

## 录制、导出与回放

- 实时历史最多18000点；显式录制最多50000点，达到上限停止录制并提示保存。
- JSON记录schema=1，标明device=tps130、设备身份、时间和sample数组。导入检查产品、数量、时间顺序与有限数值；文件不超过20MB。
- CSV列包含主机ISO时间、设备Tick、治疗标志和带单位的测点。记录仅用于诊断，传感器快照不保证各字段同一采样时刻。
- 回放提供时间滑块与按记录时间播放；回放时暂停实时轮询，禁止设备查询/新连接操作，返回实时时恢复原有连接的采集。
- 暂停查看曲线只冻结显示，不暂停设备，也不停止后台采集；暂停采集仅停止只读状态请求。
- 当前工作台只读，不提供治疗启动、急停或执行器测试按钮，避免将传输状态当作安全保护。

## 图标与软件身份

产品名TPS130 Studio、EXE名TPS130-Studio.exe、Windows AppUserModelID=Sanitlook.TPS130.Studio。图标由代码原生矢量几何生成，非沿用Electron默认图标；scripts/make_icon.py生成多分辨率ICO及PNG，SVG用于界面。打包用rcedit写入EXE资源和产品信息，窗口也显式指定ICO。

## 构建与验收

```powershell
npm ci --prefix Tools/Flasher
python scripts/build_release.py
python scripts/test.py
python scripts/package_flasher.py --skip-build --electron <electron/dist目录>
```

默认打包输出out/distribution/TPS130-Studio-2.0.0-时间戳，LATEST.txt指向最新目录。需要本机安装J-Link；分发包会携带运行时及原厂许可，不包含管理凭据、私钥或用户录制数据。

主机测试包括：设备描述校验、真实协议格式的分块解析、无效数值拒绝、数据过期与断线、记录格式/设备匹配、CSV、六页导航、工作台与离线回放联动、成功弹窗，以及原烧录/发布/固件恢复测试。GUI冒烟注入的数据是明确的测试fixture，测试后清空，发布程序启动无模拟曲线。

尚未实板验证J-Link采集稳定性、状态更新率、长期录制与实际设备安全状态；仍保留release.json中的hardware_validation=NOT_RUN。本次不声称PID调参、传感器校准、本机串口OTA或其他设备适配已完成。
