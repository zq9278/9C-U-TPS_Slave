# J-Link RTT 调试约定

日志从UART1迁移到SEGGER RTT，USART3业务/OTA帧完全不变。UART1原有PID输入兼容入口保留，但不再输出调试日志。RTT不接收治疗启动、复位、熔断、内存写入或PID修改命令。

- 默认TPS_RTT=ON；构建App、Boot使用相同SEGGER RTT源码和非阻塞skip模式。无读取端时丢弃日志，不阻塞控制任务。
- 上行0：UTF-8日志与只读查询回复；下行0：ASCII命令，LF/CR结尾、最长63字符。
- 上行1：原PID采样二进制帧，与日志分开，避免文本解码污染。
- help：列出只读命令；info：App版本、Target、HW、App地址、最低Boot版本；status：tick、治疗标志、传感器快照、剩余RTOS堆及日志丢弃次数。
- 只读命令在CommRxTask执行；stack增加至448words。传感器快照是诊断用途，不保证多个字段采样同时性。
- 工具采用后台SWD RTT环形缓冲读写，只修改RTT主机拥有的读指针和下行命令。连接选择通用Cortex-M0核心，避免特定器件连接脚本初始化工作RAM；固定1000kHz。
- 烧录使用确切STM32G070RB器件。烧录与RTT互斥；连接RTT不自动复位/暂停。已经halt的目标报错，不自动执行go。
- 状态指针超出0x20000000..0x20008FE0或控制块失效时停止连接，不能把mailbox当作RTT缓冲；复位后重新连接。

官方参考：[RTT](https://kb.segger.com/RTT)、[J-Link Commander](https://kb.segger.com/J-Link_Commander)。第三方RTT源码保留原版权和许可头。
