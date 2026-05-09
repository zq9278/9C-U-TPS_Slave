#ifndef APP_SYSTEM_APP_TASKS_H
#define APP_SYSTEM_APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建应用层全部 FreeRTOS 任务。
 *
 * 当前会创建：
 * - AppTask：业务配置与状态机入口
 * - CommRxTask / CommTxTask：串口协议收发
 * - SensorTask：压力/温度采样
 * - ControlTask：压力波形与加热闭环
 * - SafetyTask：基础安全监测
 */
void AppTasks_Init(void);

#ifdef __cplusplus
}
#endif

#endif
