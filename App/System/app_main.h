#ifndef APP_SYSTEM_APP_MAIN_H
#define APP_SYSTEM_APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化应用层运行环境。
 *
 * 该接口由系统启动流程调用，负责完成：
 * 1. 全局共享数据清零；
 * 2. FreeRTOS 关键队列创建；
 * 3. 板级用户驱动初始化；
 * 4. 通信模块初始化；
 * 5. 应用任务创建。
 */
void AppMain_FreeRTOS_Init(void);

#ifdef __cplusplus
}
#endif

#endif
