#ifndef MODULES_LOG_MODULE_LOG_CONFIG_H
#define MODULES_LOG_MODULE_LOG_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* 全局日志总开关。 */
#define MODULE_LOG_GLOBAL_ENABLE                  1U

/* 各模块独立日志开关。 */
#define MODULE_LOG_APP_TASK_ENABLE                1U
#define MODULE_LOG_APP_CONTROLLER_ENABLE          0U
#define MODULE_LOG_ACTUATOR_ENABLE                0U

#define MODULE_LOG_SENSOR_TEMP_ENABLE             0U
#define MODULE_LOG_SENSOR_PRESS_ENABLE            0U

#define MODULE_LOG_EYE_SHIELD_ENABLE             1U

#define MODULE_LOG_DRIVER_ADS1248_ENABLE         0U
#define MODULE_LOG_DRIVER_PRESSURE_ENABLE        0U

#ifdef __cplusplus
}
#endif

#endif
