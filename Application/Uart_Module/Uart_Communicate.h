/*
 * 模块名称: UART 协议 ID 与调度接口
 * 功能概述:
 *   - 统一帧 ID 规划：0x1000~0x10FF 为上位机 -> 下位机控制命令；
 *                     0x1100~0x11FF 为下位机 -> 上位机实时反馈。
 *   - 声明帧调度函数与若干数据解析辅助函数。
 *
 * 压力曲线约定（三段式，由下位机维护与解析）:
 *   1) 缓慢上升：维持时间 t1（秒），从 0 缓升到目标压力；
 *   2) 恒定压力：维持时间 t2（秒），保持目标压力；
 *   3) 脉动挤压：总维持时间 t3（秒），期间按 t_on/t_off 交替开关；
 *
 * 上位机简化交互建议:
 *   - 仅发送模式标志位（U8_MODE_SELECT） + 启动/停止命令（U8_START_TREATMENT/U8_STOP_TREATMENT）；
 *   - 具体曲线（t1/t2/t3、t_on/t_off、目标压力等）在下位机本地编辑/存储并生效；
 *   - 下方的 Mode1/Mode2 精细化写入 ID 保留用于调试/备选，不是生产必要项。
 */

#ifndef ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
#define ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H

#include <stdint.h>

/* 帧 ID 规划 */
typedef enum
{
    // 上位机 -> 下位机（设定/控制）
    U8_HEARTBEAT_REQ            = 0x1000, // 心跳请求（uint8 固定值）
    F32_PRESSURE_SET_KPA        = 0x1001, // 压力设定（float，单位 kPa，5~39）
    F32_LEFT_TEMP_SET_C         = 0x1002, // 左眼温度设定（float，°C）
    F32_RIGHT_TEMP_SET_C        = 0x1003, // 右眼温度设定（float，°C）
    U8_LEFT_PRESSURE_ENABLE     = 0x1004, // 左眼压力开关（uint8:0/1）
    U8_RIGHT_PRESSURE_ENABLE    = 0x1005, // 右眼压力开关（uint8:0/1）
    U8_LEFT_TEMP_ENABLE         = 0x1006, // 左眼加热开关（uint8:0/1）
    U8_RIGHT_TEMP_ENABLE        = 0x1007, // 右眼加热开关（uint8:0/1）
    U8_PUMP_POWER_VALUE         = 0x1008, // 气泵功率 PWM（uint8:0~255）
    U32_HOLD_TIME_MS            = 0x1009, // 保压时间（uint32，ms）
    U32_RELEASE_TIME_MS         = 0x100A, // 泄气时间（uint32，ms）
    U8_SQUEEZE_MODE             = 0x100B, // 挤压模式（uint8：0=正常/1=交替/2=同步）
    FRAME_ID_TEXT               = 0x10F0, // 调试文本帧（可选）

    // 新增：模式/曲线配置与控制
    U8_MODE_SELECT              = 0x10C0, // 当前治疗模式选择（1/2/3/4）
    U8_START_TREATMENT          = 0x10C1, // 启动治疗（当前模式）
    U8_STOP_TREATMENT           = 0x10C2, // 停止治疗（全部关闭）
    U8_SAVE_SETTINGS            = 0x10C3, // 保存当前设置到 EEPROM
    U8_USE_COMMON_TEMP          = 0x10C4, // 左右温度是否一致（uint8 0/1）


    // 下位机 -> 上位机（反馈/监测）
    U8_HEARTBEAT_ACK            = 0x1100, // 心跳应答
    F32_LEFT_PRESSURE_VALUE     = 0x1101, // 左眼压力反馈（float，kPa）
    F32_RIGHT_PRESSURE_VALUE    = 0x1102, // 右眼压力反馈（float，kPa）
    F32_LEFT_TEMP_VALUE         = 0x1103, // 左眼温度反馈（float，°C）
    F32_RIGHT_TEMP_VALUE        = 0x1104, // 右眼温度反馈（float，°C）
    U8_SYSTEM_STATE             = 0x1105, // 系统运行状态（可扩展）
    U8_ALARM_STATE              = 0x1106, // 设备告警状态（可扩展）
    U8_LEFT_HEATER_PRESENT      = 0x1107, // 左加热模块存在（0/1）
    U8_RIGHT_HEATER_PRESENT     = 0x1108, // 右加热模块存在（0/1）
    U8_LEFT_HEATER_FUSE         = 0x1109, // 左加热模块熔断（0/1）
    U8_RIGHT_HEATER_FUSE        = 0x110A, // 右加热模块熔断（0/1）
} FrameId_t;

/* 数据解析辅助接口 */
void handle_config_data(const uint8_t* data_ptr, uint16_t data_len);
uint8_t handle_uint8_t_data(const uint8_t *data_ptr, uint16_t data_len);
float handle_float_data(const uint8_t *data_ptr, uint16_t data_len);

/* 帧调度入口 */
void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len);

#endif // ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
