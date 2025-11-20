/*
 * 模块名称: UART 协议 ID 与调度接口
 * 功能概述:
 *   - 统一帧 ID 规划：0x1000~0x10FF 为上位机 -> 下位机控制命令；
 *                     0x1100~0x11FF 为下位机 -> 上位机实时反馈。
 *   - 声明帧调度函数与若干数据解析辅助函数。
 *   - 曲线阶段由下位机内部维护（t1/t2/t3 + 脉动 on/off），上位机仅选择模式、设置压力/温度并启动/停止。
 */

#ifndef ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
#define ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H

#include <stdint.h>

/* 帧 ID 规划 */
typedef enum
{
    // 上位机 -> 下位机（设定/控制）
    U8_HEARTBEAT_REQ         = 0x1000, // 心跳请求
    F32_PRESSURE_SET_KPA     = 0x1001, // 压力设定 (float kPa)
    F32_LEFT_TEMP_SET_C      = 0x1002, // 温度设定 (float °C，左右共用)
    U8_MODE_SELECT           = 0x10C0, // 曲线模式选择 (1/2/3/4)
    U8_START_TREATMENT       = 0x10C1, // 启动治疗
    U8_STOP_TREATMENT        = 0x10C2, // 停止治疗
    U8_SAVE_SETTINGS         = 0x10C3, // 保存设置

    // 下位机 -> 上位机（反馈/监测）
    U8_HEARTBEAT_ACK         = 0x1100, // 心跳应答
    F32_LEFT_PRESSURE_VALUE  = 0x1101, // 左眼压力反馈
    F32_RIGHT_PRESSURE_VALUE = 0x1102, // 右眼压力反馈
    F32_LEFT_TEMP_VALUE      = 0x1103, // 左眼温度反馈
    F32_RIGHT_TEMP_VALUE     = 0x1104, // 右眼温度反馈
    U8_SYSTEM_STATE          = 0x1105, // 系统运行状态
    U8_ALARM_STATE           = 0x1106, // 告警状态
    U8_LEFT_HEATER_PRESENT   = 0x1107, // 左加热模块存在
    U8_RIGHT_HEATER_PRESENT  = 0x1108, // 右加热模块存在
    U8_LEFT_HEATER_FUSE      = 0x1109, // 左加热模块熔断
    U8_RIGHT_HEATER_FUSE     = 0x110A, // 右加热模块熔断
} FrameId_t;

/* 数据解析辅助接口 */
void handle_config_data(const uint8_t* data_ptr, uint16_t data_len);
uint8_t handle_uint8_t_data(const uint8_t *data_ptr, uint16_t data_len);
float handle_float_data(const uint8_t *data_ptr, uint16_t data_len);

/* 帧调度入口 */
void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len);

#endif // ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H

