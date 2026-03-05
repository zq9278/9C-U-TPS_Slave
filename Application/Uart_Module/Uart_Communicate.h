/*
 * UART protocol frame IDs and dispatch interfaces.
 */

#ifndef ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
#define ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H

#include <stdint.h>

/* Frame ID plan */
typedef enum
{
    // Host -> Device
    U8_HEARTBEAT_REQ               = 0x1000, // Heartbeat request
    F32_PRESSURE_SET_KPA           = 0x1001, // Pressure set (float kPa)
    F32_LEFT_TEMP_SET_C            = 0x1002, // Temperature set (float C)
    U8_LEFT_EYE_ENABLE             = 0x1004, // Left eye enable
    U8_RIGHT_EYE_ENABLE            = 0x1005, // Right eye enable
    U16_TREAT_TIME_MIN             = 0x1006, // Treatment time (minutes)
    U8_LEFT_HEATER_FUSE_BLOW_CMD   = 0x1007, // Force blow left eye shield fuse
    U8_RIGHT_HEATER_FUSE_BLOW_CMD  = 0x1008, // Force blow right eye shield fuse
    U8_MODE_SELECT                 = 0x10C0, // Curve mode select (1..4)
    U8_START_TREATMENT             = 0x10C1, // Start treatment
    U8_STOP_TREATMENT              = 0x10C2, // Stop treatment
    U8_SAVE_SETTINGS               = 0x10C3, // Save current parameters

    // Device -> Host
    U8_HEARTBEAT_ACK               = 0x1100, // Heartbeat ack
    F32_LEFT_PRESSURE_VALUE        = 0x1101, // Left pressure feedback
    F32_RIGHT_PRESSURE_VALUE       = 0x1102, // Right pressure feedback
    F32_LEFT_TEMP_VALUE            = 0x1103, // Left temperature feedback
    F32_RIGHT_TEMP_VALUE           = 0x1104, // Right temperature feedback
    U8_LEFT_HEATER_PRESENT         = 0x1107, // Left shield present
    U8_RIGHT_HEATER_PRESENT        = 0x1108, // Right shield present
    U8_LEFT_HEATER_FUSE            = 0x1109, // Left shield fuse status
    U8_RIGHT_HEATER_FUSE           = 0x110A, // Right shield fuse status
    U8_MODE_CURVES                 = 0x110B, // Current mode stage (r/h/p)
} FrameId_t;

/* Parse helpers */
void handle_config_data(const uint8_t *data_ptr, uint16_t data_len);
uint8_t handle_uint8_t_data(const uint8_t *data_ptr, uint16_t data_len);
uint16_t handle_uint16_t_data(const uint8_t *data_ptr, uint16_t data_len);
float handle_float_data(const uint8_t *data_ptr, uint16_t data_len);

/* Frame dispatch */
void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len);

#endif // ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
