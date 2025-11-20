/*
 * 模块名称: UART 协议命令分发层（RK3576 业务口）
 *
 * 职责边界:
 *   - 输入: 来自 uart_driver.c 的“已验帧”数据: (frame_id, data_ptr, data_len)。
 *   - 处理: 将协议帧翻译为系统内部命令 app_cmd_t 并入队 gCmdQueue；必要时通过 gTxQueue 发送简要应答。
 *   - 输出: AppTask/ControlTask 从 gCmdQueue 取走命令，执行硬件/控制逻辑；CommTask 从 gTxQueue 取走帧并发送。
 *   - 不做: 任何硬件直接访问、长耗时/阻塞、存储读写等（保持分层清晰、便于测试与维护）。
 *
 * 与其它模块关系:
 *   RX: HAL UART DMA -> uart_driver.parse_rk3576_uart_port_stream() -> UartFrame_Dispatch() -> gCmdQueue
 *   TX: 任意任务 -> 组装 tx_frame_t -> gTxQueue -> CommTask -> send_rk3576_uart_port_frame() (DMA 发送)
 *
 * 协议帧格式(见 uart_driver.h):
 *   0xAA 0x55 | frame_id(LE,2) | data_type(1) | data_len(LE,2) |
 *   payload(data_len) | crc16_modbus(LE,2) | 0x0D 0x0A
 *   - 整数按小端；浮点为 IEEE754 float；长度上限由 FRAME_MAX_DATA_LEN 约束；
 *   - 本模块只做最小必要的边界检查与数据解释，CRC/边界校验在 driver 层已完成。
 *
 * 扩展说明:
 *   - 新增上位机->下位机 ID: 在 Uart_Communicate.h 中添加枚举，再在 UartFrame_Dispatch 的 switch 中实现映射；
 *   - 新增下位机->上位机反馈: 在相应任务中创建 tx_frame_t 并入队 gTxQueue（或仿照 tx_u8 增加便捷函数）；
 *   - 建议将复杂业务参数以“key 索引 + 值”的方式更新，便于按项覆盖与版本兼容。
 */
/*
 * UART 协议命令分发（UTF-8）
 * 仅负责：将解析出的帧ID+数据，转换为 AppTask 能理解的命令（app_cmd_t）并入队。
 * 不做：任何硬件直控/定时器/保存操作；心跳应答等下行通过 gTxQueue 统一发送。
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "Uart_Communicate.h"
#include "AppMain/system_app.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "LOG.h"
#include "AppMain/mode_curves.h"

// ---------------- 数据解码辅助 ----------------
void handle_config_data(const uint8_t* data_ptr, uint16_t data_len)
{
    (void)data_ptr; (void)data_len; // 预留
}

uint8_t handle_uint8_t_data(const uint8_t *data_ptr, uint16_t data_len)
{
    uint8_t v = 0; if (data_len >= 1) memcpy(&v, data_ptr, 1); return v;
}

float handle_float_data(const uint8_t *data_ptr, uint16_t data_len)
{
    float v = 0.0f; if (data_len >= sizeof(float)) memcpy(&v, data_ptr, sizeof(float)); return v;
}

static inline uint16_t read_u16_le(const uint8_t *p, uint16_t len)
{ uint16_t v=0; if (len>=2) memcpy(&v,p,2); return v; }
static inline uint32_t read_u32_le(const uint8_t *p, uint16_t len)
{ uint32_t v=0; if (len>=4) memcpy(&v,p,4); return v; }

// ---------------- 入队便捷函数 ----------------
static inline void tx_u8(uint16_t id, uint8_t v)
{
    tx_frame_t tx = {0}; tx.type = TX_DATA_UINT8; tx.frame_id = id; tx.v.u8 = v;
    (void)xQueueSend(gTxQueue, &tx, 0);
}

static inline void push_cmd_u8(app_cmd_id_t id, uint8_t v)
{ app_cmd_t c={0}; c.id=id; c.v.u8=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_f32(app_cmd_id_t id, float v)
{ app_cmd_t c={0}; c.id=id; c.v.f32=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_key_u8(app_cmd_id_t id, uint16_t key, uint8_t v)
{ app_cmd_t c={0}; c.id=id; c.key=key; c.v.u8=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_key_u16(app_cmd_id_t id, uint16_t key, uint16_t v)
{ app_cmd_t c={0}; c.id=id; c.key=key; c.v.u16=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_key_u32(app_cmd_id_t id, uint16_t key, uint32_t v)
{ app_cmd_t c={0}; c.id=id; c.key=key; c.v.u32=v; (void)xQueueSend(gCmdQueue,&c,0); }
static inline void push_cmd_key_f32(app_cmd_id_t id, uint16_t key, float v)
{ app_cmd_t c={0}; c.id=id; c.key=key; c.v.f32=v; (void)xQueueSend(gCmdQueue,&c,0); }

// ---------------- 协议帧分发 ----------------
/*
 * 函数名称: UartFrame_Dispatch
 * 功能描述:
 *   - 将上位机帧 ID 映射到内部 app_cmd_t；
 *   - 简单数值解码（u8/float/u16/u32），并入队到 gCmdQueue；
 *   - 心跳等轻量应答通过 gTxQueue 发送（见 tx_u8()）。
 * 入参:
 *   - frame_id : 帧 ID（见 Uart_Communicate.h 的 FrameId_t 规划）
 *   - data_ptr : 指向 payload 的指针
 *   - data_len : payload 长度
 * 设计约束:
 *   - 非阻塞、不可长时间停留；
 *   - 不直接访问硬件；
 *   - 未知 ID 仅记录日志，便于兼容增量。
 */
void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len)
{
    switch (frame_id)
    {
        // 基础通信
        case U8_HEARTBEAT_REQ:
            tx_u8(U8_HEARTBEAT_ACK, 1);
            break;

        // 参数设定
        case F32_PRESSURE_SET_KPA:
            push_cmd_f32(APP_CMD_SET_PRESSURE_KPA, handle_float_data(data_ptr, data_len));
            break;
        case U8_LEFT_PRESSURE_ENABLE:
            push_cmd_u8(APP_CMD_SET_LEFT_PRESSURE_ENABLE, handle_uint8_t_data(data_ptr, data_len));
            break;
        case U8_RIGHT_PRESSURE_ENABLE:
            push_cmd_u8(APP_CMD_SET_RIGHT_PRESSURE_ENABLE, handle_uint8_t_data(data_ptr, data_len));
            break;
        case F32_LEFT_TEMP_SET_C:
            push_cmd_f32(APP_CMD_SET_TEMP_LEFT, handle_float_data(data_ptr, data_len));
            break;
        case F32_RIGHT_TEMP_SET_C:
            push_cmd_f32(APP_CMD_SET_TEMP_RIGHT, handle_float_data(data_ptr, data_len));
            break;
        case U8_LEFT_TEMP_ENABLE:
            // 使用 APP_CMD_SET_MODE*_PARAM + key=100/101 作为运行期开关（不入设置）
            push_cmd_key_u8(APP_CMD_SET_MODE1_PARAM, 100, handle_uint8_t_data(data_ptr, data_len));
            break;
        case U8_RIGHT_TEMP_ENABLE:
            push_cmd_key_u8(APP_CMD_SET_MODE1_PARAM, 101, handle_uint8_t_data(data_ptr, data_len));
            break;
        case U8_PUMP_POWER_VALUE:
            // 直控泵功率与当前“闭环压力控制”冲突，保持记录
            LOG("U8_PUMP_POWER_VALUE=%u (ignored in PID mode)\r\n", handle_uint8_t_data(data_ptr, data_len));
            break;
        case U32_HOLD_TIME_MS:
            // 映射到当前模式的 t_hold_ms
            push_cmd_key_u32(APP_CMD_SET_MODE1_PARAM, 2, read_u32_le(data_ptr, data_len));
            push_cmd_key_u32(APP_CMD_SET_MODE2_PARAM, 2, read_u32_le(data_ptr, data_len));
            break;
        case U32_RELEASE_TIME_MS:
            // 现有控制未使用“独立泄气时间”，仅记录
            LOG("U32_RELEASE_TIME_MS=%lu (not used)\r\n", (unsigned long)read_u32_le(data_ptr, data_len));
            break;
        case U8_SQUEEZE_MODE:
            // 同时更新两个模式的挤压模式，便于上位机快速切换
            push_cmd_key_u8(APP_CMD_SET_MODE1_PARAM, 6, handle_uint8_t_data(data_ptr, data_len));
            push_cmd_key_u8(APP_CMD_SET_MODE2_PARAM, 6, handle_uint8_t_data(data_ptr, data_len));
            break;
        case U8_USE_COMMON_TEMP:
            push_cmd_u8(APP_CMD_SET_USE_COMMON_TEMP, handle_uint8_t_data(data_ptr, data_len));
            break;

        // 模式/流程控制
        case U8_MODE_SELECT:
            push_cmd_u8(APP_CMD_MODE_SELECT, handle_uint8_t_data(data_ptr, data_len));
            break;
        case U8_START_TREATMENT:
        {
            uint8_t en = handle_uint8_t_data(data_ptr, data_len);
            app_cmd_t c={0}; c.id = en ? APP_CMD_START : APP_CMD_STOP; (void)xQueueSend(gCmdQueue,&c,0);
            break;
        }
        case U8_STOP_TREATMENT:
        {
            app_cmd_t c={0}; c.id = APP_CMD_STOP; (void)xQueueSend(gCmdQueue,&c,0);
            break;
        }
        case U8_SAVE_SETTINGS:
        {
            app_cmd_t c={0}; c.id = APP_CMD_SAVE_PARAM; (void)xQueueSend(gCmdQueue,&c,0);
            break;
        }

    



        // 文本/未知
        case FRAME_ID_TEXT:
            LOG("TEXT len=%d\r\n", data_len);
            break;

        default:
            LOG("unknown frame_id 0x%04X len=%d\r\n", frame_id, data_len);
            break;
    }
}
