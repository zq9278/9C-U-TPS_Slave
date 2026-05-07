#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    /**
     * @brief 默认 CAN 发送超时时间，单位毫秒。
     */
    BSP_CAN_DEFAULT_TIMEOUT_MS = 100U
};

/**
 * @brief 标准 CAN 数据帧抽象。
 *
 * 这里只描述“协议无关”的一帧 CAN 内容，不直接暴露 HAL 的 Header 结构体，
 * 方便上层以后切换到底层不同实现时保持接口稳定。
 */
typedef struct
{
    uint32_t id;
    uint8_t is_extended_id;
    uint8_t is_remote_frame;
    uint8_t dlc;
    uint8_t data[8];
} BspCanFrame;

/**
 * @brief CAN 底层启动回调。
 *
 * 典型职责包括：
 * 1. 配过滤器；
 * 2. 启动 CAN；
 * 3. 开接收中断或通知。
 */
typedef uint8_t (*BspCanStartCallback)(void *context);

/**
 * @brief CAN 底层发送回调。
 */
typedef uint8_t (*BspCanSendCallback)(void *context,
                                      const BspCanFrame *frame,
                                      uint32_t timeout_ms);

/**
 * @brief CAN 底层错误恢复回调。
 *
 * 有些项目会在仲裁丢失、总线错误、ACK 错误之后尝试清标志或重新启动，
 * 这里保留一个统一入口。
 */
typedef uint8_t (*BspCanRecoverCallback)(void *context);

/**
 * @brief CAN 底层操作集合。
 */
typedef struct
{
    BspCanStartCallback start;
    BspCanSendCallback send;
    BspCanRecoverCallback recover;
} BspCanOps;

/**
 * @brief 上层感兴趣的 CAN 事件回调集合。
 */
typedef struct
{
    void (*on_rx_frame)(void *context, const BspCanFrame *frame);
    void (*on_tx_complete)(void *context);
    void (*on_error)(void *context, uint32_t error_code);
    void *context;
} BspCanCallbacks;

/**
 * @brief CAN 总线初始化配置。
 */
typedef struct
{
    BspCanOps ops;
    void *low_level_context;
    uint32_t default_timeout_ms;
    uint8_t auto_recover_on_error;
} BspCanConfig;

/**
 * @brief CAN 总线对象。
 *
 * 一路 CAN 控制器对应一个对象。这样上层协议层只需要拿 `BspCan *`，
 * 不用知道底层到底是 CAN1、CAN2 还是其他兼容实现。
 */
typedef struct
{
    BspCanConfig config;
    BspCanCallbacks callbacks;
    uint8_t started;
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t error_count;
} BspCan;

void BspCan_Init(BspCan *bus, const BspCanConfig *config);
void BspCan_SetCallbacks(BspCan *bus, const BspCanCallbacks *callbacks);
uint8_t BspCan_Start(BspCan *bus);
uint8_t BspCan_Send(BspCan *bus, const BspCanFrame *frame, uint32_t timeout_ms);
void BspCan_OnRxFrameFromIsr(BspCan *bus, const BspCanFrame *frame);
void BspCan_OnTxCompleteFromIsr(BspCan *bus);
void BspCan_OnErrorFromIsr(BspCan *bus, uint32_t error_code);

#ifdef __cplusplus
}
#endif

#endif
