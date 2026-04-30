#include "comm_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "uart_driver.h"

/*
 * 通信任务说明
 *
 * 这个文件只负责“业务串口任务”的调度，不直接解析具体业务命令。
 *
 * 当前分层是：
 * 1. uart_driver.cpp
 *    负责 USART DMA/IDLE 接收、ISR 回调、协议帧发送。
 *
 * 2. comm_task.cpp
 *    负责在 FreeRTOS 任务上下文中取 RX 队列、取 TX 队列。
 *    RX 数据交给 port.parser()，TX 数据交给 HostFrameSender。
 *
 * 3. Uart_Communicate.cpp
 *    负责把具体 frame_id 转换成 app_event_t，再发给 TreatmentApp。
 *
 * 这样拆开以后，串口驱动、任务调度、协议业务不会混在一起。
 */

/* 业务串口：实际绑定 huart3，用于和 RK3576 / Qt 上位机通信。 */
extern UartPort_t rk3576_uart_port;
/* 调试串口：实际绑定 huart1，用于日志和调试输出。 */
extern UartPort_t debug_uart_port;

/*
 * HostFrameSender
 *
 * 作用：
 * 把系统内部统一的 tx_frame_t 转换成 uart_driver 支持的 DataType_t，
 * 然后调用 UartPort_t::sender() 发送协议帧。
 *
 * 注意：
 * - 这里不关心 frame_id 的业务含义。
 * - 这里只做“类型转换 + 发送”。
 * - 真正的打包、CRC、DMA 发送在 uart_driver.cpp 里完成。
 */
class HostFrameSender {
public:
    /* 绑定一个业务串口对象，后续所有发送都通过这个 port 输出。 */
    explicit HostFrameSender(UartPort_t &port) : port_(port) {}

    /*
     * 发送一帧上位机数据。
     *
     * tx_frame_t 是项目内部的通用发送结构：
     * - tx.frame_id：上位机协议帧 ID
     * - tx.type：数据类型
     * - tx.v：实际数据
     *
     * UartPort_t::sender() 使用的是 DataType_t，所以这里需要做一次映射。
     */
    void send(const tx_frame_t &tx) const
    {
        switch (tx.type) {
            case TX_DATA_FLOAT:
                /* float 数据，例如压力、温度。 */
                port_.sender(DATA_FLOAT, tx.frame_id, &tx.v.f32);
                break;
            case TX_DATA_UINT8:
                /* uint8_t 数据，例如状态、使能、停止通知。 */
                port_.sender(DATA_UINT8_T, tx.frame_id, &tx.v.u8);
                break;
            case TX_DATA_U16:
                /* uint16_t 数据，例如时间、计数类参数。 */
                port_.sender(DATA_UINT16_T, tx.frame_id, &tx.v.u16);
                break;
            case TX_DATA_U32:
                /* uint32_t 数据，例如累计时间、版本号等。 */
                port_.sender(DATA_UINT32_T, tx.frame_id, &tx.v.u32);
                break;
            case TX_DATA_TEXT:
            default:
                /* 文本数据，或者未知类型时按文本兜底发送。 */
                port_.sender(DATA_TYPE_TEXT, tx.frame_id, tx.v.text);
                break;
        }
    }

private:
    /* 引用外部 UartPort_t，不拥有它的生命周期。 */
    UartPort_t &port_;
};

/*
 * CommTaskRunner
 *
 * 作用：
 * 把原来直接写在 CommTask() 里的循环逻辑封装成 C++ 对象。
 *
 * 为什么要这样做：
 * - FreeRTOS 仍然需要 C 风格任务入口 CommTask(void *)。
 * - 任务内部逻辑用类组织后，后续扩展更清楚：
 *   例如增加调试协议、增加多串口、增加 TX 限流策略，都可以加成员函数。
 */
class CommTaskRunner {
public:
    /*
     * hostPort：业务串口，当前是 rk3576_uart_port / USART3。
     * debugPort：调试串口，当前是 debug_uart_port / USART1。
     */
    CommTaskRunner(UartPort_t &hostPort, UartPort_t &debugPort): hostPort_(hostPort), debugPort_(debugPort), sender_(hostPort)
    {
    }

    /*
     * 初始化两个串口端口。
     *
     * rk3576_uart_port_Init():
     * - 创建业务串口 RX/TX 队列
     * - 创建 TX 完成信号量
     * - 绑定 parser/sender/crc
     * - 启动 DMA/IDLE 接收
     *
     * debug_uart_port_Init():
     * - 创建日志 TX 队列
     * - 创建调试串口 RX 队列
     * - 启动 DMA/IDLE 接收
     */
    void init()
    {
        rk3576_uart_port_Init(&hostPort_);
        debug_uart_port_Init(&debugPort_);
    }

    /*
     * 通信任务主循环。
     *
     * 每一轮执行顺序：
     * 1. 先清空业务串口 RX 队列。
     * 2. 再发送少量 TX 队列数据。
     * 3. 延时 2ms 让出 CPU。
     *
     * RX 放在 TX 前面是故意的：
     * 如果上位机一边收遥测一边发停止命令，RX 优先可以让停止命令更快进入系统。
     */
    void run()
    {
        init();
        for (;;) {
            drainHostRx();
            flushHostTx();
            flushDebugTx();
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

private:
    /*
     * 处理业务串口接收数据。
     *
     * uart_driver.cpp 的 RX 回调会把 DMA 收到的一段数据复制成
     * UartRxMessage_t，并放进 hostPort_.rx_queue。
     *
     * 这里使用 while 一次性把队列里已有的数据全部取完：
     * - 避免 RX 数据积压。
     * - 避免上位机命令被遥测 TX 延迟。
     *
     * 取出的字节流交给 hostPort_.parser()。
     * 当前 parser 是 parse_rk3576_uart_port_stream()，
     * 它会检查帧头、长度、CRC、帧尾，然后调用 UartFrame_Dispatch()。
     */
    void drainHostRx()
    {
        UartRxMessage_t rxMsg = {};
        while (xQueueReceive(hostPort_.rx_queue, &rxMsg, 0) == pdPASS) {
            hostPort_.parser(rxMsg.data, rxMsg.length);
        }
    }

    /*
     * 处理发送到上位机的数据。
     *
     * gTxQueue 是系统统一的上位机发送队列：
     * - ControlTask 会往里面放压力、温度、阶段信息。
     * - TreatmentApp 会往里面放治疗结束等业务通知。
     * - 其他模块也可以放需要发给上位机的数据。
     *
     * 这里设置 budget = 4，表示每一轮最多发送 4 帧。
     *
     * 为什么要限制发送数量：
     * 如果遥测发送很密集，TX 队列可能一直有数据。
     * 不限制的话，CommTask 可能长时间卡在发送路径，
     * 导致 RX 命令处理不及时，典型现象就是停止命令延迟或无效。
     */
    void flushHostTx()
    {
        tx_frame_t tx = {};
        uint8_t budget = 4;
        while ((budget > 0U) && (xQueueReceive(gTxQueue, &tx, 0) == pdPASS)) {
            --budget;
            sender_.send(tx);
        }
    }

    /*
     * 处理调试串口发送队列。
     *
     * PID 曲线数据当前在 ControlTask 中通过 LOG_Raw() 放入
     * debug_uart_port.tx_queue。普通日志 LOG_I/LOG_W/LOG_E 也走同一个队列。
     *
     * 之前这个队列由 DebugLogTask 发送，现在统一放到 CommTaskRunner：
     * - debugPort_ 不再只是初始化用，而是真正负责调试串口 TX。
     * - 调试串口发送路径和业务串口发送路径都集中在通信任务里。
     * - DebugLogTask 只保留调试串口 RX 命令解析，避免两个任务抢同一个 TX 队列。
     *
     * 这里也设置 budget，避免调试日志/PID 曲线过多时长时间占用 CommTask。
     */
    void flushDebugTx()
    {
        LogMessage_t logMsg = {};
        uint8_t budget = 4;

        while ((budget > 0U) && (xQueueReceive(debugPort_.tx_queue, &logMsg, 0) == pdPASS)) {
            --budget;
            HAL_UART_Transmit(debugPort_.huart,
                              (uint8_t *)logMsg.buf,
                              (uint16_t)logMsg.len,
                              100);
        }
    }

    /* 业务串口引用，不拥有对象生命周期。 */
    UartPort_t &hostPort_;
    /* 调试串口引用，不拥有对象生命周期。当前本任务只负责初始化它。 */
    UartPort_t &debugPort_;
    /* 业务串口发送适配器。 */
    HostFrameSender sender_;
};

/*
 * FreeRTOS 任务入口。
 *
 * 必须使用 extern "C"：
 * - AppMain.c 是 C 文件，xTaskCreate() 需要能链接到 C 符号名 CommTask。
 * - 如果不加 extern "C"，C++ 会改名，C 文件可能链接不到。
 *
 * runner 是任务栈上的对象，run() 不会返回。
 */
extern "C" void CommTask(void *argument)
{
    (void)argument;
    CommTaskRunner runner(rk3576_uart_port, debug_uart_port);
    runner.run();
}
