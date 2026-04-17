#ifndef VALVE_CONTROL_H
#define VALVE_CONTROL_H

#include <stdint.h>

/*
 * 电磁阀控制抽象层：
 * - VALVE_LEFT / VALVE_RIGHT：用于选择当前治疗通道
 * - VALVE：用于按波形打开/关闭挤压
 *
 * 上层业务不再直接操作底层 GPIO 宏，只通过这里提供的函数控制阀门。
 */

/*
 * 根据左右治疗使能状态，配置当前治疗通道：
 * - 左眼单眼：VALVE_LEFT(0), VALVE_RIGHT(1)
 * - 右眼单眼：VALVE_LEFT(1), VALVE_RIGHT(0)
 * - 双眼：VALVE_LEFT(0), VALVE_RIGHT(0)
 * - 全禁用：VALVE_LEFT(0), VALVE_RIGHT(0)
 */
void ValveControl_ApplyTreatmentRoute(uint8_t enable_left, uint8_t enable_right);

/*
 * 控制波形阀开关：
 * - 1 = 打开当前波形挤压
 * - 0 = 关闭当前波形挤压
 */
void ValveControl_SetWave(uint8_t enabled);

/*
 * 恢复到空闲安全态：
 * - 治疗通道回到默认态
 * - 波形阀关闭
 */
void ValveControl_SetIdle(void);

#endif /* VALVE_CONTROL_H */
