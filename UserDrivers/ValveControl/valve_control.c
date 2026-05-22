#include "ValveControl/valve_control.h"

#include "main.h"

/*
 * 把底层 GPIO 操作也收口到 ValveControl 模块里。
 * apply.h 只保留函数声明，真正实现统一放在这里，避免阀门代码分散在多个目录。
 */
void VALVE_LEFT(uint8_t enabled)
{
    HAL_GPIO_WritePin(AirValve1_GPIO_Port,
                      AirValve1_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void VALVE_RIGHT(uint8_t enabled)
{
    HAL_GPIO_WritePin(AirValve2_GPIO_Port,
                      AirValve2_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void VALVE(uint8_t enabled)
{
    HAL_GPIO_WritePin(AirPump2_GPIO_Port,
                      AirPump2_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ValveControl_ApplyTreatmentRoute(uint8_t enable_left, uint8_t enable_right)
{
    if (enable_left && enable_right) {
        VALVE_LEFT(0);
        VALVE_RIGHT(0);
    } else if (enable_left) {
        VALVE_LEFT(1);
        VALVE_RIGHT(0);
    } else if (enable_right) {
        VALVE_LEFT(0);
        VALVE_RIGHT(1);
    } else {
        VALVE_LEFT(0);
        VALVE_RIGHT(0);
    }
}

void ValveControl_SetWave(uint8_t enabled)
{
    /* 波形阀统一走本文件里的底层函数实现。 */
    VALVE(enabled ? 1 : 0);
}

void ValveControl_SetIdle(void)
{
    /* 空闲态回到双眼默认路由，同时关闭波形阀。 */
    ValveControl_ApplyTreatmentRoute(0, 0);
    ValveControl_SetWave(0);
}
