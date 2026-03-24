#ifndef _APPLY_H
#define _APPLY_H
#include <sys.h>	  



#define AirPump1(n)  (n?HAL_GPIO_WritePin(AirPump1_GPIO_Port,AirPump1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(AirPump1_GPIO_Port,AirPump1_Pin,GPIO_PIN_RESET)) 
#define AirPump2(n)  (n?HAL_GPIO_WritePin(AirPump2_GPIO_Port,AirPump2_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(AirPump2_GPIO_Port,AirPump2_Pin,GPIO_PIN_RESET)) 
#define VALVE_LEFT(n)  (n?HAL_GPIO_WritePin(AirValve1_GPIO_Port,AirValve1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(AirValve1_GPIO_Port,AirValve1_Pin,GPIO_PIN_RESET)) 
#define VALVE_RIGHT(n)  (n?HAL_GPIO_WritePin(AirValve2_GPIO_Port,AirValve2_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(AirValve2_GPIO_Port,AirValve2_Pin,GPIO_PIN_RESET)) 

typedef enum {
    EYE_NONE = 0,
    EYE_OS   = 1, // 左眼
    EYE_OD   = 2, // 右眼
    EYE_OU   = 3  // 双眼
} eye_t;

// 兼容旧代码的宏（后续可逐步替换为 eye_t 枚举量）
#define OS EYE_OS
#define OD EYE_OD
#define OU EYE_OU
typedef enum {
    WORK_IDLE = 0,
    WORK_INFLATE,
    WORK_HOLD,
    WORK_PULSE,
    WORK_DEFLATE,
} WorkState_t;


uint16_t ModbusCRC16(const uint8_t *data, uint16_t len) ;
void SendFrame(uint8_t seq, uint16_t response_id, const uint8_t *data, uint8_t data_len);
void SendPressure(u16 RP,u16 LP);
void SendEyeTemperature(u16 RT,u16 LT);
void SendWaterState(u8 WaterSta,u8 WaterSen,u16 WaterTemp);
void SendDrainWaterState(u8 drain_water_state);
void SendAddWaterState(u8 add_water_state);
void FrameDispatcher(const ProtocolFrame_t *frame);
void Work_Expression(u8 eye);
void Work_Heat(u8 eye);
u8 Work_DrainWater(void);
u8 Work_AddWater(void);
void Work_Init(void);
void Work_ResetPhases(void);
#endif

