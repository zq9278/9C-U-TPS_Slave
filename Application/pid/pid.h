// pid.h

#ifndef PID_H
#define PID_H
#include "stm32g0xx_hal.h"
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
    float integral_max;   // �����޷�ֵ
    float integral_min;   // �������޷�ֵ
    float output_max;     // ����޷�ֵ
    float output_min;     // ������޷�ֵ
    float setpoint;       // �趨ֵ
    float previous_measured_value ;
    float derivative_filtered ;
} PID_TypeDef;

// ��ʼ��PID������
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
                  float integral_max, float integral_min,
                  float output_max, float output_min, float setpoint) ;

// ����PID�������
float PID_Compute(PID_TypeDef *pid, float measured_value);

extern volatile float gPidP;
extern volatile float gPidI;
extern volatile float gPidD;

// �޷���
//#define Limit(x, min, max) ((x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x))))
#define Limit(x, min, max) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

#endif // PID_H
