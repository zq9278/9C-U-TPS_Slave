/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-15 13:31:10
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-07-01 18:14:23
 * @FilePath: \EIDEd:\Project\SLK01\Software\SLK-01-new\MDK-ARM\USER\pid\pid.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
// pid.c

#include "main.h"
#include "LOG.h"
#include "pid.h"

float p, i, d;

#define DERIVATIVE_FILTER_ALPHA 0.8f


void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
                  float integral_max, float integral_min,
                  float output_max, float output_min, float setpoint) 
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->setpoint = setpoint;
    pid->integral = 0;
    pid->derivative_filtered = 0;
    pid->previous_measured_value = 0;

    pid->integral_max = integral_max;
    pid->integral_min = integral_min;

    pid->output_max = output_max;
    pid->output_min = output_min;
}
									
float PID_Compute(PID_TypeDef *pid, float measured_value) 
{
    float error = pid->setpoint - measured_value;

    // 锟斤拷锟街硷拷锟姐及锟睫凤拷
    pid->integral += error;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    // 微锟街ｏ拷锟斤拷锟节诧拷锟斤拷值锟戒化锟斤拷+ 锟斤拷通锟剿诧拷
    float derivative = measured_value - pid->previous_measured_value;
//    pid->derivative_filtered = (DERIVATIVE_FILTER_ALPHA_NUM * pid->derivative_filtered +
//                                (DERIVATIVE_FILTER_ALPHA_DEN - DERIVATIVE_FILTER_ALPHA_NUM) * derivative)
//                               / DERIVATIVE_FILTER_ALPHA_DEN;
    pid->previous_measured_value = measured_value;

    // PID 锟斤拷锟斤拷锟斤拷悖拷锟较碉拷锟斤拷糯锟?000锟斤拷锟斤拷锟斤拷要锟斤拷锟斤拷去锟斤拷
    p = (pid->Kp * error) / 1000;
    i = (pid->Ki * pid->integral) / 1000;
    d = -(pid->Kd) / 1000;//-(pid->Kd * pid->derivative_filtered) / 1000;

    float output = p + i + d;

    // 锟斤拷锟斤拷薹锟?
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟窖★拷锟?
    //LOG("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", p, i, d,error, pid->integral,pid->setpoint, measured_value, output);

    return output;
}

