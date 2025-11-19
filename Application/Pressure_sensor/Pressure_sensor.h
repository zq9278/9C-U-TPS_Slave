/*
 * @Author: zhangqi 
 * @Date: 2024-12-09 14:17:12 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-09 14:58:08
 */
#ifndef Presure_sensor
#define Presure_sensor
#include "main.h"
#include <stdint.h>

#define Pressure_sensorAdd	0x78
#define Pressure_Deflation  50

void pressure_sensor_read(void);
#endif
