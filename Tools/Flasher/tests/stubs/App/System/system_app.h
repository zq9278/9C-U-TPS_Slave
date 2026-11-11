#ifndef TEST_SYSTEM_APP_H
#define TEST_SYSTEM_APP_H
#include <stdint.h>
typedef struct {float tempL,tempR,pressL,pressR;} sensor_data_t;
extern volatile sensor_data_t gSensorData;
extern volatile uint8_t gTreatmentRunning;
#endif
