#ifndef TEST_SYSTEM_APP_H
#define TEST_SYSTEM_APP_H
#include <stdint.h>
typedef struct {float tempL,tempR,pressL,pressR;} sensor_data_t;
extern volatile sensor_data_t gSensorData;
extern volatile uint8_t gTreatmentRunning;
typedef enum {CTRL_CMD_NONE,CTRL_CMD_PID_SET_GAINS,CTRL_CMD_PID_GET_GAINS,CTRL_CMD_PID_STREAM_ENABLE} ctrl_cmd_id_t;
typedef enum {PID_DEBUG_TARGET_PRESS_RISE,PID_DEBUG_TARGET_PRESS_HOLD,PID_DEBUG_TARGET_PRESS_PULSE,PID_DEBUG_TARGET_HEAT_LEFT,PID_DEBUG_TARGET_HEAT_RIGHT} pid_debug_target_t;
typedef struct {ctrl_cmd_id_t id;pid_debug_target_t pid_target;float kp,ki,kd;uint8_t enabled,pid_profile;uint32_t request_id;} ctrl_cmd_t;
extern void *gCtrlCmdQueue;
#define pdPASS 1
int xQueueSend(void *queue,const void *item,unsigned timeout);
#endif
