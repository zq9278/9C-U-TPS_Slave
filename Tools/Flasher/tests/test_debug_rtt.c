#include "SEGGER_RTT.h"
#include "Modules/Debug/debug_rtt.h"
#include "App/System/system_app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
volatile sensor_data_t gSensorData={25.0f,26.0f,1.0f,2.0f};
volatile uint8_t gTreatmentRunning=1;
void *gCtrlCmdQueue=(void *)1;
static ctrl_cmd_t queued;
static unsigned queued_count;
int xQueueSend(void *queue,const void *item,unsigned timeout) {(void)queue;(void)timeout;queued=*(const ctrl_cmd_t *)item;queued_count++;return pdPASS;}
uint32_t xTaskGetTickCount(void) {return 123;}
size_t xPortGetFreeHeapSize(void) {return 4096;}
static char output[2048];
static unsigned read_log(void) {
    unsigned n=SEGGER_RTT_ReadUpBuffer(0,output,sizeof(output)-1);output[n]=0;return n;
}
int main(void) {
    DebugRtt_Init();read_log();
    assert(_SEGGER_RTT.aUp[0].Flags==SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    char data[900];memset(data,'x',sizeof(data));
    assert(DebugRtt_Write(data,900)==900);
    /* No debugger consumes the log: full-buffer writes must return, not block. */
    for(unsigned i=0;i<10000;i++)assert(DebugRtt_Write(data,900)==0);
    assert(read_log()==900);
    assert(SEGGER_RTT_WriteDownBuffer(0,"info\n",5)==5);DebugRtt_Poll();read_log();
    assert(strstr(output,"target=0x9C01") && strstr(output,"app="));
    assert(SEGGER_RTT_WriteDownBuffer(0,"status\n",7)==7);DebugRtt_Poll();read_log();
    assert(strstr(output,"running=1") && strstr(output,"log_drops=10000"));
    assert(SEGGER_RTT_WriteDownBuffer(0,"reset\n",6)==6);DebugRtt_Poll();read_log();
    assert(strstr(output,"ERR unknown command") && gTreatmentRunning==1);
    const char *commands[]={"pid get 1 0 1\n","pid set 2 3 2 0.7 0.3 0\n","pid stream 3 1\n"};
    for(unsigned i=0;i<3;i++){SEGGER_RTT_WriteDownBuffer(0,commands[i],strlen(commands[i]));for(unsigned j=0;j<4;j++)DebugRtt_Poll();assert(queued.request_id==i+1);}
    assert(queued_count==3 && queued.enabled==1);
    const char *bad[]={"pid set 4 3 2 nan 0 0\n","pid set 5 0 2 1 1 1\n","pid get 0 0 1\n","pid set 6 3 2 -1 0 0\n"};
    for(unsigned i=0;i<4;i++){SEGGER_RTT_WriteDownBuffer(0,bad[i],strlen(bad[i]));for(unsigned j=0;j<4;j++)DebugRtt_Poll();read_log();assert(strstr(output,"ok=0"));}
    assert(queued_count==3);
    puts("RTT: 10000 full-buffer writes returned; read-only firmware commands passed");
    return 0;
}
