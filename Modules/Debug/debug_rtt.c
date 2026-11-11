#include "debug_rtt.h"
#include "Config/product_config.h"
#include "Shared/Upgrade/upgrade.h"
#include "App/System/system_app.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#if TPS_RTT
#include "SEGGER_RTT.h"
static uint8_t telemetry[512];
static char line[128];
static unsigned used, overflow;
static uint32_t dropped;
void DebugRtt_Init(void) {
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(1,"PID samples",telemetry,sizeof(telemetry),SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    SEGGER_RTT_WriteString(0,"[APP] J-Link RTT ready; commands: help, info, status\r\n");
}
int DebugRtt_Write(const void *data,size_t size) {
    unsigned count=SEGGER_RTT_Write(0,data,(unsigned)size);
    if(count!=size) ++dropped;
    return (int)count;
}
int DebugRtt_Telemetry(const void *data,size_t size) {
    return (int)SEGGER_RTT_Write(1,data,(unsigned)size);
}
static void pid_command(void) {
    char *tokens[9], *part=strtok(line," "); unsigned n=0;
    ctrl_cmd_t cmd={0}; char *end; unsigned long values[3]={0};
    while(part && n<9) { tokens[n++]=part; part=strtok(NULL," "); }
    if(n<4 || part) goto invalid;
    for(unsigned i=2;i<n && i<5;i++) {
        values[i-2]=strtoul(tokens[i],&end,10);
        if(*end || *tokens[i]=='-' || values[i-2]>1000000UL) goto invalid;
    }
    if(!values[0]) goto invalid;
    cmd.request_id=(uint32_t)values[0];
    if(!strcmp(tokens[1],"stream") && n==4 && values[1]<=1) {
        cmd.id=CTRL_CMD_PID_STREAM_ENABLE; cmd.enabled=(uint8_t)values[1];
    } else if((!strcmp(tokens[1],"get") && n==5) || (!strcmp(tokens[1],"set") && n==8)) {
        if(values[1]>4 || !((values[1]<3 && values[2]<2) || (values[1]>=3 && values[2]==2))) goto invalid;
        cmd.pid_target=(pid_debug_target_t)values[1]; cmd.pid_profile=(uint8_t)values[2];
        cmd.id=n==5?CTRL_CMD_PID_GET_GAINS:CTRL_CMD_PID_SET_GAINS;
        if(n==8) {
            float *gains[3]={&cmd.kp,&cmd.ki,&cmd.kd};
            for(unsigned i=0;i<3;i++) {
                *gains[i]=strtof(tokens[5+i],&end);
                if(*end || !isfinite(*gains[i]) || *gains[i]<0 || *gains[i]>1000) goto invalid;
            }
        }
    } else goto invalid;
    if(gCtrlCmdQueue && xQueueSend(gCtrlCmdQueue,&cmd,0)==pdPASS) return;
    {
        char reply[96]; snprintf(reply,sizeof(reply),"RTT> PID ack=%lu ok=0 reason=queue_full\r\n",values[0]);
        DebugRtt_Write(reply,strlen(reply)); return;
    }
invalid:
    {char reply[96]; snprintf(reply,sizeof(reply),"RTT> PID ack=%lu ok=0 reason=invalid\r\n",values[0]); DebugRtt_Write(reply,strlen(reply));}
}
static void command(void) {
    char reply[192];
    if(!strncmp(line,"pid ",4)) { pid_command(); return; }
    if(!strcmp(line,"help")) {
        snprintf(reply,sizeof(reply),"RTT> help | info | status | pid get/set <req> <target> <profile> [kp ki kd] | pid stream <req> <0/1>; RAM only\r\n");
    } else if(!strcmp(line,"info")) {
        snprintf(reply,sizeof(reply),"RTT> app=%s target=0x%04lX hw=%u base=0x%08lX boot_min=0x%08lX\r\n",
                 PRODUCT_FIRMWARE_VERSION_STRING,UP_TARGET,(unsigned)UP_HW_REV,UP_APP_BASE,UP_BOOT_VERSION);
    } else if(!strcmp(line,"status")) {
        snprintf(reply,sizeof(reply),"RTT> tick=%lu running=%u tempL=%.2f tempR=%.2f pressL=%.2f pressR=%.2f free_heap=%lu log_drops=%lu\r\n",
            (unsigned long)xTaskGetTickCount(),gTreatmentRunning,(double)gSensorData.tempL,(double)gSensorData.tempR,
            (double)gSensorData.pressL,(double)gSensorData.pressR,(unsigned long)xPortGetFreeHeapSize(),(unsigned long)dropped);
    } else snprintf(reply,sizeof(reply),"RTT> ERR unknown command; use help\r\n");
    DebugRtt_Write(reply,strlen(reply));
}
void DebugRtt_Poll(void) {
    uint8_t bytes[32]; unsigned n=SEGGER_RTT_Read(0,bytes,sizeof(bytes));
    for(unsigned i=0;i<n;i++) {
        if(bytes[i]=='\r' || bytes[i]=='\n') {
            if(overflow) DebugRtt_Write("RTT> ERR line too long\r\n",24);
            else if(used) { line[used]=0; command(); }
            used=0; overflow=0;
        } else if(bytes[i]>=32 && bytes[i]<127) {
            if(used<sizeof(line)-1) line[used++]=(char)bytes[i];
            else overflow=1;
        }
    }
}
#else
void DebugRtt_Init(void) {}
int DebugRtt_Write(const void *data,size_t size) { (void)data; (void)size; return 0; }
int DebugRtt_Telemetry(const void *data,size_t size) { (void)data; (void)size; return 0; }
void DebugRtt_Poll(void) {}
#endif
