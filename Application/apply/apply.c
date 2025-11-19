#include "apply.h"
#include "heat.h"
#include "water.h"
#include "Pressure_sensor.h"
#include "timers.h"
#include "LOG.h"
#include "Config/config.h"


#define DefultTemperature 3400

volatile u8 AirPump1PWM=127;//????PWM?0-255
volatile u16 PressureSet=30000;//????趨?5000-40000
volatile u16 RightTempSet,LeftTempSet;

/*
WorkMode
0x00 ????
0x01 ??????
0x02 ??????
0x03 ?????
0x04 ???????
0x05 ???????+??????
0x06 ???????+?????? 锟斤拷????
0x07 ???????+????? 锟斤拷????
0x08 ???????
0x09 ???????+?????? 锟斤拷????
0x0a ???????+??????
0x0b ???????+????? 锟斤拷????
0x0c ??????
0x0d ??????+?????? 锟斤拷????
0x0e ??????+?????? 锟斤拷????
0x0f ??????+?????


0xFF ????????????
*/
volatile u8 WorkMode=0;
volatile u8 WorkModeState=0;



extern volatile u8 WaterState;

extern PID_TypeDef RightHeat,LeftHeat;
extern volatile u16 RTDTemperature[2];

extern u16 left_pressure, right_pressure;
extern TIM_HandleTypeDef htim15;
extern UART_HandleTypeDef huart3;

uint16_t ModbusCRC16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= data[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void SendFrame(uint8_t seq, uint16_t response_id, const uint8_t *data, uint8_t data_len)
{
    uint8_t tx_buffer[32];  // ????????
    uint8_t len = 1 + 2 + data_len + 2;  // ???+ID+????+CRC
    uint8_t index = 0;

    tx_buffer[index++] = 0x5A;
    tx_buffer[index++] = 0xA5;
    tx_buffer[index++] = len;

    tx_buffer[index++] = seq;
    tx_buffer[index++] = (response_id >> 8) & 0xFF;
    tx_buffer[index++] = (response_id >> 0) & 0xFF;

    for (uint8_t i = 0; i < data_len; i++) {
        tx_buffer[index++] = data[i];
    }

    // ???? CRC??Modbus??
    uint16_t crc = ModbusCRC16(&tx_buffer[3], len - 2);  // ???CRC锟斤拷????
    tx_buffer[index++] = crc & 0xFF;        // ????????
    tx_buffer[index++] = (crc >> 8) & 0xFF;

    HAL_UART_Transmit(&huart3, tx_buffer, index, 100);  // ???? DMA
}

void SendPressure(u16 RP,u16 LP)
{
	u8 data_buffer[4];
	data_buffer[0]=RP>>8;
	data_buffer[1]=RP;
	data_buffer[2]=LP>>8;
	data_buffer[3]=LP;
	SendFrame(0x80,0x8001,data_buffer,4);
}

void SendEyeTemperature(u16 RT,u16 LT)
{
	u8 data_buffer[4];
	data_buffer[0]=RT>>8;
	data_buffer[1]=RT;
	data_buffer[2]=LT>>8;
	data_buffer[3]=LT;
	SendFrame(0x80,0x8002,data_buffer,4);
}

void SendWaterState(u8 WaterSta,u8 WaterSen,u16 WaterTemp)
{
	u8 data_buffer[4];
	data_buffer[0]=WaterSta;
	data_buffer[1]=WaterSen;
	data_buffer[2]=WaterTemp>>8;
	data_buffer[3]=WaterTemp;
	SendFrame(0x80,0x8003,data_buffer,4);
}

void SendDrainWaterState(u8 drain_water_state)
{
	u8 data_buffer[4];
	data_buffer[0]=0;
	data_buffer[1]=0;
	data_buffer[2]=0;
	data_buffer[3]=drain_water_state;
	SendFrame(0x80,0x8103,data_buffer,4);
}

void SendAddWaterState(u8 add_water_state)
{
	u8 data_buffer[4];
	data_buffer[0]=0;
	data_buffer[1]=0;
	data_buffer[2]=add_water_state;
	data_buffer[3]=0;
	SendFrame(0x80,0x8103,data_buffer,4);
}




 WorkState_t workState = WORK_IDLE;
TimerHandle_t xHoldTimer = NULL;      // 淇濆帇瀹氭椂鍣?
 TimerHandle_t xReleaseTimer = NULL;   // 娉勬皵瀹氭椂鍣?
 uint8_t currentEye = OD;
uint32_t holdSeconds   = 1000;  // 淇濆帇鏃堕棿锛坢s锛?uint32_t releaseSeconds = 500; // 娉勬皵鏃堕棿锛坢s锛?
// Pressure engine runtime counters (assuming 10ms tick via caller)
static uint32_t ramp_elapsed_ms = 0;
static uint32_t pulse_elapsed_ms = 0;
static uint32_t pulse_phase_elapsed_ms = 0;
static uint8_t  pulse_on_phase = 0;   // 0=off, 1=on
static uint8_t  pulse_alt_eye = 0;    // 0=right, 1=left (for alternate mode)

static void vHoldTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    //LOG("[HoldTimer] 淇濆帇闃舵缁撴潫锛屽紑濮嬫硠姘斻€俓r\n");

    switch (currentEye)
                case OD:
            AirValve1(1);
            break;
        case OS:
            AirValve2(1);
            break;
        case OU:
            AirValve1(1);
            AirValve2(1);
            break;
    }

    TIM15->CCR1 = 0;  // 鍋滄姘旀车
    workState = WORK_DEFLATE;
    //LOG("[HoldTimer] 鍒囨崲鐘舵€?-> WORK_DEFLATE\r\n");

    // 鍚姩鈥滄硠姘旀椂闂粹€濆畾鏃跺櫒
    //LOG("[HoldTimer] 鍚姩 ReleaseTimer锛屾椂闀?%d 绉抃r\n", releaseSeconds);
    xTimerChangePeriod(xReleaseTimer, pdMS_TO_TICKS(releaseSeconds), 0);
    xTimerStart(xReleaseTimer, 0);
}

// ==========================================================
// 娉勬皵闃舵缁撴潫 鈫?閲嶆柊杩涘叆鍏呮皵闃舵
// ==========================================================
static void vReleaseTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    //LOG("[ReleaseTimer] 娉勬皵闃舵缁撴潫锛屽叧闂榾闂紝鍑嗗鍐嶆鍏呮皵銆俓r\n");
  switch (currentEye)
                case OD:
            AirValve1(0);
            break;
        case OS:
             AirValve2(0);
            break;
        case OU:
            AirValve1(0);
    		AirValve2(0);
            break;
    }
    

    workState = WORK_INFLATE;
    //LOG("[ReleaseTimer] 鍒囨崲鐘舵€?-> WORK_INFLATE\r\n");
}

// 新的保压结束回调：切入脉动阶段（若配置开启），否则直接泄气
static void vHoldTimerCallback2(TimerHandle_t xTimer)
{
    (void)xTimer;
    const PressureProfile_t *pf = &g_settings.mode[(g_settings.mode_select<=1)?0:1];

    if (pf->t_pulse_total_ms > 0 && (pf->t_pulse_on_ms > 0 || pf->t_pulse_off_ms > 0))
    {
        // 进入脉动阶段
        pulse_elapsed_ms = 0;
        pulse_phase_elapsed_ms = 0;
        pulse_on_phase = 1; // 先ON
        pulse_alt_eye = 0;  // 右眼先开始
        workState = WORK_PULSE;
    }
    else
    {
        // 直接进入泄气
        switch (currentEye)
                case OD: AirValve1(1); break;
            case OS: AirValve2(1); break;
            case OU: AirValve1(1); AirValve2(1); break;
        }
        TIM15->CCR1 = 0;  // 停止气泵
        workState = WORK_DEFLATE;
        xTimerChangePeriod(xReleaseTimer, pdMS_TO_TICKS(releaseSeconds), 0);
        xTimerStart(xReleaseTimer, 0);
    }
}

// ==========================================================
// 鍒濆鍖栧嚱鏁帮細鍒涘缓涓や釜涓€娆℃€ц蒋浠跺畾鏃跺櫒
// ==========================================================
void Work_Init(void)
{
    //LOG("[Work_Init] 鍒濆鍖栧伐浣滅姸鎬佹満涓庡畾鏃跺櫒銆俓r\n");

    if (xHoldTimer == NULL)
    {
        xHoldTimer = xTimerCreate("HoldTimer",
                                  pdMS_TO_TICKS(holdSeconds),
                                  pdFALSE,
                                  NULL,
                                  vHoldTimerCallback2);
        //LOG("[Work_Init] 鍒涘缓 HoldTimer, 鍛ㄦ湡 = %d 绉掋€俓r\n", holdSeconds);
    }

    if (xReleaseTimer == NULL)
    {
        xReleaseTimer = xTimerCreate("ReleaseTimer",
                                     pdMS_TO_TICKS(releaseSeconds),
                                     pdFALSE,
                                     NULL,
                                     vReleaseTimerCallback);
        //LOG("[Work_Init] 鍒涘缓 ReleaseTimer, 鍛ㄦ湡 = %d 绉掋€俓r\n", releaseSeconds);
    }

    workState = WORK_IDLE;
    //LOG("[Work_Init] 鍒濆鐘舵€?-> WORK_IDLE\r\n");
}

// ==========================================================
// 涓绘帶鍒跺嚱鏁帮紙鍦ㄤ换鍔″惊鐜腑鍛ㄦ湡鎵ц锛?// ==========================================================
void Work_Expression(uint8_t eye)
{
    currentEye = eye;  // 璁板綍褰撳墠鎿嶄綔鐨勭溂锛堝彸/宸?鍙岋級
    //LOG("[Work_Expression] 褰撳墠宸ヤ綔鐘舵€? %d, Eye = %d\r\n", workState, currentEye);

    switch (workState)
    {
        // ---------- 鍒濆闃舵 ----------
        case WORK_IDLE:
            AirValve1(1);//鍏堟硠姘?
            AirValve2(1);
            TIM15->CCR1 = 0;
            // reset phase counters each cycle
            ramp_elapsed_ms = 0;
            pulse_elapsed_ms = 0;
            pulse_phase_elapsed_ms = 0;
            pulse_on_phase = 0;
            pulse_alt_eye = 0;
            workState = WORK_INFLATE;
            break;

        // ---------- 鍏呮皵闃舵 ----------
        case WORK_INFLATE:
                        // Apply slow ramp for PressureSet (10ms tick)
            const PressureProfile_t *pf = &g_settings.mode[(g_settings.mode_select<=1)?0:1];
            uint32_t dt_ms = 10;
            uint32_t target_pa = (uint32_t)(pf->target_kpa * 1000.0f);
            if (pf->t_rise_ms > 0 && ramp_elapsed_ms < pf->t_rise_ms)
            {
                uint32_t ramp_sp = (uint32_t)((uint64_t)target_pa * ramp_elapsed_ms / pf->t_rise_ms);
                PressureSet = (u16)ramp_sp;
                ramp_elapsed_ms += dt_ms;
            }
            else
            {
                PressureSet = (u16)target_pa;
            }
            switch (eye)
            {
                case OD://鍙崇溂
                    if (right_pressure <= Pressure_Deflation) {//鍏呮皵闃舵锛屽彸鍏?
                        AirValve1(0);
                        AirValve2(1);
                        TIM15->CCR1 = AirPump1PWM;
                    }
                    if (right_pressure >= PressureSet) {
                        TIM15->CCR1 = 0;
                        xTimerStart(xHoldTimer, 0);
                        workState = WORK_HOLD;
                    }
                    break;

                case OS:
                    if (left_pressure <= Pressure_Deflation) {
                        AirValve2(0);
                        AirValve1(1);
                        TIM15->CCR1 = AirPump1PWM;
                    }
                    if (left_pressure >= PressureSet) {
                        TIM15->CCR1 = 0;
                        xTimerStart(xHoldTimer, 0);
                        workState = WORK_HOLD;
                    }
                    break;

                case OU:
                    if (right_pressure <= Pressure_Deflation && left_pressure <= Pressure_Deflation) {
                        AirValve1(0);
                        AirValve2(0);
                        TIM15->CCR1 = AirPump1PWM;
                    }
                    if (right_pressure >= PressureSet && left_pressure >= PressureSet) {
                        TIM15->CCR1 = 0;
                        xTimerStart(xHoldTimer, 0);
                        workState = WORK_HOLD;
                    }
                    break;
            }
            break;

        // ---------- 淇濆帇闃舵 ----------
        case WORK_HOLD:
            //LOG("[WORK_HOLD] 淇濆帇涓紝绛夊緟瀹氭椂鍣ㄨЕ鍙戞硠姘斻€俓r\n");
            break;

        // ---------- 脉动阶段 ----------
        case WORK_PULSE:
        {
            const PressureProfile_t *pf = &g_settings.mode[(g_settings.mode_select<=1)?0:1];
            uint32_t dt_ms = 10;
            // End condition
            if (pulse_elapsed_ms >= pf->t_pulse_total_ms)
            {
                // enter deflate
                switch (currentEye)
                {
                    case OD: AirValve1(1); AirValve2(0); break;
                    case OS: AirValve2(1); AirValve1(0); break;
                    case OU: AirValve1(1); AirValve2(1); break;
                }
                TIM15->CCR1 = 0;
                workState = WORK_DEFLATE;
                xTimerChangePeriod(xReleaseTimer, pdMS_TO_TICKS(releaseSeconds), 0);
                xTimerStart(xReleaseTimer, 0);
                break;
            }

            // Update on/off phase
            uint32_t phase_target = pulse_on_phase ? pf->t_pulse_on_ms : pf->t_pulse_off_ms;
            if (phase_target == 0) {
                pulse_on_phase = !pulse_on_phase;
                pulse_phase_elapsed_ms = 0;
            } else if (pulse_phase_elapsed_ms >= phase_target) {
                pulse_on_phase = !pulse_on_phase;
                pulse_phase_elapsed_ms = 0;
                if (pf->squeeze_mode == 1 && (eye == OU))
                    pulse_alt_eye ^= 1;
            }

            // Apply valve/pump based on phase and mode
            if (pulse_on_phase) {
                // ON: inflate/maintain
                if (pf->squeeze_mode == 1 && (eye == OU)) {
                    if (pulse_alt_eye == 0) { // right on
                        AirValve1(0); AirValve2(1);
                    } else {
                        AirValve2(0); AirValve1(1);
                    }
                    TIM15->CCR1 = AirPump1PWM;
                } else {
                    if (eye == OD)      { AirValve1(0); AirValve2(1); }
                    else if (eye == OS) { AirValve2(0); AirValve1(1); }
                    else                { AirValve1(0); AirValve2(0); }
                    TIM15->CCR1 = AirPump1PWM;
                }
            } else {
                // OFF: release
                if (pf->squeeze_mode == 1 && (eye == OU)) {
                    AirValve1(1); AirValve2(1); TIM15->CCR1 = 0;
                } else {
                    if (eye == OD)      { AirValve1(1); AirValve2(0); }
                    else if (eye == OS) { AirValve2(1); AirValve1(0); }
                    else                { AirValve1(1); AirValve2(1); }
                    TIM15->CCR1 = 0;
                }
            }

            // Advance time
            pulse_elapsed_ms      += dt_ms;
            pulse_phase_elapsed_ms += dt_ms;
            break;
        }

        // ---------- 娉勬皵闃舵 ----------
        case WORK_DEFLATE:
            //LOG("[WORK_DEFLATE] 娉勬皵涓紝绛夊緟 ReleaseTimer 瑙﹀彂銆俓r\n");
            break;

        default:
            //LOG("[Work_Expression] 鏈煡鐘舵€侊細%d\r\n", workState);
            break;
    }
}





void Work_Heat(u8 eye)
{
	if(eye==OD)
	{
		PID_Heat(Right,&RightHeat,RTDTemperature[0]);
	}
	if(eye==OS)
	{
		PID_Heat(Left,&LeftHeat,RTDTemperature[1]);
	}
	if(eye==OU)
	{
		PID_Heat(Right,&RightHeat,RTDTemperature[0]);
		PID_Heat(Left,&LeftHeat,RTDTemperature[1]);
	}
}

u8 Work_DrainWater(void)
{
	//???????return 2?????return 0?????return 1
	static u16 delay_count;
	static u8 DrainWaterState;
	switch(DrainWaterState)
		{
			case 0:
				//??????
				WaterPump1(0);
				DrainWaterState=1;
				break;
			case 1:
				//???1s
				delay_count++;
				if(delay_count==100)
				{
					delay_count=0;
					DrainWaterState=2;
				}
			case 2:
				//?????锟斤拷????????
				WaterValve1(1);
				WaterValve2(1);
				WaterValve3(1);
				WaterValve4(1);
				WaterValve5(1);
				DrainWaterState=3;
				break;
			case 3:
				//???1s
				delay_count++;
				if(delay_count==100)
				{
					delay_count=0;
					DrainWaterState=4;
				}
				break;
			case 4:
				//????????
				WaterPump1(1);
				DrainWaterState=5;
				break;
			case 5:
				//????锟斤拷?????锟斤拷??????
				if(HAL_GPIO_ReadPin(WS1_GPIO_Port,WS1_Pin)==1)
				{
					DrainWaterState=6;
				}
				delay_count++;
				if(delay_count==2000)
				{
					delay_count=0;
					DrainWaterState=0;
					WaterPump1(0);
					AirPump3(0);
					WaterValve1(0);
					WaterValve2(0);
					WaterValve3(0);
					WaterValve4(0);
					WaterValve5(0);
					WaterState=WaterState&0xFD;
					WorkMode=WorkMode&0xEF;
					return 0;
				}
				break;
			case 6:
				//???10s
				delay_count++;
				if(delay_count==1500)
				{
					delay_count=0;
					DrainWaterState=7;
				}
				break;
			case 7:
				//?????????????
				AirPump3(1);
				DrainWaterState=8;
				break;
			case 8:
				//???10s
				delay_count++;
				if(delay_count==1000)
				{
					delay_count=0;
					DrainWaterState=9;
				}
				break;
			case 9:
				//?????????????
				AirPump3(0);
				DrainWaterState=10;
				break;
			case 10:
				//???10s
				delay_count++;
				if(delay_count==1000)
				{
					delay_count=0;
					DrainWaterState=11;
				}
				break;
			case 11:
				//??????
				delay_count=0;
				WaterPump1(0);
				WorkMode=WorkMode&0xEF;
				DrainWaterState=0;
				WaterState=WaterState&0xFD;
				WaterValve1(0);
				WaterValve2(0);
				WaterValve3(0);
				WaterValve4(0);
				WaterValve5(0);
				return 1;
			default:
				delay_count=0;
				WorkMode=WorkMode&0xEF;
				WaterState=WaterState&0xFD;
				DrainWaterState=0;
				WaterPump1(0);
				AirPump3(0);
				WaterValve1(0);
				WaterValve2(0);
				WaterValve3(0);
				WaterValve4(0);
				WaterValve5(0);
				return 0;
		}
		WaterState=WaterState|0x02;
		return 2;
}

u8 Work_AddWater(void)
{
	//???????return 2?????return 0?????return 1
	static u16 AddWater_delay_count,WaterSenseCount;
	static u8 AddWaterState;
	switch(AddWaterState)
		{
			case 0:
				//??????
				WaterPump1(0);
				WaterValve5(1);
				AddWaterState=1;
				break;
			case 1:
				//???1s
				AddWater_delay_count++;
				if(AddWater_delay_count==100)
				{
					AddWater_delay_count=0;
					AddWaterState=2;
				}
			case 2:
				//????锟斤拷?????锟斤拷??????
				if(HAL_GPIO_ReadPin(WS1_GPIO_Port,WS1_Pin)==0)
				{
					AddWaterState=3;
					WaterSenseCount=0;
				}
				WaterSenseCount++;
				if(WaterSenseCount==30000)
				{
					WaterPump1(0);
					WaterValve5(1);
					AddWater_delay_count=0;
					WorkMode=WorkMode&0xDF;
					WaterState=WaterState&0xFB;
					WaterSenseCount=0;
					AddWaterState=0;
					return 0;
				}
				break;
			case 3:
				//???????
				WaterPump1(1);
				WaterValve5(0);
				AddWaterState=4;
				break;
			case 4:
				//???10s
				AddWater_delay_count++;
				if(AddWater_delay_count==1000)
				{
					AddWater_delay_count=0;
					AddWaterState=5;
				}
				break;
			case 5:
				//????锟斤拷?????锟斤拷??????
				WaterValve5(1);
				if(HAL_GPIO_ReadPin(WS1_GPIO_Port,WS1_Pin)==0)
				{
					AddWaterState=6;
				}
				WaterSenseCount++;
				if(WaterSenseCount==30000)
				{
					WaterPump1(0);
					WaterValve5(1);
					AddWater_delay_count=0;
					WorkMode=WorkMode&0xDF;
					WaterState=WaterState&0xFB;
					WaterSenseCount=0;
					AddWaterState=0;
					return 0;
				}
				break;
			case 6:
				//???1s
				WaterValve5(0);
				AddWater_delay_count++;
				if(AddWater_delay_count==100)
				{
					AddWater_delay_count=0;
					AddWaterState=7;
				}
				break;
			case 7:
				//????锟斤拷?????锟斤拷??????
				if(HAL_GPIO_ReadPin(WS1_GPIO_Port,WS1_Pin)==0)
				{
					WaterPump1(0);
					WaterValve5(0);
					AddWater_delay_count=0;
					WorkMode=WorkMode&0xDF;
					WaterState=WaterState&0xFB;
					WaterSenseCount=0;
					AddWaterState=0;
					return 1;
				}
				else
				{
					AddWaterState=5;
				}
				break;
			
			default:
				WaterPump1(0);
				WaterValve5(0);
				AddWater_delay_count=0;
				WorkMode=WorkMode&0xDF;
				WaterState=WaterState&0xFB;
				WaterSenseCount=0;
				AddWaterState=0;
				return 0;
		}
		WaterState=WaterState|0x04;
		return 2;
}


void FrameDispatcher(const ProtocolFrame_t *frame)
{
    uint16_t id = (frame->frame_id[0] << 8) | frame->frame_id[1];

    switch (id)
    {
        case 0x0001:
            // ?????
						if((frame->payload[0]&0x03)==0x00)
						{
							WorkMode=WorkMode&0xFC;
							WorkModeState=0;
							HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);
							TIM15->CCR1=0;
							AirValve1(0);
							AirValve2(0);
						}
						else
						{
							WorkMode=WorkMode|(frame->payload[0]&0x03);
							AirPump1PWM=frame->payload[1];
							PressureSet=(frame->payload[2]<<8)|frame->payload[3];
							HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
							TIM15->CCR1=AirPump1PWM;
						}
            break;
						
				case 0x0002:
					//??????	
					WorkMode=WorkMode&0xF3;
					WorkMode=WorkMode|((frame->payload[0]&0x03)<<2);			
			
					RightTempSet=DefultTemperature+50*frame->payload[2];//????????趨
					LeftTempSet=DefultTemperature+50*frame->payload[3];//????????趨
					PID_Init(&RightHeat,400,2,200,100000,0,1999,0,RightTempSet);
					PID_Init(&LeftHeat,400,2,200,100000,0,1999,0,LeftTempSet);
					HeatPower(1,(frame->payload[0]&0x02)>>1);//???????
					HeatPWMSet(1,0);
				  HeatPower(2,frame->payload[0]&0x01);//???????
					HeatPWMSet(2,0);
					break;
					
				case 0x0003:
					//?锟斤拷????
					if(frame->payload[0]+frame->payload[1]+frame->payload[2]==1 && (WorkMode&0xF0)==0)//??????????????????????????????
					{
						if(frame->payload[0]==0x01)
						{
							//???
							WorkMode=WorkMode|0x20;
						}
						else if(frame->payload[1]==0x01)
						{
							//???
							WorkMode=WorkMode|0x10;
						}
						else if(frame->payload[2]==0x01)
						{
							//????????
							WaterState=StartWaterPump();
						}
					}
					else if(frame->payload[2]==0)
					{
						//???????
						WaterState=0;
						StopWaterPump();
					}
					break;
					
				case 0x0004:
					//????????
					if(frame->payload[0]==0x00)
					{
						StopIPLCold();
					}
					else
					{
						StartIPLCold(frame->payload[3]);
					}
					break;
        default:
            // 锟斤拷????????
            break;
    }

    // ???????
    if (frame->payload)
        vPortFree(frame->payload);
}















