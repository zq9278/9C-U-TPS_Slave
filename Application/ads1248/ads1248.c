#include "ads1248.h"

extern SPI_HandleTypeDef hspi2;

const uint16_t temp_table_hot[TEMP_TABLE_HOT_SIZE] = {5000, 5101, 5206, 5314, 5428, 5546, 5670, 5800, 5937, 6080, 6232, 6393, 6563, 6744, 6938, 7144, 7368, 7610, 7871, 8159, 8477, 8828, 9226, 9679, 10200};
const uint16_t temp_table_mid[TEMP_TABLE_MID_SIZE] = {4993, 4973, 4953, 4933, 4914, 4895, 4875, 4856, 4837, 4819, 4800, 4782, 4763, 4745, 4727, 4709, 4691, 4673, 4656, 4638, 4621, 4604, 4587, 4570, 4553, 4536, 4519, 4503, 4486, 4470, 4454, 4438, 4421, 4406, 4390, 4374, 4358, 4343, 4327, 4312, 4297, 4282, 4267, 4252, 4237, 4222, 4207, 4193, 4178, 4164, 4149, 4135, 4121, 4107, 4093, 4079, 4065, 4051, 4037, 4024, 4010, 3996, 3983, 3970, 3956, 3943, 3930, 3917, 3904, 3891, 3878, 3865, 3853, 3840, 3827, 3815, 3802, 3790, 3778, 3765, 3753, 3741, 3729, 3717, 3705, 3693, 3681, 3669, 3658, 3646, 3634, 3623, 3611, 3600, 3588, 3577, 3566, 3555, 3543, 3532, 3521, 3510, 3499, 3488, 3477, 3467, 3456, 3445, 3434, 3424, 3413, 3403, 3392, 3382, 3371, 3361, 3351, 3340, 3330, 3320, 3310, 3300, 3290, 3280, 3270, 3260, 3250, 3240, 3230, 3220, 3211, 3201, 3191, 3182, 3172, 3163, 3153, 3144, 3134, 3125, 3116, 3107, 3097, 3088, 3079, 3070, 3061, 3052, 3043, 3034, 3025, 3016, 3007, 2998};
const uint16_t temp_table_cold[TEMP_TABLE_COLD_SIZE] = {62, 127, 195, 266, 339, 414, 493, 576, 661, 751, 845, 943, 1047, 1156, 1271, 1393, 1522, 1660, 1808, 1966, 2137, 2323, 2526, 2748, 2995};

void ADS1248_Reg_Set(u8 reg,u8 set)
{
	u8 TransData[2];
	TransData[0]=ADC_CMD_WREG|reg;
	TransData[1]=0x00;
	RTD_CS(0);
	HAL_SPI_Transmit(&hspi2,TransData,2,100);
	TransData[0]=set;
	HAL_SPI_Transmit(&hspi2,TransData,1,100);
	RTD_CS(1);
}
void ADS1248_SendCommand(uint8_t cmd)
{
    uint8_t tx = cmd;
    uint8_t rx = 0xFF;

    // 拉低 CS 开始通信
    RTD_CS(0);

    // 发送命令（例如 0x08 START）
    //HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi2,&tx,1,100);

    // 拉高 CS 结束通信
    RTD_CS(1);

    // 可选：给一点延时，保证命令生效
    HAL_Delay(1);
}

void ADS1248_Init(void)
{
	RTD_START(1);
	HAL_Delay(2);
	// ADS1248_Reg_Set(0x00,0x13);//AIN2 AIN3
	// ADS1248_Reg_Set(0x02,0x28);//REFP1 REFN1

	ADS1248_Reg_Set(0x00,0x01);//0x13:AIN2 AIN3;0x01:AIN0 AIN1
	ADS1248_Reg_Set(0x02,0x20);//0x28:REFP1 REFN1;0x20:REFP0 REFN0

	ADS1248_Reg_Set(0x03,0x03);//PGA=1,40SPS
	ADS1248_Reg_Set(0x0A,0x01);//IDAC=50uA
	ADS1248_Reg_Set(0x0B,0x89);//IEXC1 IEXC2
	
}

void ADS1248_ChangeChannel(u8 RTCNum)
{
    ADS1248_Init();
	    // 1. 停止当前转换
    ADS1248_SendCommand(0x0A);   // 0x0A
	if(RTCNum==RTD1)
	{
		ADS1248_Reg_Set(0x00,0x13);//0x13:AIN2 AIN3;0x01:AIN0 AIN1
		ADS1248_Reg_Set(0x02,0x28);//0x28:REFP1 REFN1;0x20:REFP0 REFN0
	}
	else if (RTCNum==RTD2) {
	ADS1248_Reg_Set(0x00,0x01);//0x13:AIN2 AIN3;0x01:AIN0 AIN1
	ADS1248_Reg_Set(0x02,0x20);//0x28:REFP1 REFN1;0x20:REFP0 REFN0
	}
	  // 3. 同步并唤醒滤波器
    ADS1248_SendCommand(0x04);   // 0x04
    ADS1248_SendCommand(0x02); // 0x02
	 // 4. 重新启动转换
    ADS1248_SendCommand(0x08);  // 0x08
}


u32 ADS1248_Read(void)
{
    u8  Cmd[5]={ADC_CMD_RDATA,ADC_CMD_NOP,ADC_CMD_NOP,ADC_CMD_NOP,ADC_CMD_NOP};  //最后一个字节是为了强制拉高nDRDY
    u8  Buf[5];
    u32  Data = 0;  

    RTD_CS(0);
        HAL_SPI_TransmitReceive(&hspi2,Cmd,Buf,5,100);        //1个命令，3个空操作接收数据，最后一个拉高nDRDY
    RTD_CS(1);


    Data=Buf[1];
		Data=(Data<<8)|Buf[2];
		Data=(Data<<8)|Buf[3];

		return Data; 
}
volatile int64_t r;
u16 ADC2Temperature(u32 adc_code)
{
    u16 resistance_ohm;
    if (adc_code == 0 || adc_code == 0xFFFFFF) {
        return 0; // invalid reading
    }
    resistance_ohm=((int64_t)adc_code * R_REF) / ADC_MAX;
    // 低温段：温度 0~30°C，电阻 > 8301Ω（电阻表是降序）
    if (resistance_ohm > 8301) {
        s16 temperature_index = (27445 - (s16)resistance_ohm) / 796;
        if (temperature_index >= TEMP_TABLE_COLD_SIZE - 1) return temp_table_cold[TEMP_TABLE_COLD_SIZE - 1];
				if (temperature_index <0) return 0;
        uint16_t resistance_start = 27445 - temperature_index * 796;
        uint16_t resistance_end = resistance_start - 796;
        uint16_t temperature_start = temp_table_cold[temperature_index];
        uint16_t temperature_end = temp_table_cold[temperature_index + 1];
				int32_t delta_r = (int32_t)(resistance_start - resistance_ohm);
				int32_t delta_t = (int32_t)(temperature_end - temperature_start);
				int32_t delta_res = (int32_t)(resistance_start - resistance_end);
        return temperature_start + (delta_r * delta_t) / delta_res;
    }

    // 中温段：温度 30~50°C，高精度段（表正序）
    else if (resistance_ohm >= 4170) {
        s16 temperature_index = ((s16)resistance_ohm - 4170) / 27;
        if (temperature_index >= TEMP_TABLE_MID_SIZE - 1) return temp_table_mid[TEMP_TABLE_MID_SIZE - 1];
				if (temperature_index <=0) return temp_table_mid[0];
        uint16_t resistance_start = 4170 + temperature_index * 27;
        uint16_t resistance_end = resistance_start + 27;
        uint16_t temperature_start = temp_table_mid[temperature_index];
        uint16_t temperature_end = temp_table_mid[temperature_index + 1];
        int32_t delta_r = (int32_t)(resistance_start - resistance_ohm);
				int32_t delta_t = (int32_t)(temperature_end - temperature_start);
				int32_t delta_res = (int32_t)(resistance_start - resistance_end);
        return temperature_start + (delta_r * delta_t) / delta_res;
    }

    // 高温段：温度 > 50°C，电阻 < 4170Ω（表降序）
    else {
        s16 temperature_index = (4170 - (s16)resistance_ohm) / 132;
        if (temperature_index >= TEMP_TABLE_HOT_SIZE - 1) return temp_table_hot[TEMP_TABLE_HOT_SIZE - 1];
				if (temperature_index <=0) return temp_table_hot[0];
        uint16_t resistance_start = 4170 - temperature_index * 132;
        uint16_t resistance_end = resistance_start - 132;
        uint16_t temperature_start = temp_table_hot[temperature_index];
        uint16_t temperature_end = temp_table_hot[temperature_index + 1];
        int32_t delta_r = (int32_t)(resistance_start - resistance_ohm);
				int32_t delta_t = (int32_t)(temperature_end - temperature_start);
				int32_t delta_res = (int32_t)(resistance_start - resistance_end);
        return temperature_start + (delta_r * delta_t) / delta_res;
    }
}

