#ifndef _LED_H
#define _LED_H
#include <sys.h>	  


#define LED2	2
#define LED3	3
#define LED4	4

#define LEDOff	0
#define LEDOn		1
#define LEDPWM	2
	
	
void LEDPWMSet(u8 LEDNub,u16 PWMVal);
void LEDCtrl(u8 Nub,u8 State);
//void LEDUpdate(void);

#endif

