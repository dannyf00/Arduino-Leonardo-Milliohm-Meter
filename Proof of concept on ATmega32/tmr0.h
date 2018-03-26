#ifndef _TMR0_H
#define _TMR0_H

//header file to use tmr0
#include "gpio.h"

//prescaler settings
#define TMR0PS_1x			1
#define TMR0PS_8x			2
#define TMR0PS_64x			3
#define TMR0PS_256x			4
#define TMR0PS_1024x		5
#define TMR0PS_MASK			0x07

//rtc period settings
//tmr period settings
#define TMR_ms				(F_CPU / 1000)		//1us period - minimum period
#define TMR_1ms				(TMR_ms * 1)				//1ms
#define TMR_2ms				(TMR_ms * 2)				//2ms period
#define TMR_5ms				(TMR_ms * 5)				//5ms period
#define TMR_10ms			(TMR_ms * 10)				//10ms
#define TMR_20ms			(TMR_ms * 20)				//20ms period
#define TMR_50ms			(TMR_ms * 50)				//50ms period
//#define TMR_100ms			(TMR_ms * 100)				//100ms
//#define TMR_200ms			(TMR_ms * 200)				//200ms period
//#define TMR_500ms			(TMR_ms * 500)				//500ms period
//#define TMR_1000ms			(TMR_ms * 1000)				//1000ms
//#define TMR_2000ms			(TMR_ms * 2000)				//2000ms period
//#define TMR_5000ms			(TMR_ms * 5000)				//5000ms period
//#define TMR_10000ms			(TMR_ms * 10000)			//10000ms


//reset the tmr
void tmr0_init(unsigned char ps);
//set tmr1 isr ptr
void tmr0_act(void (*isr_ptr)(void));

#endif
