#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED

#include "gpio.h"

//hardware configuration
//end hardware configuration

//global defines
//adc channels
#define AIN0				0
#define AIN1				1
#define AIN2				2
#define AIN3				3
#define AIN4				4
#define AIN5				5
#define AIN6				6
#define AIN7				7
#define AIN0_0x10			8
#define AIN1_0x10			9
#define AIN0_0x200			10
#define AIN1_0x200			11
#define AIN2_2x10			12
#define AIN3_2x10			13
#define AIN2_2x200			14
#define AIN3_2x200			15
#define AIN0_1				16
#define AIN1_1				17
#define AIN2_1				18
#define AIN3_1				19
#define AIN4_1				20
#define AIN5_1				21
#define AIN6_1				22
#define AIN7_1				23
#define AIN0_2				24
#define AIN1_2				25
#define AIN2_2				26
#define AIN3_2				27
#define AIN4_2				28
#define AIN5_2				29
#define AINVBG				30		//1.22Vbg
#define AINGND				31

//initialize the adc
//using internal Vref
void adc_init(void);

//read adc
int16_t adc_read(uint8_t ain);

#endif // ADC_H_INCLUDED
