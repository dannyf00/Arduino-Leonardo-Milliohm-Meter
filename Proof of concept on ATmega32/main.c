//#include <avr/io.h>							//we use gcc-avr
#include "gpio.h"
#include "delay.h"							//we use software delays
#include "led4_pins.h"						//we use 7-segment led
#include "tmr2oc.h"							//we use timer2
#include "adc.h"							//we use adc

//hardware configuration
#define LED_PORT			PORTC
#define LED_DDR				DDRC
#define LED					(1<<0)

#define RuOHM				(440000ul)	//current limit resistors, in uOHM - to be adjusted by user
#define AIN_uOHMx1			AIN0_1			//1x input
#define AIN_uOHMx10			AIN1_0x10		//10x input
#define AIN_uOHMx200		AIN1_0x200		//200x input
#define uOHM_THx1			64				//threshold for 1x gain. below which, switch to 10x/200x gains. must be less than 1024/10
#define uOHM_THx10			32				//threshold for 10x gain. below which, switch to 200x gain. must be less than 1024/(200/100
//end hardware configuration

//global defines
#define VREF				2560000ul		//ADC vref, in uv
#define ADCx1_uOHM(adc)		((adc) * (VREF / 1024 / 100) * (RuOHM / 10) / 5)			//convert ADC(1x) to uOHM
#define ADCx10_uOHM(adc)	(ADCx1_uOHM(adc) / 10)
#define ADCx200_uOHM(adc)	(ADCx1_uOHM(adc) / 200)


//global variables
uint32_t uOHM=0;							//DUT microOhm reading

//flip led
void led_flp(void) {
	IO_FLP(LED_PORT, LED);
}

int main(void) {
	uint32_t tmp;
	char dp;								//decimal point

	mcu_init();								//reset the mcu
	//initailize the led
	led_init();
	//lRAM[0]=ledfont_num[1]; lRAM[1]=ledfont_num[2]; lRAM[2]=ledfont_num[3] | 0x80; lRAM[3]=ledfont_num[4];
	//initialize timer2
	tmr2_init(TMR2PS_64x);
	tmr2_act(led_display);
	//tmr2oc_setpr(100);
	//tmr2oc_act(led_display);
	//IO_OUT(LED_DDR, LED);
	adc_init();							//initialize the adc
	ei();
	while(1) {
		//IO_FLP(LED_PORT, LED);
		//display uOHM in milliOHM - range 0.001 to 9999. mOhm
		//tmp = uOHM;
		//uOHM = 9999567ul;
		//uOHM = 10000;												//for debugging only
		//uOHM = -adc_read(AIN_uOHMx1);								//adc at 1x gain - reversed polarity
		//if (uOHM > uOHM_THx1) uOHM = ADCx1_uOHM(uOHM);				//result sufficiently high for 1x gain. convert to uOHM
		//else {
			uOHM = adc_read(AIN_uOHMx10);							//adc at 10x gain
			if (uOHM > uOHM_THx10) uOHM = ADCx10_uOHM(uOHM);		//result sufficiently high for 10x gain. convert to uOHM
			else {
				uOHM = adc_read(AIN_uOHMx200);						//adc at 200x gain
				uOHM = ADCx200_uOHM(uOHM);							//convert to uOHM
			}
		//}
		//optional 1: adc offset correction
		//optional 2: oversampling
		//optional 3: put uOHM error correction here

		//display uOHM
		//uOHM = 10; uOHM = ADCx1_uOHM(uOHM);						//for debugging only
		//uOHM = 1100;												//for debugging only
		tmp = (uOHM >=9999000ul)?9999000ul:uOHM;					//top out at 9999mohm
		//now uOHM max is 9999mOhm
		     if (tmp > 999999ul) {tmp = uOHM / 1000; dp = 3;}		//1000000..9999999uOHM
		else if (tmp >  99999ul) {tmp = uOHM /  100; dp = 2;}		//100000..999999uOHM
		else if (tmp >   9999ul) {tmp = uOHM /   10; dp = 1;}		//10000..99999uOHM
		else                     {tmp = uOHM /    1; dp = 0;}		//0..9999uOHM
		//convert tmp to font
		lRAM[3]= ledfont_num[tmp % 10]; tmp /= 10;
		lRAM[2]= ledfont_num[tmp % 10]; tmp /= 10;
		lRAM[1]= ledfont_num[tmp % 10]; tmp /= 10;
		lRAM[0]= ledfont_num[tmp % 10]; tmp /= 10;
		lRAM[dp] |= 0x80;							//add decimal point
		delay_ms(100);


	}

	return 0;
}
