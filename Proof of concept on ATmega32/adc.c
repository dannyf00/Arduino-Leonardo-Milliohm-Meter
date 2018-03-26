#include "adc.h"							//we use adc

//hardware configuration
//end hardware configuration

//global defines

//global variables

//initialize the adc
//using internal Vref
void adc_init(void) {
	ADCSRA = 	(1<<ADEN) |					//1->turn on the adc, 0->turn off the adc
				(0<<ADSC) |					//0->don't star the adc, 1->start the adc
				(0<<ADATE) |				//1->enable auto trigger, 0->disable auto trigger
				(0<<ADIF) |					//0->adc interrupt flag not set, 1->adc interrupt flag set
				(0<<ADIE) |					//1->enable adc interrupt, 0->disable adc interrupt
				(7<<ADPS0) |				//adc prescaler. 0..7: 2x .. 128x
				0x00;
	ADMUX = 	(ADMUX & 0x1f) |			//clear highest 3 bits (REFS1/REFS0/ADLAR)
				(3<<REFS0) |				//select reference: 0->AREF, 1->AVCC, 2->reserved, 3->internal 2.56v Vref
				(0<<ADLAR) |				//1->left adjust adc result, 0->right adjust adc result
				0x00;
	SFIOR = 	(SFIOR & 0x1f) |
				(0<<ADTS0) |				//0..7: 0->free running, 1->analog comparaator, 2-INT2, 3->timer0 compare match, 4->timer0 overflow, ...
				0x00;
}

//read adc
int16_t adc_read(uint8_t ain) {
	uint16_t tmp;
	ADMUX =		(ADMUX &~0x1f) |			//clear the MUX4..0 bits
				(ain & 0x1f);				//set the channel
	ADCSRA |= 	(1<<ADSC);					//start the conversion
	while (ADCSRA & (1<<ADSC)) continue;	//wait for conversion to finish
	//per datasheet, the following read orders must be maintained
	tmp = ADCL;								//must read adcl first
	tmp |= ADCH << 8;						//then read ADCH
	return tmp;
}
