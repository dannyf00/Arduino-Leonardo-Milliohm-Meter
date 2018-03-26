#include "tmr2.h"							//tmr2 header file

//empty handler
static void /*_tmr3_*/empty_handler(void) {
	//default tmr handler
}

static void (* /*_tmr3*/_isr_ptrovf)(void)=empty_handler;	//tmr2_ptr pointing to empty_handler by default

//tmr2 overflow isr
ISR(TIMER2_OVF_vect) {
	//clear the flag - done automatically
	/*_tmr3*/_isr_ptrovf();						//execute the handler
}

//reset the tmr
void tmr2_init(unsigned char prescaler) {
	TCCR2 = 	(0<<FOC2) |					//1->force output compare
				(0<<WGM20) | (0<<WGM21) |	//WGM21..0=0b00->normal mode
				(0<<COM21) | (0<<COM20) |	//COM21..0=0b00->OC pints normal gpio operation
				(0<<CS22) | (0<<CS21) | (0<<CS20) |	//CS22..0=0b000->timer stopped
				0x00;								//
	//overflow top at 0xff
	TCNT2 = 0;								//reset the timer / counter
	//OCR2 = 0;								//reset OCR2
	//clera overflow interrupt flag, disable overflow interrupt
	TIFR  |= ((1<<TOV2) | (1<<OCF2));					//1->clera overflow interrupt
	TIMSK &=~((1<<TOIE2) | (1<<OCIE2));					//0->disable overflow interrupt, 1->enable overflow interrupt
	//clear output compare interrupt flag, disable output compare interrupt
	//TIMSK &=~(1<<OCIE2);					//0->disable output compare, 1->enable output compare
	//TIFR  |= (1<<OCF2);						//1->clear output compare flag

	//start the timer
	TCCR2 = (TCCR2 &~TMR2PS_MASK) | (prescaler & TMR2PS_MASK);
	//timer2 now running
}

//install timer2 overflow interrupt handler
void tmr2_act(void (*isr_ptr)(void)) {
	/*_tmr3*/_isr_ptrovf=isr_ptr;					//reassign tmr3 isr ptr
	//clera overflow interrupt flag, enable overflow interrupt
	TIFR  |= (1<<TOV2);					//1->clera overflow interrupt
	TIMSK |= (1<<TOIE2);					//0->disable overflow interrupt, 1->enable overflow interrupt
}

