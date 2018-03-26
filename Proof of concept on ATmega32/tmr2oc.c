#include "tmr2oc.h"							//tmr2 header file

//empty handler
static void /*_tmr3_*/empty_handler(void) {
	//default tmr handler
}

static void (* /*_tmr3*/_isr_ptrovf)(void)=empty_handler;	//tmr2_ptr pointing to empty_handler by default
static void (* /*_tmr3*/_isr_ptroc)(void)=empty_handler;	//tmr2_ptr pointing to empty_handler by default
static unsigned char _tmr2oc_pr=0xff;

//tmr3 overflow isr
ISR(TIMER2_OVF_vect) {
	//clera the flag - done automatically
	//OCR2 += _tmr2oc_pr;							//advance to the next match point
	//TCCR2 |= (1<<FOC2);							//1->force a compare
	/*_tmr3*/_isr_ptrovf();						//execute the handler
}

//tmr3 compare isr
ISR(TIMER2_COMP_vect) {
	//clera the flag - done automatically
	OCR2 += _tmr2oc_pr;							//advance to the next match point
	//TCCR2 |= (1<<FOC2);							//1->force a compare
	/*_tmr3*/_isr_ptroc();						//execute the handler
}

//reset the tmr
void tmr2_init(unsigned char prescaler) {
	TCCR2 = 	(0<<FOC2) |					//1->force output compare
				(0<<WGM20) | (0<<WGM21) |	//WGM21..0=0b00->normal mode, 01->phase correct pwm, 10->CTC, 11->fast pwm
				(0<<COM21) | (0<<COM20) |	//COM21..0=0b00->OC pints normal gpio operation
				(0<<CS22) | (0<<CS21) | (0<<CS20) |	//CS22..0=0b000->timer stopped
				0x00;								//
	//overflow top at 0xff
	TCNT2 = 0;								//reset the timer / counter
	//OCR2 = 0xff;								//reset OCR2
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

//install timer2 output compare interrupt handler
void tmr2_act(void (*isr_ptr)(void)) {
	_isr_ptrovf=isr_ptr;						//install user handler
	//clear output compare interrupt flag, enable output compare interrupt
	TIFR  |= (1<<TOV2);						//1->clear output compare flag
	TIMSK |= (1<<TOIE2);					//0->disable output compare, 1->enable output compare
}

//set timer2 output compare period
void tmr2oc_setpr(unsigned char pr) {
	//clear output compare interrupt flag, disable output compare interrupt
	//TIFR  |= (1<<OCF2);						//1->clear output compare flag
	//TIMSK &=~(1<<OCIE2);					//0->disable output compare, 1->enable output compare
	_tmr2oc_pr = pr-1;
	OCR2 = TCNT2 + _tmr2oc_pr;
}

//install timer2 output compare interrupt handler
void tmr2oc_act(void (*isr_ptr)(void)) {
	_isr_ptroc=isr_ptr;						//install user handler
	//clear output compare interrupt flag, enable output compare interrupt
	TIFR  |= (1<<OCF2);						//1->clear output compare flag
	TIMSK |= (1<<OCIE2);					//0->disable output compare, 1->enable output compare
}
