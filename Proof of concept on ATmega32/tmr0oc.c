#include "tmr0oc.h"							//tmr0 header file

//empty handler
static void /*_tmr0_*/empty_handler(void) {
	//default tmr handler
}

//uses roman black's zero cumulative error approach
static void (* /*_tmr0*/_isr_ptrovf)(void)=empty_handler;				//tmr0_ptr pointing to empty_handler by default
static void (* /*_tmr0*/_isr_ptrocf)(void)=empty_handler;				//tmr0_ptr pointing to empty_handler by default
static volatile unsigned char _tmr0oc_pr=0xff;							//compare period

//tmr0 compare isr
ISR(TIMER0_OVF_vect) {
	//clear the flag - done automatically
	//OCR0 += _tmr0oc_pr;						//advance tot he next match point
	/*_tmr0*/_isr_ptrovf();					//execute the handler
}

//tmr0 compare isr
ISR(TIMER0_COMP_vect) {
	//clear the flag - done automatically
	OCR0 += _tmr0oc_pr;						//advance tot he next match point
	/*_tmr0*/_isr_ptrocf();					//execute the handler
}

//reset the tmr
void tmr0_init(unsigned char ps) {
	_isr_ptrovf = _isr_ptrocf = empty_handler;
	_tmr0oc_pr = 0xff;

	//configure the timer
	TCCR0 = 	(0<<FOC0) |					//1->force output compare
				(0<<WGM01) | (0<<WGM00)	|	//00->normal mode, 01->phase correct pwm, 10->CTC (top @ OCR0), 11->fast pwm
				(0<<COM00) |				//0->normal mode, 1->...
				(0<<CS00) |					//0->no clock / timer stopped, ...
				0x00;
	//OCR0 = 0xff;						//minimum time interval is 1ms
	TCNT0 = 0;								//reset the timer / counter

	//clear the flag and disable the interrupt
	TIFR |= (1<<OCF0) | (0<<TOV0);			//1->clera the flags, 0->no effect
	TIMSK&=~((1<<OCIE0) | (0<<TOIE0));		//1->enable the interrupts, 0->disable the interrupts
	//clear the timer
	TCCR0 = (TCCR0 &~TMR0PS_MASK) | (ps & TMR0PS_MASK);
	//timer now running
}

//install the handler
void tmr0_act(void (*isr_ptr)(void)) {
	_isr_ptrovf = isr_ptr;					//install user handler
	TIFR |= (1<<TOV0);						//1-.clera the flag, 0->no effect
	TIMSK|= (1<<TOIE0);						//1->set the interrupt, 0->disable the interrupt
}
//set output compare period
void tmr0oc_setpr(unsigned char pr) {
	_tmr0oc_pr = pr-1;
	OCR0 = TCNT0 + _tmr0oc_pr;
}
//install the handler
void tmr0oc_act(void (*isr_ptr)(void)) {
	_isr_ptrocf = isr_ptr;					//install user handler
	TIFR |= (1<<OCF0);						//1-.clera the flag, 0->no effect
	TIMSK|= (1<<OCIE0);						//1->set the interrupt, 0->disable the interrupt
}
