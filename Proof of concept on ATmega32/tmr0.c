#include "tmr0.h"							//tmr0 header file

//empty handler
static void /*_tmr0_*/empty_handler(void) {
	//default tmr handler
}

//uses roman black's zero cumulative error approach
static void (* /*_tmr0*/_isr_ptrovf)(void)=empty_handler;				//tmr0_ptr pointing to empty_handler by default

//tmr0 overflow isr
ISR(TIMER0_OVF_vect) {
	//clear the flag - done automatically
	/*_tmr0*/_isr_ptrovf();					//execute the handler
}


//reset the tmr
void tmr0_init(unsigned char ps) {
	_isr_ptrovf = empty_handler;

	//configure the timer
	TCCR0 = 	(0<<FOC0) |					//1->force output compare
				(0<<WGM01) | (0<<WGM00)	|	//00->normal mode, 01->phase correct pwm, 10->CTC (top @ OCR0), 11->fast pwm
				(0<<COM00) |				//0->normal mode, 1->...
				(0<<CS00) |					//0->no clock / timer stopped, ...
				0x00;
	//OCR1A = period;						//minimum time interval is 1ms
	TCNT0 = 0;								//reset the timer / counter

	//clear the flag and disable the interrupt
	TIFR |= (1<<OCF0) | (1<<TOV0);			//1->clera the flags, 0->no effect
	TIMSK&=~((1<<OCIE0) | (1<<TOIE0));		//1->enable the interrupts, 0->disable the interrupts
	//clear the timer
	TCCR0 = (TCCR0 &~TMR0PS_MASK) | (ps & TMR0PS_MASK);
	//timer now running
}

void tmr0_act(void (*isr_ptr)(void)) {
	/*_tmr0*/_isr_ptrovf=isr_ptr;					//reassign tmr0 isr ptr
	TIFR |= (1<<TOV0);						//1->clear the interrupts, 0->no effect
	TIMSK|= (1<<TOIE0);						//1->enable the interrupt, 0->disable the interrupt
}

