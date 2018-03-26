#include "tmr1oc.h"							//tmr1 header file

//empty handler
static void /*_tmr1_*/empty_handler(void) {
	//default tmr handler
}

static void (* /*_tmr1*/_isr_ptrovf)(void)=empty_handler;	//tmr1_ptr pointing to empty_handler by default

//tmr1 overflow isr
ISR(TIMER1_OVF_vect) {
	//clear the flag - done automatically
	/*_tmr1*/_isr_ptrovf();						//execute the handler
}

//reset the tmr
void tmr1_init(unsigned char prescaler) {
	_isr_ptrovf = empty_handler;

	//configure the timer
	TCCR1B =	TCCR1B &~TMR1PS_MASK;			//turn off tmr1
	TCCR1A =	(0<<COM1A1) | (0<<COM1A0) |	//output compare a pins normal operation
				(0<<COM1B1) | (0<<COM1B0) |	//output compare b pins normal operation
				(0<<FOC1A) | (0<<FOC1B) |		//1->force a compare
				(0<<WGM11) | (0<<WGM10)		//wgm13..0 = 0b0100 -> ctc, top at ocr1a
				;
	TCCR1B =	(0<<ICNC1) |				//nput capture noise canceller disabled
				(0<<ICES1) |				//input capture edge selection on falling edge
				(0<<WGM13) | (0<<WGM12) |	//wgm13..0 = 0b0100 -> ctc, top at ocr1a
				(prescaler & TMR1PS_MASK) |	//prescaler, per the header file
				(0<<CS10) |					//0->turn off timer, 1..7->set prescaler / clock source
				0x00;
	//OCR1A = period-1;						//set the period
	TCNT1 = 0;								//reset the timer / counter
	TIFR |= (1<<OCF1A) | (0<<OCF1B) | (0<<TOV1);	//1->clear the flag, 0->no effect
	TIMSK&=~(	(0<<TICIE1) |				//input capture isr: disabled
				(1<<OCIE1B) |				//output compare isr for ch b: disabled
				(0<<OCIE1A) |				//output compare isr for ch c: disabled
				(0<<TOIE1))					//tmr overflow interrupt: enabled
				;
	//start the timer
	TCCR1B = 	(TCCR1B &~TMR1PS_MASK) | (prescaler & TMR1PS_MASK);
	//timer now starting
}

//install usr handler for timer overflow
void tmr1_act(void (*isr_ptr)(void)) {
	/*_tmr1*/_isr_ptrovf=isr_ptr;					//reassign tmr1 isr ptr
	TIFR |= (1<<TOV1);							//1->clera the flag, 0->no effect
	TIMSK|= (1<<TOIE1);						//1->enable the interrupt, 0->disable the interrupt
}
