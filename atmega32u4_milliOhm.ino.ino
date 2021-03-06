//milliOhm meter with 4-digit 7-segment LED display
//built on ATmega32U4/Arduino Leonardo
//Revision history:
//v0.1 @ 3/25/2018: initial release (no ADC)
//v0.2 @ 3/27/2018: 1) ADC implemented; 2) auto ranging implemented; 3) AFE schematic shown
//v0.3 @ 3/28/2018: 1) implemented ADC oversampling
//v0.3a@ 3/29/2018: 1) implemented manual ADC calibration
//v0.4 @ 3/30/2018: going live.
//v0.5 @ 3/31/2018: 1) added exponential smoothing; 2) all channels calibrated for ADC offset errors
//v0.6 @ 3/31/2018: 1) added serial output capability; 2) slowed down the LED update frequency to 10SPS.
//
//connections:
//
//Analog Front End:
//
//    +5v
//     |
//     |
//     |
//     |
//     |
//    [ ] R1 (220R)
//     |
//     |       R4 (1K)
//     |------------[ ]--------------- A3 of Leonardo
//     |                      |
//     |                      |
//    [ ] DUT (<10R)          = C1 (.1u)
//     |                      |
//     |                      |
//     |------------[ ]--------------- A5 of Leonardo
//     |       R5 (1K)
//     |
//    [ ] R2 (220R)
//     |
//     |
//     |
//     |
//     |
//    GND

//LED display:
//DIG1..4 connects to Leonardo directly;
//SEGA..G + DP connects to Leonardo via 8x 1K resistors
//connections:
//LED		Arduino		AVR
//==========================
//SEGE		13			PC7
//SEGD		12			PD6
//SEGDP	 	11			PB7
//SEGC		4		 	PD4
//SEGG		3		 	PD0
//DIG4		2		 	PD1

//DIG1		10			PB6
//SEGA		9		 	PB5
//SEGF		8		 	PB4
//DIG2		7		 	PE6
//DIG3		6		 	PD7
//SEGB		5		 	PC6

//hardware configuration
#define LEDR_PORT		 	PORTC
#define LEDR_DDR			DDRC
#define LEDR				(1<<7)
#define LEDR_DLY			100

#define USE_UART							//uncomment if serial output is desired
//smooth out the uOHM reading - higher number means more weight to historical data
#define uOHM_AVGN			16				//weight for exponential smoothing
#define uOHM_DLY			100				//delays between each measurement of uOHM, in ms
#define RuOHM				(440000ul)		//current limit resistors, in mOHM - to be adjusted by user
#define RuOHM_10x			(RuOHM)			//current limit resistors, in mOHM - to be adjusted by user
#define RuOHM_40x			(RuOHM)			//current limit resistors, in mOHM - to be adjusted by user
#define RuOHM_200x			(RuOHM)			//current limit resistors, in mOHM - to be adjusted by user
//#define AIN_uOHMx1			AIN0_1			//1x input
#define AIN_10x				AIN4_0x10		//PGA gain for high resistance value: 10x input
#define AIN_40x				AIN4_0x40		//PGA gain for mediaum resistance value: 40x input
#define AIN_200x			AIN4_0x200		//PGA gain for low resistance value: 200x input
#define AIN_10x_CAL			0				//from manual calibration at AIN_10x setting: typically 0
#define AIN_40x_CAL			0				//from manual calibration at AIN_40x setting: typically 0
#define AIN_200x_CAL		4				//from manual calibration at AIN_200x setting: typically 8
#define THRSH_10x			200				//threshold for 10x gain. below which, switch to 40x gain. must be less than 1024/(40/10) = 256
#define THRSH_40x			150				//threshold for 40x gain. below which, switch to 200x gain. must be less than 1024/(200/40) = 200
#define uOHM_CNT			32				//number of oversampling

//digit definitions - for Arduino Leonardo / ATmega32U4 configuration
//LED		Arduino		AVR
//SEGE		13			PC7
//SEGD		12			PD6
//SEGDP	 	11			PB7
//SEGC		4		 	PD4
//SEGG		3		 	PD0
//DIG4		2		 	PD1

//DIG1		10			PB6
//SEGA		9		 	PB5
//SEGF		8		 	PB4
//DIG2		7		 	PE6
//DIG3		6		 	PD7
//SEGB		5		 	PC6

//
#define DIG1_PORT	 	PORTB
#define DIG1_DDR		DDRB
#define DIG1			(1<<6)

#define DIG2_PORT	 	PORTE
#define DIG2_DDR		DDRE
#define DIG2			(1<<6)

#define DIG3_PORT	 	PORTD
#define DIG3_DDR		DDRD
#define DIG3			(1<<7)

#define DIG4_PORT	 	PORTD
#define DIG4_DDR		DDRD
#define DIG4			(1<<1)

//segment definitions
//map 4-digit led display to pic16f1936 direction
#define SEGA_PORT	 	PORTB
#define SEGA_DDR		DDRB
#define SEGA			(1<<5)

#define SEGB_PORT	 	PORTC
#define SEGB_DDR		DDRC
#define SEGB			(1<<6)

#define SEGC_PORT	 	PORTD
#define SEGC_DDR		DDRD
#define SEGC			(1<<4)

#define SEGD_PORT	 	PORTD
#define SEGD_DDR		DDRD
#define SEGD			(1<<6)

#define SEGE_PORT	 	PORTC
#define SEGE_DDR		DDRC
#define SEGE			(1<<7)

#define SEGF_PORT	 	PORTB
#define SEGF_DDR		DDRB
#define SEGF			(1<<4)

#define SEGG_PORT	 	PORTD
#define SEGG_DDR		DDRD
#define SEGG			(1<<0)

#define SEGDP_PORT		PORTB
#define SEGDP_DDR	 	DDRB
#define SEGDP		 	(1<<7)
//end hardware configuration

//global defines
#define VREF				2560000ul		//ADC vref, in uv
//#define ADC2uOHM(adc)		((adc) * (VREF / 512 / 100 / 5) * (RuOHM / 10) / 1)			//convert ADC(1x) to uOHM
#define ADC2uOHM_10x(adc)	((adc) * (VREF / 512 / 100 / 5) * (RuOHM_10x  / 10) / 10)	//512 for differential channels, 1024 for single ended channels
#define ADC2uOHM_40x(adc)	((adc) * (VREF / 512 / 100 / 5) * (RuOHM_40x / 10) / 40)	//512 for differential channels, 1024 for single ended channels
#define ADC2uOHM_200x(adc)	((adc) * (VREF / 512 / 100 / 5) * (RuOHM_200x / 10) / 200)	//512 for differential channels, 1024 for single ended channels
#define uOHM_EC(uOHM)		((uOHM) + ((uOHM) / ((RuOHM) / 1000) * ((uOHM) / 1000) / 1000))		//error correction for uOHM


//=====================================gpio.h==================================
#ifndef _GPIO_H_
#define _GPIO_H_

//gpio header file for lpc21xx
#if defined(__GNUC__)
#include <stdint.h>						 //uint8_t ... types
#include <avr/io.h>						 //we use avr
#include <avr/interrupt.h>
#elif defined(__ICCAVR__)
//#if defined(__EWAVR__)						//alternatively
#include <ioavr.h>							//we use iar avr
#include <stdint.h>						 //have to use dlib (General Options->Library Configuration->Normal DLIB
#include <intrinsics.h>					 //we use _nop_(). IAR AVR only
#else
#warning "no general header files included" //need to put something here if not covered by gcc/iar avr
#endif

//port definitions
#define IO_SET(port, bits)	port |= (bits)		//set bits on port
#define IO_CLR(port, bits)	port &=~(bits)		//clear bits on port
#define IO_FLP(port, bits)	port ^= (bits)		//flip bits on port
#define IO_GET(port, bits)	((port) & (bits)) //return bits on port
#define IO_IN(ddr, bits)	ddr &=~(bits)	 //set bits as input
#define IO_OUT(ddr, bits) ddr |= (bits)	 //ddr |= (bits)	 //set bits as output

//gpio definitions
#define GIO_SET(port, bits) IO_SET(port->PORT,bits)	 //set bits on port
#define GIO_CLR(port, bits) IO_CLR(port->PORT,bits)	 //clear bits on port
#define GIO_FLP(port, bits) IO_FLP(port->PORT,bits)	 //flip bits on port
#define GIO_GET(port, bits) IO_GET(port->PIN, bits)	 //return bits on port
#define GIO_IN(port, bits)	IO_IN(port->DDR, bits)		//set bits as input
#define GIO_OUT(port, bits) IO_OUT(port->DDR, bits)	 //ddr |= (bits)	 //set bits as output

#define NOP()			 asm("nop")			//nop
#define NOP2()				{NOP(); NOP();}
#define NOP4()				{NOP2(); NOP2();}
#define NOP8()				{NOP4(); NOP4();}
#define NOP16()			 {NOP8(); NOP8();}
#define NOP24()			 {NOP16(); NOP8();}
#define NOP32()			 {NOP16(); NOP16();}
#define NOP40()			 {NOP32(); NOP8();}
#define NOP64()			 {NOP32(); NOP32();}

#if defined(__ICCAVR__)						 //for ewavr
//#if defined(__EWAVR__)							//alternatively
#define ei()				__enable_interrupt()
#define di()			__disable_interrupt()
#endif

#if defined(__GNUC__)						 //for gcc avr
#include <avr/interrupt.h>					//sei/cli defined in interrupt.h
#define ei()			sei()			 //enable interrupt
#define di()			cli()			 //disable interrupt
#endif

#ifndef F_CPU
#define F_CPU		 1000000ul		 //cpu runs at 1Mhz
#endif

//void (*mcu_reset)(void) = 0x0000;			 //jump to 0x0000 -> software reset
void mcu_init(void);

//simple multiples
#define x1(val)			 (val)							 //multiply val by 1
#define x2(val)			 (((val) << 1))						//multiply val by 2
#define x3(val)			 (x2(val) + (val))				 //multiply val by 3
#define x4(val)			 (((val) << 2))						//multiply val by 4
#define x5(val)			 (x4(val) + (val))				 //multiply val by 5
#define x6(val)			 (x4(val) + x2(val))				 //multiply val by 6
#define x7(val)			 (x6(val) + (val))				 //multiply val by 7
#define x8(val)			 ((val) << 3)						//multiply val by 8
#define x9(val)			 (x8(val) + (val))				 //multiply val by 9

//multiples of 10s
#define x10(val)			(x8(val) + x2(val))				 //multiply val by 10
#define x100(val)		 (x10(x10(val)))					 //multiply val by 100
#define x1000(val)			(x100(x10(val)))					//multiply val by 1000
#define x1k(val)			x1000(val)							//multiply val by 1000
#define x10k(val)		 (x100(x100(val)))				 //multiply val by 10000

#define x20(val)			(x2(x10(val)))
#define x30(val)			(x3(x10(val)))
#define x40(val)			(x4(x10(val)))
#define x50(val)			(x5(x10(val)))
#define x60(val)			(x6(x10(val)))
#define x70(val)			(x7(x10(val)))
#define x80(val)			(x8(x10(val)))
#define x90(val)			(x9(x10(val)))

//multiples of 100s
#define x200(val)		 (x2(x100(val)))
#define x300(val)		 (x3(x100(val)))
#define x400(val)		 (x4(x100(val)))
#define x500(val)		 (x5(x100(val)))
#define x600(val)		 (x6(x100(val)))
#define x700(val)		 (x7(x100(val)))
#define x800(val)		 (x8(x100(val)))
#define x900(val)		 (x9(x100(val)))

//custom definitions
#define x34(val)			(x30(val) + x4(val))				//multiply val by 34
#define x97(val)			(x90(val) + x7(val))				//multiply val by 97x

//arduino related macros
//arduino pin functions
#define AIO_SET(pin)		digitalWrite(pin, HIGH)
#define AIO_CLR(pin)		digitalWrite(pin, LOW)
#define AIO_FLP(pin)		digitalWrite(pin, !digitalRead(pin))

#define AIO_OUT(pin)		pinMode(pin, OUTPUT)
#define AIO_IN(pin)		 pinMode(pin, INPUT)

//gpio definitions
typedef struct {
	volatile unsigned char PIN; //input data register
	volatile unsigned char DDR; //data direction register
	volatile unsigned char PORT;	//output data register
} GPIO_TypeDef;			 //gpio data registers

#define GPIOA			 ((GPIO_TypeDef *) &PINA)
#define GPIOB			 ((GPIO_TypeDef *) &PINB)
#define GPIOC			 ((GPIO_TypeDef *) &PINC)
#define GPIOD			 ((GPIO_TypeDef *) &PIND)
#define GPIOE			 ((GPIO_TypeDef *) &PINE)
#define GPIOF			 ((GPIO_TypeDef *) &PINF)
#define GPIOG			 ((GPIO_TypeDef *) &PING)
#define GPIOH			 ((GPIO_TypeDef *) &PINH)

#endif //gpio_h_

//==================================gpio.c=================================================
#include "gpio.h"

//reset the mcu
void mcu_init(void) {						//reset the mcu
}

//===================================tmr4oc.h=============================================
#ifndef _TMR4OC_H
#define _TMR4OC_H

#include "gpio.h"

//timer2 routines
//using output compare as additional timer
//header file to use tmr=2
#define TMR4PS_1x			1
#define TMR4PS_2x		 2
#define TMR4PS_4x		 3
#define TMR4PS_8x		 4
#define TMR4PS_16x			5
#define TMR4PS_32x			6
#define TMR4PS_64x			7
#define TMR4PS_128x		 8
#define TMR4PS_256x		 9
#define TMR4PS_512x		 10
#define TMR4PS_1024x		11
#define TMR4PS_2048x		12
#define TMR4PS_4096x		13
#define TMR4PS_8192x		14
#define TMR4PS_16384x	 15
#define TMR4PS_MASK		 TMR4PS_16384x

//tmr period settings
#define TMR_ms				(F_CPU / 1000)				//1ms period - minimum period
#define TMR_1ms			 (TMR_ms * 1)				//1ms
#define TMR_2ms			 (TMR_ms * 2)				//2ms period
#define TMR_5ms			 (TMR_ms * 5)				//5ms period
#define TMR_10ms			(TMR_ms * 10)			 //10ms
#define TMR_20ms			(TMR_ms * 20)			 //20ms period
#define TMR_50ms			(TMR_ms * 50)			 //50ms period
#define TMR_100ms		 (TMR_ms * 100)				//100ms
#define TMR_200ms		 (TMR_ms * 200)				//200ms period
#define TMR_500ms		 (TMR_ms * 500)				//500ms period
#define TMR_1000ms			(TMR_ms * 1000)			 //1000ms
#define TMR_2000ms			(TMR_ms * 2000)			 //2000ms period
#define TMR_5000ms			(TMR_ms * 5000)			 //5000ms period
#define TMR_10000ms		 (TMR_ms * 10000)			//10000ms

//reset the tmr
void tmr4_init(unsigned char prescaler);
void tmr4_act(void (*isr_ptr)(void));


void tmr4a_setpr(unsigned char pr); //set timer4 output compare period
void tmr4a_act(void (*isr_ptr)(void));	//install timer4 output compare interrupt handler
void tmr4b_setpr(unsigned char pr); //set timer4 output compare period
void tmr4b_act(void (*isr_ptr)(void));	//install timer4 output compare interrupt handler
void tmr4d_setpr(unsigned char pr); //set timer4 output compare period
void tmr4d_act(void (*isr_ptr)(void));	//install timer4 output compare interrupt handler

#endif

//========================================tmr4oc.c========================================
//#include "tmr4oc.h"							//tmr2 header file

//empty handler
static void /*_tmr3_*/empty_handler(void) {
	//default tmr handler
}

static void (* /*_tmr3*/_isr_ptrovf)(void) = empty_handler; //tmr2_ptr pointing to empty_handler by default
static void (* /*_tmr3*/_isr_ptroca)(void) = empty_handler; //tmr2_ptr pointing to empty_handler by default
static void (* /*_tmr3*/_isr_ptrocb)(void) = empty_handler; //tmr2_ptr pointing to empty_handler by default
static void (* /*_tmr3*/_isr_ptrocd)(void) = empty_handler; //tmr2_ptr pointing to empty_handler by default
static unsigned char _tmroca_pr = 0xff;
static unsigned char _tmrocb_pr = 0xff;
static unsigned char _tmrocd_pr = 0xff;

//tmr3 overflow isr
ISR(TIMER4_OVF_vect) {
	//clera the flag - done automatically
	//OCR2 += _tmr2oc_pr;						 //advance to the next match point
	//TCCR2 |= (1<<FOC2);						 //1->force a compare
	/*_tmr3*/_isr_ptrovf();					 //execute the handler
}

//tmr4 compare isr
ISR(TIMER4_COMPA_vect) {
	//clera the flag - done automatically
	OCR4A += _tmroca_pr;							//advance to the next match point
	//TCCR2 |= (1<<FOC2);						 //1->force a compare
	/*_tmr3*/_isr_ptroca();					 //execute the handler
}

//tmr4 compare isr
ISR(TIMER4_COMPB_vect) {
	//clera the flag - done automatically
	OCR4B += _tmrocb_pr;							//advance to the next match point
	//TCCR2 |= (1<<FOC2);						 //1->force a compare
	/*_tmr3*/_isr_ptrocb();					 //execute the handler
}

//tmr4 compare isr
ISR(TIMER4_COMPD_vect) {
	//clera the flag - done automatically
	OCR4D += _tmrocd_pr;							//advance to the next match point
	//TCCR2 |= (1<<FOC2);						 //1->force a compare
	/*_tmr3*/_isr_ptrocd();					 //execute the handler
}

void tmr4_init(unsigned char prescaler) {
	_isr_ptrovf = _isr_ptroca = _isr_ptrocb = _isr_ptrocd = empty_handler;
	_tmroca_pr = _tmrocb_pr = _tmrocd_pr = 0xff;

	//configure the timer
	//stop the timer
	TCCR4B = (TCCR4B & ~TMR4PS_MASK) | (0x00 & TMR4PS_MASK);
	TCCR4A =	(0 << COM4A0) |		 //0->normal port operation
						(0 << COM4B0) |		 //0->normal port operation
						(0 << FOC4A) |			//1->force a compare
						(0 << FOC4B) |			//force a compare
						(0 << PWM4A) |			//1->pwm enabled, 0->pwm disabled
						(0 << PWM4B) |			//1->pwm enabled, 0->pwm disabled
						0x00;
	TCCR4B =	(0 << PWM4X) |			//1->enable pwmm inversion mode, 0->disable pwm inversion mode
						(0 << PSR4) |			 //1->reset prescaler
						(0 << DTPS40) |		 //0->dead time programming: 0->1x, 1->2x, 2->4x, 3->8x
						(0 << CS40) |			 //0->disable the timer, 1..15->enable the clock
						0x00;
	TCCR4C =	(0 << COM4A0S) |			//0..3->comparator output mode
						(0 << COM4B0S) |			//0..3->comparator output mode
						(0 << COM4D0) |		 //0..3->comparator output mode
						(0 << FOC4D) |			//1->force a compare
						(0 << PWM4D) |			//1->enable pwm, 0->disable pwm
						0x00;
	TCCR4D =	(0 << FPIE4) |			//1->enable fault protection interrupt
						(0 << FPEN4) |			//1->enable fault protection mode
						(0 << FPNC4) |			//1->enable fault protection noise canceler
						(0 << FPES4) |			//1->fault protection edge select
						(0 << FPAC4) |			//1->enablefault protection analog comparator
						(0 << FPF4) |			 //flag for fault protection interrupt
						(0 << WGM41) | (0 << WGM40) | //wave form mode bits
						0x00;
	TCCR4E =	(0 << TLOCK4) |		 //register update lock;						 //reset value
						(0 << ENHC4) |			//1->enable enhanced mode (1 extra bit
						(0 << OC4OE5) |		 //output compare override enable bit
						(0 << OC4OE4) |		 //output compare override enable bit
						(0 << OC4OE3) |		 //output compare override enable bit
						(0 << OC4OE2) |		 //output compare override enable bit
						(0 << OC4OE1) |		 //output compare override enable bit
						(0 << OC4OE0) |		 //output compare override enable bit
						0x00;
	//overflow top at 0xff
	TC4H = TCNT4 = 0;							 //reset the timer / counter
	//OCR2 = 0;							 //reset OCR2
	//clera overflow interrupt flag, disable overflow interrupt
	TIFR4	|= ((1 << TOV4) | (1 << OCF4A) | (1 << OCF4B) | (1 << OCF4D)); //1->clera overflow interrupt
	TIMSK4 &= ~((1 << TOIE4) | (1 << OCIE4A) | (1 << OCIE4B) | (1 << OCIE4D)); //0->disable overflow interrupt, 1->enable overflow interrupt
	//clear output compare interrupt flag, disable output compare interrupt
	//TIMSK &=~(1<<OCIE2);					//0->disable output compare, 1->enable output compare
	//TIFR	|= (1<<OCF2);					 //1->clear output compare flag

	//start the timer
	TCCR4B = (TCCR4B & ~TMR4PS_MASK) | (prescaler & TMR4PS_MASK);
	//timer2 now running
}

//install timer2 output compare interrupt handler
void tmr4_act(void (*isr_ptr)(void)) {
	_isr_ptrovf = isr_ptr;					//install user handler
	//clear output compare interrupt flag, enable output compare interrupt
	TIFR4	|= (1 << TOV4);					//1->clear output compare flag
	TIMSK4 |= (1 << TOIE4);			 //0->disable output compare, 1->enable output compare
}

//set timer2 output compare period
void tmr4a_setpr(unsigned char pr) {
	//clear output compare interrupt flag, disable output compare interrupt
	//TIFR	|= (1<<OCF2);					 //1->clear output compare flag
	//TIMSK &=~(1<<OCIE2);					//0->disable output compare, 1->enable output compare
	_tmroca_pr = pr - 1;
	OCR4A = TCNT4 + _tmroca_pr;
}

//install timer2 output compare interrupt handler
void tmr4a_act(void (*isr_ptr)(void)) {
	_isr_ptroca = isr_ptr;					//install user handler
	//clear output compare interrupt flag, enable output compare interrupt
	TIFR4	|= (1 << OCF4A);				 //1->clear output compare flag
	TIMSK4 |= (1 << OCIE4A);				//0->disable output compare, 1->enable output compare
}

//set timer2 output compare period
void tmr4b_setpr(unsigned char pr) {
	//clear output compare interrupt flag, disable output compare interrupt
	//TIFR	|= (1<<OCF2);					 //1->clear output compare flag
	//TIMSK &=~(1<<OCIE2);					//0->disable output compare, 1->enable output compare
	_tmrocb_pr = pr - 1;
	OCR4B = TCNT4 + _tmrocb_pr;
}

//install timer2 output compare interrupt handler
void tmr4b_act(void (*isr_ptr)(void)) {
	_isr_ptrocb = isr_ptr;					//install user handler
	//clear output compare interrupt flag, enable output compare interrupt
	TIFR4	|= (1 << OCF4B);				 //1->clear output compare flag
	TIMSK4 |= (1 << OCIE4B);				//0->disable output compare, 1->enable output compare
}

//set timer2 output compare period
void tmr4d_setpr(unsigned char pr) {
	//clear output compare interrupt flag, disable output compare interrupt
	//TIFR	|= (1<<OCF2);					 //1->clear output compare flag
	//TIMSK &=~(1<<OCIE2);					//0->disable output compare, 1->enable output compare
	_tmrocd_pr = pr - 1;
	OCR4D = TCNT4 + _tmrocd_pr;
}

//install timer2 output compare interrupt handler
void tmr4d_act(void (*isr_ptr)(void)) {
	_isr_ptrocd = isr_ptr;					//install user handler
	//clear output compare interrupt flag, enable output compare interrupt
	TIFR4	|= (1 << OCF4D);				 //1->clear output compare flag
	TIMSK4 |= (1 << OCIE4D);				//0->disable output compare, 1->enable output compare
}

//==========================led4_pins.h=============================================
/*
	 File:	 led4_bits.h

	 Created on April 15, 2014, 9:27 PM
*/

#ifndef LED4_PINS_H
#define	LED4_PINS_H

#include "gpio.h"

//hardware configuration
#define DISPLAY_MAX		 9999							//maximum value to be displayed - modify for your application

//uncomment if display is CC (common cathode)
//#define DISPLAY_IS_CA									//CC=common cathod, and CA=common anode
//end hardware configuration

//global defines

//global variables
extern unsigned char lRAM[];							//display buffer, to be provided by the user. 4 digit long
extern const unsigned char ledfont_num[];							 //led font for numerical values, '0'..'f', including blanks
extern const unsigned char ledfont_alpha[];						 //led font for alphabeta values, 'a'..'z', including blanks

//initialize the pins
void led_init(void);

//display the ledram
void led_display(void);

#endif	/* LED4_PINS_H */


//======================================led4_pin.c===================================================
//#include "led4_pins.h"					//we use 4-digit 7-segment leds

//hardware configuration
//#define PORTB		 PORTB
//#define PORTA		 PORTA
#if 0
//digit definitions - LED on lowest 6 pins
#define DIG1_PORT	 PORTC
#define DIG1_DDR		DDRC
#define DIG1			(1<<4)

#define DIG2_PORT	 PORTC
#define DIG2_DDR		DDRC
#define DIG2			(1<<1)

#define DIG3_PORT	 PORTC
#define DIG3_DDR		DDRC
#define DIG3			(1<<0)

#define DIG4_PORT	 PORTD
#define DIG4_DDR		DDRD
#define DIG4			(1<<6)

//segment definitions
//map 4-digit led display to pic16f1936 direction
#define SEGA_PORT	 PORTC
#define SEGA_DDR		DDRC
#define SEGA			(1<<3)

#define SEGB_PORT	 PORTD
#define SEGB_DDR		DDRD
#define SEGB			(1<<7)

#define SEGC_PORT	 PORTD
#define SEGC_DDR		DDRD
#define SEGC			(1<<4)

#define SEGD_PORT	 PORTD
#define SEGD_DDR		DDRD
#define SEGD			(1<<2)

#define SEGE_PORT	 PORTD
#define SEGE_DDR		DDRD
#define SEGE			(1<<1)

#define SEGF_PORT	 PORTC
#define SEGF_DDR		DDRC
#define SEGF			(1<<2)

#define SEGG_PORT	 PORTD
#define SEGG_DDR		DDRD
#define SEGG			(1<<5)

#define SEGDP_PORT		PORTD
#define SEGDP_DDR	 DDRD
#define SEGDP		 (1<<3)

#else
#endif
//end hardware configuration

#if defined(DISPLAY_IS_CA)			 //DISPLAY_MODE==CA	//for common anode displays
//digit control - active high (Common Anode) or active low (Common Cathode)
#define DIG_ON(port, pins)		IO_SET(port, pins)			//turn on a digit
#define DIG_OFF(port, pins)	 IO_CLR(port, pins)			//turn off a digit

//segment control - active low (Common Anode) or active high (Common Cathode)
#define SEG_ON(port, pins)		IO_CLR(port, pins)			//turn on a segment
#define SEG_OFF(port, pins)	 IO_SET(port, pins)			//turn off a segment

#else				 //for common cathode displays
//digit control - active high (Common Anode) or active low (Common Cathode)
#define DIG_ON(port, pins)		IO_CLR(port, pins)			//turn on a digit
#define DIG_OFF(port, pins)	 IO_SET(port, pins)			//turn off a digit

//segment control - active low (Common Anode) or active high (Common Cathode)
#define SEG_ON(port, pins)		IO_SET(port, pins)			//turn on a segment
#define SEG_OFF(port, pins)	 IO_CLR(port, pins)			//turn off a segment
#endif

//global defines

//global variables
unsigned char lRAM[4];				//led display buffer
//led font.
//SEGDP = 0x80
//SEGG	 = 0x40
//SEGF	 = 0x20
//SEGE	 = 0x10
//SEGD	 = 0x08
//SEGC	 = 0x04
//SEGB	 = 0x02
//SEGA	 = 0x01
//led font for numerical display '0'..'9''a'..'f', active high
const unsigned char ledfont_num[] = { //led font, for common anode
	0x3f,							 //'0'
	0x06,							 //'1'
	0x5b,							 //'2'
	0x4f,							 //'3'
	0x66,							 //'4'
	0x6d,							 //'5'
	0x7d,							 //'6'
	0x07,							 //'7'
	0x7f,							 //'8'
	0x6f,							 //'9'
	0x5f,							 //'a'
	0x7c,							 //'b'
	0x58,							 //'c'
	0x5e,							 //'d'
	0x79,							 //'e'
	0x71,							 //'f'
	0x00								//' ' blank
};
//led font for alphabetic display 'a'..'z'
const unsigned char ledfont_alpha[] = { //led font, for common anode
	0x5f,							 //'a'
	0x7c,							 //'b'
	0x58,							 //'c'
	0x5e,							 //'d'
	0x79,							 //'e'
	0x71,							 //'f'
	0x6f,							 //'g'
	0x07,							 //'h'
	0x74,							 //'i'
	0x0e,							 //'j'
	0x00,							 //'k'
	0x38,							 //'l'
	0x00,							 //'m'
	0x54,							 //'n'
	0x5c,							 //'o'
	0x73,							 //'p'
	0x67,							 //'q'
	0x77,							 //'r'
	0x6d,							 //'s'
	0x00,							 //'t'
	0x1c,							 //'u'
	0x00,							 //'v'
	0x00,							 //'w'
	0x00,							 //'x'
	0x6e,							 //'y'
	0x00,							 //'z'
	0x00								//' ' blank
};

//initialize the pins
void led_init(void) {
	//turn off the digits send set pins to output
	DIG_OFF(DIG1_PORT, DIG1); IO_OUT(DIG1_DDR, DIG1);
	DIG_OFF(DIG2_PORT, DIG2); IO_OUT(DIG2_DDR, DIG2);
	DIG_OFF(DIG3_PORT, DIG3); IO_OUT(DIG3_DDR, DIG3);
	DIG_OFF(DIG4_PORT, DIG4); IO_OUT(DIG4_DDR, DIG4);

	//turn off the segments
	SEG_OFF(SEGA_PORT, SEGA); IO_OUT(SEGA_DDR, SEGA);
	SEG_OFF(SEGB_PORT, SEGB); IO_OUT(SEGB_DDR, SEGB);
	SEG_OFF(SEGC_PORT, SEGC); IO_OUT(SEGC_DDR, SEGC);
	SEG_OFF(SEGD_PORT, SEGD); IO_OUT(SEGD_DDR, SEGD);
	SEG_OFF(SEGE_PORT, SEGE); IO_OUT(SEGE_DDR, SEGE);
	SEG_OFF(SEGF_PORT, SEGF); IO_OUT(SEGF_DDR, SEGF);
	SEG_OFF(SEGG_PORT, SEGG); IO_OUT(SEGG_DDR, SEGG);
	SEG_OFF(SEGDP_PORT, SEGDP); IO_OUT(SEGDP_DDR, SEGDP);

	//set all pins to output
	//put your code here
}

//display the ledram
void led_display(void) {
	static unsigned char dig = 0; //current digit
	unsigned char tmp;

	//turn off the digits
	DIG_OFF(DIG1_PORT, DIG1);
	DIG_OFF(DIG2_PORT, DIG2);
	DIG_OFF(DIG3_PORT, DIG3);
	DIG_OFF(DIG4_PORT, DIG4);

	//if user filled lRAM with numbers only
	//tmp=ledfont_num[lRAM[dig]];				 //retrieve font / segment info from the display buffer
	//if user filled lRAM with segment information
	tmp = lRAM[dig];								//retrieve font / segment info from the display buffer
	//turn on/off the segments
	if (tmp & 0x01) SEG_ON(SEGA_PORT, SEGA); else SEG_OFF(SEGA_PORT, SEGA);
	if (tmp & 0x02) SEG_ON(SEGB_PORT, SEGB); else SEG_OFF(SEGB_PORT, SEGB);
	if (tmp & 0x04) SEG_ON(SEGC_PORT, SEGC); else SEG_OFF(SEGC_PORT, SEGC);
	if (tmp & 0x08) SEG_ON(SEGD_PORT, SEGD); else SEG_OFF(SEGD_PORT, SEGD);
	if (tmp & 0x10) SEG_ON(SEGE_PORT, SEGE); else SEG_OFF(SEGE_PORT, SEGE);
	if (tmp & 0x20) SEG_ON(SEGF_PORT, SEGF); else SEG_OFF(SEGF_PORT, SEGF);
	if (tmp & 0x40) SEG_ON(SEGG_PORT, SEGG); else SEG_OFF(SEGG_PORT, SEGG);
	if (tmp & 0x80) SEG_ON(SEGDP_PORT, SEGDP); else SEG_OFF(SEGDP_PORT, SEGDP);

	//turn on the digit and advance to the next digit
	switch (dig) {
		case 0: DIG_ON(DIG1_PORT, DIG1); dig = 1; break;
		case 1: DIG_ON(DIG2_PORT, DIG2); dig = 2; break;
		case 2: DIG_ON(DIG3_PORT, DIG3); dig = 3; break;
		case 3: DIG_ON(DIG4_PORT, DIG4); dig = 0; break;
	}
}

//================================================adc.h=============================================
#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED

#include "gpio.h"

//hardware configuration
//end hardware configuration

//global defines
//adc channels
//6 bits mux: mux4..0 in ADMUX, mux5 in ADCSRB
#define AIN0			0b000000
#define AIN1			0b000001
#define AIN4			0b000100
#define AIN5			0b000101
#define AIN6			0b000110
#define AIN7			0b000111
#define AIN1_0x10		0b001001
#define AIN1_0x200		0b001011
#define AIN0_1			0b010000
#define AIN4_1			0b010100
#define AIN5_1			0b010101
#define AIN6_1			0b010110
#define AIN7_1			0b010111
#define AINVBG			0b011110		//1.1Vbg
#define AINGND			0b011111
#define AIN8			0b100000
#define AIN9			0b100001
#define AIN10			0b100010
#define AIN11			0b100011
#define AIN12			0b100100
#define AIN13			0b100101
#define AIN1_0x40		0b100110
#define AINTEMP			0b100111		//temperature sensor
#define AIN4_0x10		0b101000
#define AIN5_0x10		0b101001
#define AIN6_0x10		0b101010
#define AIN7_0x10		0b101011
#define AIN4_1x10		0b101100
#define AIN5_1x10		0b101101
#define AIN6_1x10		0b101110
#define AIN7_1x10		0b101111
#define AIN4_0x40		0b110000
#define AIN5_0x40		0b110001
#define AIN6_0x40		0b110010
#define AIN7_0x40		0b110011
#define AIN4_1x40		0b110100
#define AIN5_1x40		0b110101
#define AIN6_1x40		0b110110
#define AIN7_1x40		0b110111
#define AIN4_0x200		0b111000
#define AIN5_0x200		0b111001
#define AIN6_0x200		0b111010
#define AIN7_0x200		0b111011
#define AIN4_1x200		0b111100
#define AIN5_1x200		0b111101
#define AIN6_1x200		0b111110
#define AIN7_1x200		0b111111


//initialize the adc
//using internal Vref
void adc_init(void);

//read adc
int16_t adc_read(uint8_t ain);

#endif // ADC_H_INCLUDED

//======================================================adc.c==============================================
//#include "adc.h"							//we use adc

//hardware configuration
//end hardware configuration

//global defines

//global variables

//initialize the adc
//using internal Vref
void adc_init(void) {
	ADCSRA =	(1 << ADEN) |			 //1->turn on the adc, 0->turn off the adc
				(0 << ADSC) |			 //0->don't star the adc, 1->start the adc
				(0 << ADATE) |			//1->enable auto trigger, 0->disable auto trigger
				(0 << ADIF) |			 //0->adc interrupt flag not set, 1->adc interrupt flag set
				(0 << ADIE) |			 //1->enable adc interrupt, 0->disable adc interrupt
				(7 << ADPS0) |			//adc prescaler. 0..7: 2x .. 128x
				0x00;
	ADMUX =	 	(ADMUX & 0x1f) |			//clear highest 3 bits (REFS1/REFS0/ADLAR)
				(3 << REFS0) |			//select reference: 0->AREF, 1->AVCC, 2->reserved, 3->internal 2.56v Vref
				(0 << ADLAR) |			//1->left adjust adc result, 0->right adjust adc result
				0x00;
	ADCSRB =	(0 << ADHSM) |			//1->adc high speed mode enabled, 0->disable high speed mode
				(0 << MUX5) |			 //highest bit of MUX
				(0 << ADTS0) |			//0..8: 0 free running mode, ...
				0x00;
}

//read adc
int16_t adc_read(uint8_t ain) {
	int16_t tmp;
	//set mux4..0
	ADMUX =	 	(ADMUX &~0x1f) |		 //clear the MUX4..0 bits
				(ain & 0x1f);			 //set the channel
	//set mux5
	ADCSRB =	(ADCSRB &~(1 << MUX5)) | (ain & (1 << MUX5)); //set mux5
	ADCSRA |=	(1 << ADSC);				//start the conversion
	while (ADCSRA & (1 << ADSC)) continue; //wait for conversion to finish
	//per datasheet, the following read orders must be maintained
	tmp = ADCL;							 //must read adcl first
	tmp |= ADCH << 8;					 //then read ADCH
	if (tmp & (1<<9)) {tmp |=~0x03ff; /*tmp = -tmp;*/}	//for negative values
	return tmp;						//bipolar adc results
}

//start of main loop / user code

//global variables
uint32_t uOHM = 0;

//flip ledr - for debugging
void ledr_flp(void) {
	IO_FLP(LEDR_PORT, LEDR);
}

//read the uOHM for osCNT number of times
//return the average
int16_t uOHM_read(uint8_t ain, uint8_t osCNT) {
	int16_t tmp;
	uint8_t cnt;
	
	//read at 10x
	adc_read(ain);				//dummy read - ignore the result from the 1st adc
	for (tmp=cnt=0; cnt<osCNT; cnt++) {
		tmp += adc_read(ain);	//oversamples ain
	}
	tmp = tmp / osCNT;			//calculate the average
	if (tmp < 0) tmp = 0;		//don't need negative values
	switch (ain) {
	case AIN_200x: tmp = (tmp > AIN_200x_CAL)?(tmp - AIN_200x_CAL):0; break;
	case AIN_40x:  tmp = (tmp > AIN_40x_CAL )?(tmp - AIN_40x_CAL ):0; break;
	case AIN_10x:  tmp = (tmp > AIN_10x_CAL )?(tmp - AIN_10x_CAL ):0; break;
	}
	return tmp;					//return the average
	
}

//apply exponential smoothing to uOHM
uint32_t uOHM_avg(uint32_t uOHM) {
	static uint32_t sum=0, avg=0;
	
	sum += (int32_t) (uOHM - avg);	//update the sum
	avg = sum / uOHM_AVGN;			//update the avearge
	return avg;					//return the average
}

//run only once
void setup(void) {
	//mcu_init();
	//pinMode(13, OUTPUT);	//IO_OUT(DDRC, 1<<7);
	//pinMode(12, OUTPUT); digitalWrite(12, LOW);
	IO_OUT(LEDR_DDR, LEDR);
	//IO_CLR(PORTD, 1 << 6); IO_OUT(DDRD, 1 << 6);

	//initialize the led display
	led_init();

	//install timer and update the led display in the background
	tmr4_init(TMR4PS_256x);			 			//display update frequency = F_CPU / 256 / prescaler = 244Hz per digit, or 244Hz/4=60Hz per frame
	tmr4_act(led_display);

	//reset the adc
	adc_init();									//prescaler = 128, manual conversion, internal 2.56v reference

#if defined(USE_UART)
	//initialize serial
	Serial.begin(9600);
#endif
	
	ei();										//enable interrupts
}

//run repeatedly
void loop(void) {
	static uint32_t t;							//for timing measurement
	uint32_t tmp;			 					//temp variable for display
	uint8_t dp;									//position of the decimal point

	//================measuring uOHM===============
	//measure uOHM by auto-ranging the ADC
	//t = micros();
	//auto ranging - start conversion at the lowest gain / highest resistive range
	//if the values returned are too small, increase the gain / lower the measurement range
	tmp = uOHM_read(AIN_10x, uOHM_CNT); uOHM = ADC2uOHM_10x(tmp);	//120us * uOHM_CNT approximately
	if (tmp < THRSH_10x) {tmp = uOHM_read(AIN_40x , uOHM_CNT); uOHM = ADC2uOHM_40x(tmp);}
	if (tmp < THRSH_40x) {tmp = uOHM_read(AIN_200x, uOHM_CNT); uOHM = ADC2uOHM_200x(tmp);}
	//t = micros() - t;
	//smooth out uOHM
	uOHM = uOHM_avg(uOHM);
	//tmp = t;									//display the value of t
	//uOHM = 9600000ul;							//for debug only
	//optional: error correction
	uOHM = uOHM_EC(uOHM);						//error correct for uOHM
	//===============end measuring uOHM===========
	
	//===============measuring temperature========
	//tmp = uOHM_read(AINTEMP, 16);				//adc value = 301-302
	//===============end measuring temperature====
	
	//===============calibrating offset
	//values shown on the LED display go into AINCAL_10x/MED/LOW macros
	//manual calibration process:
	//1. connect A3..A5 with a short (low resistance) wire.
	//2. start the calibration process by setting AINCAL_10x/MED/LOW macros to 0 initially
	//3. uncomment the corresponding ADC statement below for the corresponding macros to be calibrated
	//   ie. uncomment AIN_10x to calibrate AIN_10x_CAL
	//4. compile and upload the code to your chip
	//5. write down the value displayed on the LED display
	//6. record that value in the corresponding AIN_10x/MED/LOW_CAL macros
	//uOHM = uOHM_read(AIN_10x, uOHM_CNT);		//calibrating high setting: typically 0
	//uOHM = uOHM_read(AIN_40x, uOHM_CNT);		//calibrating medium setting: typically 0
	//uOHM = uOHM_read(AIN_200x, uOHM_CNT);		//calibrating low setting: typically 0 - 4 - 8, sometimes 12-16-20
	//===============end calibrating offset=======
	
	//display uOHM
	tmp = uOHM;									//display uOHM
	//display tmp
	//now uOHM max is 9999mOhm, with rounding up
			if (tmp > 999999ul)	{tmp = (tmp + 500) / 1000; dp = 3;}		//1000000..9999999uOHM
	else 	if (tmp >  99999ul)	{tmp = (tmp +  50) /  100; dp = 2;}		//100000..999999uOHM
	else 	if (tmp >   9999ul)	{tmp = (tmp +   5) /   10; dp = 1;}		//10000..99999uOHM
	else						{tmp = (tmp +   0) /    1; dp = 0;}		//0..9999uOHM
	//convert tmp to font
	//can be better optimized
	lRAM[3]= ledfont_num[tmp % 10]; tmp /= 10;
	lRAM[2]= ledfont_num[tmp % 10]; tmp /= 10;
	lRAM[1]= ledfont_num[tmp % 10]; tmp /= 10;
	lRAM[0]= ledfont_num[tmp % 10]; tmp /= 10;
	lRAM[dp] |= 0x80;							//add floating decimal point
	
#if defined(USE_UART)
	//serial output, if selected
	Serial.print("uOHM = "); Serial.print(uOHM); Serial.println("uR.");
#endif

	//ledr_flp();								//for debug only
	delay(uOHM_DLY);							//slow down the display update frequency
}
