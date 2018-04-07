//uno_hx711_milliohm.ino
//milliOhm meter with 4-digit 7-segment LED display
//built on ATmega328P/Arduino Uno or similar chips + HX711 24-bit ADC
//Revision history:
//v0.1 @ 3/31/2018: initial release: LED display only
//v0.2 @ 3/31/2018: 1) ADC implemented, using Channel A at 128x gain 2) auto ranging implemented; 3) AFE schematic shown
//v0.5 @ 4/ 6/2018: going live!
//
//connections:
//
//Analog Front End:
//
//    E+ of HX711
//     |
//     |
//     |
//     |
//     |
//    [ ] R1 (1K) 
//     |                                          ===============
//     |                                         |               |
//     |------------------------- CH A+ of HX711 |          GND  | ----> A1/PC1 of Uno
//     |                                         |               |
//     |                                         |          DOUT | ----> A2/PC2 of Uno
//    [ ] DUT (<10R)                             |     HX711     |
//     |                                         |          SCLK | ----> A3/PC3 of Uno
//     |                                         |               |
//     |-------[ ]--------------- CH A- of HX711 |          VCC  | ----> Vcc/5v
//     |     R5 (1K)                             |               |
//     |                                          ===============
//    [ ] R2 (1K)
//     |
//     |
//     |
//     |
//     |
//    E- of HX711

//LED display:
//DIG1..4 connects to Leonardo directly;
//SEGA..G + DP connects to Leonardo via 8x 1K resistors
//connections:
//LED		Arduino		AVR
//==========================
//SEGE		13			PB5
//SEGD		12			PB4
//SEGDP	 	11			PB3
//SEGC		4		 	PD4
//SEGG		3		 	PD3
//DIG4		2		 	PD2

//DIG1		10			PB2
//SEGA		9		 	PB1
//SEGF		8		 	PB0
//DIG2		7		 	PD7
//DIG3		6		 	PD6
//SEGB		5		 	PD5

//hardware configuration
#define LEDR_PORT		PORTB
#define LEDR_DDR		DDRB
#define LEDR			(1<<1)
#define LEDR_DLY		100

#define USE_UART							//comment out if UART isn't to be used
#define uOHM_AVGN				2			//exponential smoothing for uOHM readings
#define uOHM_DLY				100			//display delays for uOHM measurement, in ms
#define ADC_OFFSET				700			//ADC offset calibration: about 600-700

//HX711 connection
#define HX711GND_PORT			PORTC
#define HX711GND_DDR			DDRC
#define HX711GND				(1<<1)

#define HX711DOUT_PORT			PINC		//an input port
#define HX711DOUT_DDR			DDRC
#define HX711DOUT				(1<<2)

#define HX711SCK_PORT			PORTC
#define HX711SCK_DDR			DDRC
#define HX711SCK				(1<<3)

//=====led connections=================
#define DIG1_PORT		PORTB
#define DIG1_DDR		DDRB
#define DIG1			(1<<2)

#define DIG2_PORT	 	PORTD
#define DIG2_DDR		DDRD
#define DIG2			(1<<7)

#define DIG3_PORT	 	PORTD
#define DIG3_DDR		DDRD
#define DIG3			(1<<6)

#define DIG4_PORT	 	PORTD
#define DIG4_DDR		DDRD
#define DIG4			(1<<2)

//segment definitions
//map 4-digit led display to Arduino Uno direction
#define SEGA_PORT	 	PORTB
#define SEGA_DDR		DDRB
#define SEGA			(1<<1)

#define SEGB_PORT	 	PORTD
#define SEGB_DDR		DDRD
#define SEGB			(1<<5)

#define SEGC_PORT	 	PORTD
#define SEGC_DDR		DDRD
#define SEGC			(1<<4)

#define SEGD_PORT	 	PORTB
#define SEGD_DDR		DDRB
#define SEGD			(1<<4)

#define SEGE_PORT	 	PORTB
#define SEGE_DDR		DDRB
#define SEGE			(1<<5)

#define SEGF_PORT	 	PORTB
#define SEGF_DDR		DDRB
#define SEGF			(1<<0)

#define SEGG_PORT	 	PORTD
#define SEGG_DDR		DDRD
#define SEGG			(1<<3)

#define SEGDP_PORT		PORTB
#define SEGDP_DDR	 	DDRB
#define SEGDP		 	(1<<3)
//end hardware configuration

//global defines
#define VREF 				HX711_VREFmv		//HX711 reference voltage, in mv
#define RmOHM				(2000000ul)			//current limiting resistor, in mOHM
#define ADCHI2LO			100000ul				//ADC threshold above which, switch to ADCHI2uOHM()
#define ADCHI2uOHM(adc24)	(ADCLO2uOHM(((adc24) +  64) / 128) * 128)		//for adc24 >= 2048 (=ADCHI2LO). high end is about adc24=90,000,000/1OHM
#define ADCLO2uOHM(adc24)	((adc24) * (((RmOHM) + 128) / 256) / 128 * (1000 / 8) / 8192)		//for adc24 <= 2048 (=ADCHI2LO)
#define ADC2uOHM(adc24)		((double) (adc24) * (RmOHM) * 1000.0 / 128.0 / (1ul << 24))			//floating point conversion algorithm
#define uOHM_EC(uOHM)		((uOHM) + ((uOHM) / ((RmOHM) / 1000) * ((uOHM) / 1000) / 1000))		//error correction for uOHM

//global variables
uint32_t uOHM = 0;

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
#define NOP2()			{NOP(); NOP();}
#define NOP4()			{NOP2(); NOP2();}
#define NOP8()			{NOP4(); NOP4();}
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
#define F_CPU		 	16000000ul		 //cpu runs at 1Mhz
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
//#include "gpio.h"

//reset the mcu
void mcu_init(void) {						//reset the mcu
}

//==========================tmr2oc.h================================================
#ifndef _TMR2OC_H
#define _TMR2OC_H
//header file to use tmr1

//#include "gpio.h"

//prescaler
#define TMR2PS_NOCLK		0x00
#define TMR2PS_1x			0x01
#define TMR2PS_8x			0x02
#define TMR2PS_32x			0x03
#define TMR2PS_64x			0x04
#define TMR2PS_128x			0x05
#define TMR2PS_256x			0x06
#define TMR2PS_1024x		0x07
#define TMR2PS_MASK			0x07

//tmr period settings
#define TMR_ms				(F_CPU / 1000)				//1ms period - minimum period
#define TMR_1ms				(TMR_ms * 1)				//1ms
#define TMR_2ms				(TMR_ms * 2)				//2ms period
#define TMR_5ms				(TMR_ms * 5)				//5ms period
#define TMR_10ms			(TMR_ms * 10)				//10ms
#define TMR_20ms			(TMR_ms * 20)				//20ms period
#define TMR_50ms			(TMR_ms * 50)				//50ms period
#define TMR_100ms			(TMR_ms * 100)				//100ms
#define TMR_200ms			(TMR_ms * 200)				//200ms period
#define TMR_500ms			(TMR_ms * 500)				//500ms period
#define TMR_1000ms			(TMR_ms * 1000)				//1000ms
#define TMR_2000ms			(TMR_ms * 2000)				//2000ms period
#define TMR_5000ms			(TMR_ms * 5000)				//5000ms period
#define TMR_10000ms			(TMR_ms * 10000)			//10000ms

//reset the tmr
void tmr2_init(uint8_t prescaler);
void tmr2_act(void (*isr_ptr)(void));

//set period
void tmr2a_setpr(uint8_t pr);
void tmr2b_setpr(uint8_t pr);

//set tmr1 isr ptr
void tmr2a_act(void (*isr_ptr)(void));
void tmr2b_act(void (*isr_ptr)(void));
#endif
//==========================tmr2oc.c================================================
//#include "tmr2oc.h"							//tmr2 header file

//empty handler
static void /*_tmr1_*/empty_handler(void) {
	//default tmr handler
}

static void (* _isrptr_tov)(void)=empty_handler;	//tmr1_ptr pointing to empty_handler by default
static void (* _isrptr_oca)(void)=empty_handler;	//tmr1_ptr pointing to empty_handler by default
static void (* _isrptr_ocb)(void)=empty_handler;	//tmr1_ptr pointing to empty_handler by default
static uint8_t _oca_inc=0xff;					//output compare increment
static uint8_t _ocb_inc=0xff;					//output compare increment

//tmr2 overflow isr
ISR(TIMER2_OVF_vect) {
	//clear the flag - done automatically
	//OCR2+=_tmr2_oc;								//advance to the next output match point
	_isrptr_tov();						//execute the handler
}

//tmr2 output compare isr
ISR(TIMER2_COMPA_vect) {
	//clear the flag - done automatically
	OCR2A+=_oca_inc;								//advance to the next output match point
	_isrptr_oca();						//execute the handler
}

//tmr2 output compare isr
ISR(TIMER2_COMPB_vect) {
	//clear the flag - done automatically
	OCR2B+=_ocb_inc;								//advance to the next output match point
	_isrptr_ocb();						//execute the handler
}

//reset the tmr
//pin in normal operation
//mode: normal mode (wgm20=0b00)
void tmr2_init(uint8_t prescaler) {
	//initialize the handler
	_isrptr_tov = _isrptr_oca = _isrptr_ocb = empty_handler;
	_oca_inc = _ocb_inc = 0xff;

	//initialize the timer
	TCCR2B =	TCCR2B & (~TMR2PS_MASK);			//turn off tmr1
	TCCR2A |=	(0<<COM2A1) | (0<<COM2A0) |	//output compare a pins normal operation
				(0<<COM2B1) | (0<<COM2B0) |	//output compare b pins normal operation
				//(0<<COM1C1) | (0<<COM1C0) |	//output compare c pins normal operation
				(0<<WGM21) | (0<<WGM20)		//wgm2..0 = 0b00 -> normal mode
				;
	TCNT2 = 0;								//reset the timer / counter
	TIFR2 |= (1<<OCF2A) | (1<<OCF2B) | (1<<TOV2);			//clear the flag by writing '1' to it
	TIMSK2 =		//(0<<TICIE1) |				//input capture isr: disabled
				//(0<<OCIE1C) |				//output compare isr for ch a: disabled
				(0<<OCIE2B) |				//output compare isr for ch b: disabled
				(0<<OCIE2A) |				//output compare isr for ch c: disabled
				(0<<TOIE2)					//tmr overflow interrupt: disabled
				;
	TCCR2B |=	(prescaler & TMR2PS_MASK)	//prescaler, per the header file
				;
	//now timer1 is running
}

//set period for output compare
void tmr2a_setpr(uint8_t pr) {
	_oca_inc = pr - 1;
	OCR2A = TCNT2 + _oca_inc;
}

//set period for output compare
void tmr2b_setpr(uint8_t pr) {
	_ocb_inc = pr - 1;
	OCR2B = TCNT2 + _ocb_inc;
}

//install overflow isr
void tmr2_act(void (*isr_ptr)(void)) {
	_isrptr_tov=isr_ptr;					//reassign tmr1 isr ptr
	TIFR2 |= (0<<OCF2A) | (0<<OCF2B) | (1<<TOV2);			//clear the flag by writing '1' to it
	TIMSK2 =		//(0<<TICIE1) |				//input capture isr: disabled
				//(0<<OCIE1C) |				//output compare isr for ch a: disabled
				(0<<OCIE2B) |				//output compare isr for ch b: disabled
				(0<<OCIE2A) |				//output compare isr for ch c: disabled
				(1<<TOIE2)					//tmr overflow interrupt: disabled
				;
}

//install compare match isr
void tmr2a_act(void (*isr_ptr)(void)) {
	_isrptr_oca=isr_ptr;					//reassign tmr1 isr ptr
	TIFR2 |= (1<<OCF2A) | (0<<OCF2B) | (0<<TOV2);			//clear the flag by writing '1' to it
	TIMSK2 =		//(0<<TICIE1) |				//input capture isr: disabled
				//(0<<OCIE1C) |				//output compare isr for ch a: disabled
				(1<<OCIE2A) |				//output compare isr for ch b: disabled
				(0<<OCIE2B) |				//output compare isr for ch c: disabled
				(0<<TOIE2)					//tmr overflow interrupt: disabled
				;
}

//install compare match isr
void tmr2b_act(void (*isr_ptr)(void)) {
	_isrptr_ocb=isr_ptr;					//reassign tmr1 isr ptr
	TIFR2 |= (0<<OCF2A) | (1<<OCF2B) | (0<<TOV2);			//clear the flag by writing '1' to it
	TIMSK2 =		//(0<<TICIE1) |				//input capture isr: disabled
				//(0<<OCIE1C) |				//output compare isr for ch a: disabled
				(0<<OCIE2A) |				//output compare isr for ch b: disabled
				(1<<OCIE2B) |				//output compare isr for ch c: disabled
				(0<<TOIE2)					//tmr overflow interrupt: disabled
				;
}

//==========================led4_pins.h=============================================
/*
	 File:	 led4_bits.h

	 Created on April 15, 2014, 9:27 PM
*/

#ifndef LED4_PINS_H
#define	LED4_PINS_H

//#include "gpio.h"

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

//==========================hx711.h=================================
#ifndef HX711_H_INCLUDED
#define HX711_H_INCLUDED

//#include <pic24.h>						//we use pic24f
//#include "gpio.h"

//hardware configuration
//end hardware configuration

//global defines
#define HX711_VREFmv			1250			//HX711 Vref = 1.250mv

//global variables

//initialize hx711
void hx711_init(void);

//read hx711, 25 pulses
int32_t hx711_read(void);

//read hx711, x128 gain
//Channel A
#define hx711_readx128()		hx711_read()

//read hx711, x32 gain
//channel B
int32_t hx711_readx32(void);

//read hx711, x64 gain
//Channel A
int32_t hx711_readx64(void);

//test if hx711 is busy
//hx711 is busy when DIN is high
#define hx711_busy()			(IO_GET(HX711DOUT_PORT, HX711DOUT))
#define hx711_readA128()		hx711_readx128()
#define hx711_readB32()			hx711_readx32()
#define hx711_readA64()			hx711_readx64()

#endif /* HX711_H_INCLUDED */
//===============================hx711.c======================================
//#include "hx711.h"			//we use hx711

//hardware configuration
//end hardware configuration

//global defines
#define HX711_DLY()							{NOP8();}
#define HX711_DLY2()						{HX711_DLY(); HX711_DLY();}

//global variables

//initialize hx711
void hx711_init(void) {
	//connect GND
	IO_CLR(HX711GND_PORT, HX711GND);
	IO_OUT(HX711GND_DDR, HX711GND);
	
	//dout idles as input
	IO_IN(HX711DOUT_DDR, HX711DOUT);

	//sck as output and idles low
	IO_CLR(HX711SCK_PORT, HX711SCK);
	IO_OUT(HX711SCK_DDR, HX711SCK);
}

//read hx711, 25 pulses
//x128 gain
int32_t hx711_read(void) {
	uint8_t mask = 24;						//mask, msb first
	int32_t tmp=0;

	//hx711_init();							//rest the pins, optional
	IO_CLR(HX711SCK_PORT, HX711SCK);		//sck idles low
	do {
		tmp = tmp << 1;
		IO_SET(HX711SCK_PORT, HX711SCK);	//rising edge of sck
		HX711_DLY2();
		if (IO_GET(HX711DOUT_PORT, HX711DOUT)) tmp |= 1<<0;
		mask-= 1;							//decrement mask
		IO_CLR(HX711SCK_PORT, HX711SCK);	//falling edge of sck
		HX711_DLY();						//for 50% DC
	} while (mask);
	HX711_DLY(); IO_SET(HX711SCK_PORT, HX711SCK); HX711_DLY2(); IO_CLR(HX711SCK_PORT, HX711SCK);	HX711_DLY2(); //25th pulse
	//if (tmp &  0x800000ul) {tmp = (tmp ^ 0xfffffful) + 1; tmp = -tmp;}
	return (tmp & 0x800000ul)?(tmp | 0xff000000ul):tmp;
}

//read hx711, x32 gain
int32_t hx711_readx32(void) {
	int32_t tmp;
	tmp=hx711_read();						//read hx711, 25 pulses
	HX711_DLY(); IO_SET(HX711SCK_PORT, HX711SCK); HX711_DLY2(); IO_CLR(HX711SCK_PORT, HX711SCK);	HX711_DLY2(); //26th pulse
	return tmp;
}
//read hx711, x64 gain
int32_t hx711_readx64(void) {
	int32_t tmp;
	tmp=hx711_read();						//read hx711, 25 pulses
	HX711_DLY(); IO_SET(HX711SCK_PORT, HX711SCK); HX711_DLY2(); IO_CLR(HX711SCK_PORT, HX711SCK);	HX711_DLY2(); //26th pulse
	HX711_DLY(); IO_SET(HX711SCK_PORT, HX711SCK); HX711_DLY(); IO_CLR(HX711SCK_PORT, HX711SCK);	HX711_DLY(); //27th pulse
	return tmp;
}

//start of main loop / user code

//read the uOHM from hx711, no oversampling
int32_t uOHM_read(void) {
	static int32_t tmp=0;
	
	//read ch A at 128x
	tmp = hx711_readA128();			//read hx711 128x gain
	tmp = tmp - ADC_OFFSET;			//adjust for ADC offset error
	//if (tmp < 0) lRAM[0] |= 0x80;	//dot on digit 1 indicates a negative number
	return (tmp>0)?tmp:(-tmp);	//return only the positive values
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
	//IO_OUT(LEDR_DDR, LEDR);
	//IO_CLR(PORTD, 1 << 6); IO_OUT(DDRD, 1 << 6);

	//initialize the led display
	led_init();

	//install timer and update the led display in the background
	tmr2_init(TMR2PS_256x);			 			//display update frequency = F_CPU / 256 / prescaler = 244Hz per digit, or 244Hz/4=60Hz per frame
	tmr2_act(led_display);

	//reset the adc
	hx711_init();								//initialize hx711
	while (hx711_busy()) continue;				//wait for hx711 to be ready
	hx711_readA128();							//first read - dummy read

#if defined(USE_UART)
	Serial.begin(9600);
#endif

	//slow down the clock
	//CLKPR = 0x80;								//unlock the prescaler
	//CLKPR = 0x05;								//set the prescaler

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
	if (!hx711_busy()) {
		tmp = uOHM_read();						//read the adc, at 128x gain
		//if (tmp < ADCHI2LO) uOHM = ADCLO2uOHM(tmp);	//convert ADC to uOHM
		//else uOHM = ADCHI2uOHM(tmp);
		uOHM = (uint32_t) ADC2uOHM(tmp);					//floating point math version
		//smooth out uOHM
		uOHM = uOHM_avg(uOHM);
		//optional: error correction
		uOHM = uOHM_EC(uOHM);						//error correct for uOHM
		//uOHM = tmp;
	}
	//t = micros() - t;
	//tmp = t;									//display the value of t
	//uOHM = 9600000ul;							//for debug only
	//===============end measuring uOHM===========
	
	//display uOHM
	tmp = uOHM;									//display uOHM
	//display tmp
	tmp = (tmp > 9999999ul)?9999999ul:tmp;			//max = 9.999OHM
	//now uOHM max is 9999mOhm, with rounding up
			if (tmp > 999999ul - 500)	{tmp = (tmp + 500) / 1000; dp = 3;}		//1000000..9999999uOHM
	else 	if (tmp >  99999ul -  50)	{tmp = (tmp +  50) /  100; dp = 2;}		//100000..999999uOHM
	else 	if (tmp >   9999ul -   5)	{tmp = (tmp +   5) /   10; dp = 1;}		//10000..99999uOHM
	else								{tmp = (tmp +   0) /    1; dp = 0;}		//0..9999uOHM
	//convert tmp to font
	//can be better optimized
	lRAM[3]= ledfont_num[tmp % 10]; tmp /= 10;
	lRAM[2]= ledfont_num[tmp % 10]; tmp /= 10;
	lRAM[1]= ledfont_num[tmp % 10]; tmp /= 10;
	lRAM[0]= ledfont_num[tmp % 10]; tmp /= 10;
	lRAM[dp] |= 0x80;							//add floating decimal point
	
	//debug only
	//ledr_flp();								//for debug only
	delay(uOHM_DLY);							//slow down the display update frequency - simulate HX711 sample time

#if defined(USE_UART)
	Serial.print("uOHM = "); Serial.print(uOHM); Serial.println("uR.");
#endif
}
