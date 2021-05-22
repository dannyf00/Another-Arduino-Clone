#ifndef _STM32DUINO_H
#define _STM32DUINO_H

//STM32duino code
// - using STM32F030F4P6 chip
// - free running systick for ticks
// - 
// - version history
// - v2, 5/22/2021: simplified port
// - 
//
//              |=====================|
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |=====================|
//
//
//

#include <stm32f0xx.h>						//we use STM32F0
#include <stdint.h>							//we use uint types

//hardware configuration
#define TIM1CH1toGPIO()				TIM1CH1toPA8()
#define TIM1CH2toGPIO()				TIM1CH2toPA9()
#define TIM1CH3toGPIO()				TIM1CH3toPA10()
#define TIM1CH4toGPIO()				TIM1CH4toPA11()

#define TIM3CH1toGPIO()				TIM3CH1toPA6()		//TIM3CH1toPB4()
#define TIM3CH2toGPIO()				TIM3CH2toPA7()		//TIM3CH2toPB5()
#define TIM3CH3toGPIO()				TIM3CH3toPB0()
#define TIM3CH4toGPIO()				TIM3CH4toPB1()

#define TIM14CH1toGPIO()			TIM14CH1toPA4()		//TIM14CH1toPA7(), TIM14CH1toPB1()

#define TIM15CH1toGPIO()			TIM15CH1toPA2()		//TIM15CH1toPB14() - only on STM32F030x8
#define TIM15CH2toGPIO()			TIM15CH2toPA3()		//TIM15CH1toPB15() - only on STM32F030x8

#define TIM16CH1toGPIO()			TIM16CH1toPA6()		//TIM16CH1toPB8()

#define TIM17CH1toGPIO()			TIM17CH1toPA7()		//TIM17CH1toPB9()
//end pin configuration

//oscillator configuration
#define F_XTAL				8000000ul		//crystal frequency, user-specified
//end user specification

#define F_PHB				(F_CPU)			//cpu runs at 8Mhz/2 by default -> Fxtal = 8Mhz. *4 for PLL. RCDIV set to 0 (1:1 postscaler)
#define F_CPU				(SystemCoreClock)			//peripheral block runs at F_PHB - default = F_CPU / 1

#define PWM_PR				0xffff			//pwm period - don't change

//port manipulation macros for PIC.
//MODE:
//Output modes: GPIOMODE_OUTPP, GPIOMODE_OUTOD
//Input mode: GPIOMODE_INPUT, GPIOMODE_INPUTPD, GPIOMODE_INPUTPU
//Alternate function: GPIOMODE_AFPP, GPIOMODE_AFOD
//for moder
#define GPIOMODE_INPUT		(0<<0)		//(0<<0)
#define GPIOMODE_OUTPUT		(1<<0)		//(1<<0)
#define GPIOMODE_AF			(2<<0)
#define GPIOMODE_AN			(3<<0)
//for otyper
#define GPIOMODE_PP			(1<<3)
#define GPIOMODE_OD			(0<<3)
//for pupdr
#define GPIOMODE_FL			(0<<4)
#define GPIOMODE_PU			(1<<4)
#define GPIOMODE_PD			(2<<4)

//AF defs
#define GPIOMODE_AF0		0
#define GPIOMODE_AF1		1
#define GPIOMODE_AF2		2
#define GPIOMODE_AF3		3
#define GPIOMODE_AF4		4
#define GPIOMODE_AF5		5
#define GPIOMODE_AF6		6
#define GPIOMODE_AF7		7

#define GPIOMODE_OUTPP		(GPIOMODE_OUTPUT | GPIOMODE_PP)		//gpio, output, push-pull
#define GPIOMODE_OUTOD		(GPIOMODE_OUTPUT | GPIOMODE_OD)		//gpio, output, od
#define GPIOMODE_INFL		(GPIOMODE_INPUT)
#define GPIOMODE_INPU		(GPIOMODE_INPUT | GPIOMODE_PU)
#define GPIOMODE_INPD		(GPIOMODE_INPUT | GPIOMODE_PD)
#define GPIOMODE_AFPP		(GPIOMODE_AF | GPIOMODE_PP)
#define GPIOMODE_AFOD		(GPIOMODE_AF | GPIOMODE_OD)
#define GPIOMODE_INAN		(GPIOMODE_AN)

//global variables

//port/gpio oriented macros
#define IO_SET(port, pins)					port->ODR |= (pins)				//set bits on port
#define IO_CLR(port, pins)					port->ODR &=~(pins)				//clear bits on port
#define IO_FLP(port, pins)					port->ODR ^= (pins)				//flip bits on port
#define IO_GET(port, pins)					((port->IDR) & (pins))			//return bits on port
//set a pin to output/input
#define IO_OUTPP(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTPP)	//push-pull mode (CR1 set, CR2 cleared)	//IO_OUTPP(GPIOx, GPIO_Pins).
#define IO_OUTOD(port, pins)				GPIO_DDR(port, pins, GPIOMODE_OUTOD)	//open drain mode (cr1 + cr2 cleared)	//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_OD_LOW_FAST)
#define IO_OUT(port, pins)					IO_OUTPP(port, pins)					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_PP_LOW_FAST)
#define IO_INFL(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INFL)		//floating input, no interrupt			//IO_INFL(GPIOx, GPIO_Pins)
#define IO_INPU(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INPU)		//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define IO_INPD(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INPD)		//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define IO_IN(port, pins)					IO_INFL(port, pins)					//IO_IN(port, pins)				//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_FL_NO_IT)
#define IO_AFPP(port, pins)					GPIO_DDR(port, pins, GPIOMODE_AFPP)		//configure pin for alternative function output, push-pull, 10Mhz
#define IO_AFOD(port, pins)					GPIO_DDR(port, pins, GPIOMODE_AFOD)		//configure pin for alternative function output, open-drain, 10Mhz
#define IO_AN(port, pins)					GPIO_DDR(port, pins, GPIOMODE_INAN)

//fast routines through BRR/BSRR registers
#define GIO_SET(port, pins)					port->BSRR = (pins)
#define GIO_CLR(port, pins)					port->BRR = (pins)
#define GIO_FLP(port, pins)					IO_FLP(port, pins)
#define GIO_GET(port, pins)					IO_GET(port, pins)
#define GIO_IN(port, pins)					IO_IN(port, pins)
#define GIO_OUT(port, pins)					IO_OUT(port, pins)
//configure gpio mode (cnf1..0 + mod1..0 bits)
void GPIO_DDR(GPIO_TypeDef * gpio, uint32_t mask, uint32_t mode);

#define NOP()				Nop()                           //asm("nop")					//nop()
#define NOP2()				{NOP(); NOP();}
#define NOP4()				{NOP2(); NOP2();}
#define NOP8()				{NOP4(); NOP4();}
#define NOP16()				{NOP8(); NOP8();}
#define NOP16()				{NOP8(); NOP8();}
#define NOP24()				{NOP16(); NOP8();}
#define NOP32()				{NOP16(); NOP16();}
#define NOP40()				{NOP32(); NOP8();}
#define NOP64()				{NOP32(); NOP32();}

#define sleep()				asm("sleep")					//put the mcu into sleep

#ifndef ei
#define ei()				//asm volatile ("ei")				//__builtin_enable_interrupts()	//do {INTEnableInterrupts();	INTEnableSystemMultiVectoredInt();} while (0)	//__builtin_enable_interrupts()
#endif

#ifndef di
#define di()				//asm volatile ("di")				//__builtin_enable_interrupts()	//INTDisableInterrupts()			//__builtin_disable_interrupts()	//
#endif


//simple multiples
#define x1(val)				(val)								//multiply val by 1
#define x2(val)				(((val) << 1))						//multiply val by 2
#define x3(val)				(x2(val) + (val))					//multiply val by 3
#define x4(val)				(((val) << 2))						//multiply val by 4
#define x5(val)				(x4(val) + (val))					//multiply val by 5
#define x6(val)				(x4(val) + x2(val))					//multiply val by 6
#define x7(val)				(x6(val) + (val))					//multiply val by 7
#define x8(val)				((val) << 3)						//multiply val by 8
#define x9(val)				(x8(val) + (val))					//multiply val by 9

//multiples of 10s
#define x10(val)			(x8(val) + x2(val))					//multiply val by 10
#define x100(val)			(x10(x10(val)))						//multiply val by 100
#define x1000(val)			(x100(x10(val)))					//multiply val by 1000
#define x1k(val)			x1000(val)							//multiply val by 1000
#define x10k(val)			(x100(x100(val)))					//multiply val by 10000

#define x20(val)			(x2(x10(val)))
#define x30(val)			(x3(x10(val)))
#define x40(val)			(x4(x10(val)))
#define x50(val)			(x5(x10(val)))
#define x60(val)			(x6(x10(val)))
#define x70(val)			(x7(x10(val)))
#define x80(val)			(x8(x10(val)))
#define x90(val)			(x9(x10(val)))

//multiples of 100s
#define x200(val)			(x2(x100(val)))
#define x300(val)			(x3(x100(val)))
#define x400(val)			(x4(x100(val)))
#define x500(val)			(x5(x100(val)))
#define x600(val)			(x6(x100(val)))
#define x700(val)			(x7(x100(val)))
#define x800(val)			(x8(x100(val)))
#define x900(val)			(x9(x100(val)))

//custom definitions
#define x34(val)			(x30(val) + x4(val))				//multiply val by 34
#define x97(val)			(x90(val) + x7(val))				//multiply val by 97x

typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned long uint32_t;
typedef signed long int32_t;

//port structure

//global defines

//pin enum - matches GPIO_PinDef[]
typedef enum {
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
#if defined(GPIOC)
	PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
#endif		//GPIOC
#if defined(GPIOD)
	PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
#endif		//GPIOC
#if defined(GPIOE)
	PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
#endif		//GPIOC
#if defined(GPIOF)
	PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
#endif		//GPIOC
	PMAX
} PIN_TypeDef;

//map pin number to GPIOx
typedef struct {
	GPIO_TypeDef *gpio;					//gpio for a pin
	uint16_t mask;						//pin mask - 16-bit port
} PIN2GPIO;
	
#define INPUT				0
#define OUTPUT				1			//(!INPUT)
#define INPUT_PULLUP		2

#define LOW					0
#define HIGH				(!LOW)

#define PI 					3.1415926535897932384626433832795
#define HALF_PI 			(PI / 2)							//1.5707963267948966192313216916398
#define TWO_PI 				(PI + PI)							//6.283185307179586476925286766559
#define DEG_TO_RAD 			(TWO_PI / 360)						//0.017453292519943295769236907684886
#define RAD_TO_DEG 			(360 / TWO_PI)						//57.295779513082320876798154814105
#define EULER 				2.718281828459045235360287471352	//Euler's number

#define SERIAL  			0x0
#define DISPLAY 			0x1

#define LSBFIRST 			0
#define MSBFIRST 			1									//(!LSBFIRST)							//1

#define CHANGE 				1
#define FALLING 			2
#define RISING 				3

#ifndef min
#define min(a,b) 			((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) 			((a)>(b)?(a):(b))
#endif
#define abs(x) 				((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     		((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) 		((deg)*DEG_TO_RAD)
#define degrees(rad) 		((rad)*RAD_TO_DEG)
#define sq(x) 				((x)*(x))

#define interrupts() 		ei()
#define noInterrupts() 		di()

#define clockCyclesPerMillisecond() 	( F_CPU / 1000L )
#define clockCyclesPerMicrosecond() 	( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) 	( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) 	( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) 			((uint8_t) ((w) & 0xff))
#define highByte(w) 		((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) 	((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bit(n)				(1ul<<(n))

#define false				0
#define true				(!false)

//characters
#define isAlphaNumeric(c)	isalnum(c)
#define isAlpha(c)			isalpha(c)
#define isAscii(c)			isascii(c)
#define isWhitespace(c)		isblank(c)
#define isControl(c)		iscntrl(c)
#define isDigit(c)			isdigit(c)
#define isGraph(c)			isgraph(c)
#define isLowerCase(c)		islower(c)
#define isPrintable(c)		isprint(c)
#define isPunct(c)			ispunct(c)
#define isSpace(c)			isspace(c)
#define isUpperCase(c)		isupper(c)
#define isHexadecimalDigit(c)	isxdigit(c)

//external setup/loop - defined by user
extern void setup(void);
extern void loop(void);

//random number
#define randomSeed(seed)	srand(seed)
#define random(max)			random2(0, max)
#define random2(min, max)	((min) + (int32_t) ((max) - (min)) * rand() / 32768)

//GPIO
void pinMode(PIN_TypeDef pin, uint8_t mode);
void digitalWrite(PIN_TypeDef pin, uint8_t mode);
int digitalRead(PIN_TypeDef pin);

//time base
//uint32_t ticks(void);								//timer ticks from timer2
uint32_t systicks(void);
#define ticks()				systicks()				//map ticks to systicks()
#define millis()			(ticks() / cyclesPerMillisecond())
#define micros()			(ticks() / cyclesPerMicrosecond())
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
#define cyclesPerMicrosecond()			(F_CPU / 1000000ul)
#define cyclesPerMillisecond()			(F_CPU / 1000)

//advanced IO
//void tone(void);									//tone frequency specified by F_TONE in STM8Sduino.h
//void noTone(void);
//shiftin/out: bitOrder = MSBFIRST or LSBFIRST
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder);
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val);
uint32_t pulseIn(PIN_TypeDef pin, uint8_t state);		//wait for a pulse and return timing

//pwm output
//dc = 0x00..0x0fff for pwm2/3/4/5, 0x00..0xffff for pwm1
//RP4=PWM1, RP12=PWM2, RP13=PWM3, RP14=PWM4, RP15=PWM5
void analogWrite(uint8_t pin, uint16_t dc);

//analog read on ADC1
//read DRL first for right aligned results
//uint16_t analogRead(uint8_t pin);

//analog reference - default to AVdd-AVss
//Vref sources: 0->Vref = AVdd-AVss, 1->Vref+-AVss, 2->AVdd-Vref-, 3->Vref+ - Vref-
//void analogReference(uint8_t Vref);

//interrupts
//install external interrupt handler
//mode 1: falling edge, 0: rising edge
//void attachInterrupt(uint8_t intx, void (*isrptr) (void), uint8_t mode);
//void detachInterrupt(uint8_t intx);

//change notification interrupts
//install user CN interrupt handler
//void attachCNInterrupt(void (*isrptr) (void));
//void detachCNInterrupt(void);
//void activateCNInterrupt(uint8_t cnx, uint8_t pue);
//void deactivateCNInterrupt(uint8_t cnx);

//global variables

//reset the mcu
void mcu_init(void);

//empty interrupt handler
void empty_handler(void);


//#define Mhz					000000ul	//suffix for Mhz
#define F_UART				(F_PHB)	//8Mhz		//crystal frequency
#define UART_BR300			300ul		//buadrate=300
#define UART_BR600			600ul		//buadrate=600
#define UART_BR1200			1200ul		//buadrate=1200
#define UART_BR2400			2400ul		//buadrate=2400
#define UART_BR4800			4800ul		//buadrate=4800
#define UART_BR9600			9600ul		//buadrate=9600
#define UART_BR19200		19200ul		//buadrate=19200
#define UART_BR38400		38400ul		//buadrate=38400
#define UART_BR57600		57600ul		//buadrate=57600
#define UART_BR115200		115200ul	//buadrate=115200

//for compatability
#define uart1Put(ch)		uart1Putch(ch)
#define uart1Get()			uart1Getch()

//initiate the hardware usart
void uart1Init(unsigned long baud_rate);
void uart1Putch(char ch);
void uart1Puts(char *str);
uint16_t uart1Available(void);
uint16_t uart1Busy(void);

void uart1Putline(char *ln);
//read a char from usart
uint8_t uart1Getch(void);

//for compatability
#define uart2Put(ch)		uart2Putch(ch)
#define uart2Get()			uart2Getch()
//initiate the hardware usart
void uart2Init(unsigned long baud_rate);
void uart2Putch(char ch);
void uart2Puts(char *str);
uint16_t uart2Available(void);
uint16_t uart2Busy(void);
void uart2Putline(char *ln);

//read a char from usart
uint8_t uart2Getch(void);
//end Serial

//initialize the timer1 (16bit)
void tmr1Init(uint16_t ps);
void tmr1SetPR1(uint16_t pr);					//set period
void tmr1AttachISR1(void (*isrptr)(void));		//activate the isr handler
void tmr1SetPR2(uint16_t pr);					//set period
void tmr1AttachISR2(void (*isrptr)(void));		//activate the isr handler
void tmr1SetPR3(uint16_t pr);					//set period
void tmr1AttachISR3(void (*isrptr)(void));		//activate the isr handler
void tmr1SetPR4(uint16_t pr);					//set period
void tmr1AttachISR4(void (*isrptr)(void));		//activate the isr handler

//initialize the timer3 (16bit)
void tmr3Init(uint16_t ps);
void tmr3SetPR1(uint16_t pr);					//set period
void tmr3AttachISR1(void (*isrptr)(void));		//activate the isr handler
void tmr3SetPR2(uint16_t pr);					//set period
void tmr3AttachISR2(void (*isrptr)(void));		//activate the isr handler
void tmr3SetPR3(uint16_t pr);					//set period
void tmr3AttachISR3(void (*isrptr)(void));		//activate the isr handler
void tmr3SetPR4(uint16_t pr);					//set period
void tmr3AttachISR4(void (*isrptr)(void));		//activate the isr handler

//initialize the timer14 (16bit)
void tmr14Init(uint16_t ps);
void tmr14SetPR(uint16_t pr);					//set period
void tmr14AttachISR(void (*isrptr)(void));		//activate the isr overflow handler

//initialize the timer16 (16bit)
void tmr16Init(uint16_t ps);
void tmr16SetPR(uint16_t pr);					//set period
void tmr16AttachISR(void (*isrptr)(void));		//activate the isr overflow handler

//initialize the timer17 (16bit)
void tmr17Init(uint16_t ps);
void tmr17SetPR(uint16_t pr);					//set period
void tmr17AttachISR(void (*isrptr)(void));		//activate the isr overflow handler

//pwm / oc
//global defines
//MODER=0b10 (alternate function), OTYPER=(push-pull), OSPEEDR = medium speed
//Alternate function = 2, AFR[1]/high byte
#define TIM1CH1toPA8()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 8))) | (0b10<<(2* 8)); GPIOA->OTYPER &=~(1<< 8); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 8))) | (0b01<<(2* 8)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 8%8)))) | (2<<(4*( 8%8)));} while (0)
#define TIM1CH2toPA9()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 9))) | (0b10<<(2* 9)); GPIOA->OTYPER &=~(1<< 9); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 9))) | (0b01<<(2* 9)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*( 9%8)))) | (2<<(4*( 9%8)));} while (0)
#define TIM1CH3toPA10()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2*10))) | (0b10<<(2*10)); GPIOA->OTYPER &=~(1<<10); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2*10))) | (0b01<<(2*10)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(10%8)))) | (2<<(4*(10%8)));} while (0)
#define TIM1CH4toPA11()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2*11))) | (0b10<<(2*11)); GPIOA->OTYPER &=~(1<<11); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2*11))) | (0b01<<(2*11)); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f<<(4*(11%8)))) | (2<<(4*(11%8)));} while (0)

//MODER=0b10 (alternate function), OTYPER=(push-pull), OSPEEDR = medium speed
//Alternate function = 1, AFR[0]/low byte
#define TIM3CH1toPA6()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 6))) | (0b10<<(2* 6)); GPIOA->OTYPER &=~(1<< 6); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 6))) | (0b01<<(2* 6)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 6%8)))) | (1<<(4*( 6%8)));} while (0)
//AF1
#define TIM3CH1toPB4()				do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(0b11<<(2* 4))) | (0b10<<(2* 4)); GPIOB->OTYPER &=~(1<< 4); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(0b11<<(2* 4))) | (0b01<<(2* 4)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 4%8)))) | (1<<(4*( 4%8)));} while (0)
//AF1
#define TIM3CH2toPA7()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 7))) | (0b10<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 7))) | (0b01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (1<<(4*( 7%8)));} while (0)
//AF1
#define TIM3CH2toPB5()				do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(0b11<<(2* 5))) | (0b10<<(2* 5)); GPIOB->OTYPER &=~(1<< 5); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(0b11<<(2* 5))) | (0b01<<(2* 5)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 5%8)))) | (1<<(1*( 5%8)));} while (0)
//AF1
#define TIM3CH3toPB0()				do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(0b11<<(2* 0))) | (0b10<<(2* 0)); GPIOB->OTYPER &=~(1<< 0); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(0b11<<(2* 0))) | (0b01<<(2* 0)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 0%8)))) | (1<<(4*( 0%8)));} while (0)
//AF1
#define TIM3CH4toPB1()				do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(0b11<<(2* 1))) | (0b10<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(0b11<<(2* 1))) | (0b01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (1<<(4*( 1%8)));} while (0)

//AF4, AFR[0]/low byte
#define TIM14CH1toPA4()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 4))) | (0b10<<(2* 4)); GPIOA->OTYPER &=~(1<< 4); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 4))) | (0b01<<(2* 4)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 4%8)))) | (4<<(4*( 4%8)));} while (0)
#define TIM14CH1toPA7()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 7))) | (0b10<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 7))) | (0b01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (4<<(4*( 7%8)));} while (0)
//AF0, AFR[0]/low byte
#define TIM14CH1toPB1()				do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(0b11<<(2* 1))) | (0b10<<(2* 1)); GPIOB->OTYPER &=~(1<< 1); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(0b11<<(2* 1))) | (0b01<<(2* 1)); GPIOB->AFR[0] = (GPIOB->AFR[0] &~(0x0f<<(4*( 1%8)))) | (0<<(4*( 1%8)));} while (0)

//MODER=0b10 (alternate function), OTYPER=(push-pull), OSPEEDR = medium speed
//Alternate function = 0, AFR[0]/low byte
#define TIM15CH1toPA2()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 2))) | (0b10<<(2* 2)); GPIOA->OTYPER &=~(1<< 2); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 2))) | (0b01<<(2* 2)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 2%8)))) | (0<<(4*( 2%8)));} while (0)
#define TIM15CH2toPA3()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 3))) | (0b10<<(2* 3)); GPIOA->OTYPER &=~(1<< 3); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 3))) | (0b01<<(2* 3)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 3%8)))) | (0<<(4*( 3%8)));} while (0)
//AF=1, AFR[1]/high byte
#define TIM15CH1toPB14()			do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(0b11<<(2*14))) | (0b10<<(2*14)); GPIOB->OTYPER &=~(1<<14); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(0b11<<(2*14))) | (0b01<<(2*14)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*(14%8)))) | (1<<(4*(14%8)));} while (0)
#define TIM15CH1toPB15()			do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(0b11<<(2*15))) | (0b10<<(2*15)); GPIOB->OTYPER &=~(1<<15); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(0b11<<(2*15))) | (0b01<<(2*15)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*(15%8)))) | (1<<(4*(15%8)));} while (0)

//MODER=0b10 (alternate function), OTYPER=(push-pull), OSPEEDR = medium speed
//Alternate function = 5, AFR[0]/low byte
#define TIM16CH1toPA6()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 6))) | (0b10<<(2* 6)); GPIOA->OTYPER &=~(1<< 6); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 6))) | (0b01<<(2* 6)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 6%8)))) | (5<<(4*( 6%8)));} while (0)
//AF2, AFR[1]/high byte
#define TIM16CH1toPB8()				do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(0b11<<(2* 8))) | (0b10<<(2* 8)); GPIOB->OTYPER &=~(1<< 8); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(0b11<<(2* 8))) | (0b01<<(2* 8)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*( 8%8)))) | (2<<(4*( 8%8)));} while (0)

//MODER=0b10 (alternate function), OTYPER=(push-pull), OSPEEDR = medium speed
//Alternate function = 5, AFR[0]/low byte
#define TIM17CH1toPA7()				do {RCC->AHBENR |= RCC_AHBENR_GPIOAEN; GPIOA->MODER = (GPIOA->MODER &~(0b11<<(2* 7))) | (0b10<<(2* 7)); GPIOA->OTYPER &=~(1<< 7); GPIOA->OSPEEDR = (GPIOA->OSPEEDR &~(0b11<<(2* 7))) | (0b01<<(2* 7)); GPIOA->AFR[0] = (GPIOA->AFR[0] &~(0x0f<<(4*( 7%8)))) | (5<<(4*( 7%8)));} while (0)
//AF2, AFR[1]/high byte
#define TIM17CH1toPB9()				do {RCC->AHBENR |= RCC_AHBENR_GPIOBEN; GPIOB->MODER = (GPIOB->MODER &~(0b11<<(2* 9))) | (0b10<<(2* 9)); GPIOB->OTYPER &=~(1<< 9); GPIOB->OSPEEDR = (GPIOB->OSPEEDR &~(0b11<<(2* 9))) | (0b01<<(2* 9)); GPIOB->AFR[1] = (GPIOB->AFR[1] &~(0x0f<<(4*( 9%8)))) | (2<<(4*( 9%8)));} while (0)
//global variables

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm1Init(uint16_t TxCCP_PS);

//set duty cycle
#define pwm1SetDC1(dc)		TIM1->CCR1 = (dc)
#define pwm1SetDC2(dc)		TIM1->CCR2 = (dc)
#define pwm1SetDC3(dc)		TIM1->CCR3 = (dc)
#define pwm1SetDC4(dc)		TIM1->CCR4 = (dc)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm3Init(uint16_t TxCCP_PS);

//set duty cycle
#define pwm3SetDC1(dc)			TIM3->CCR1 = (dc)
#define pwm3SetDC2(dc)			TIM3->CCR2 = (dc)
#define pwm3SetDC3(dc)			TIM3->CCR3 = (dc)
#define pwm3SetDC4(dc)			TIM3->CCR4 = (dc)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm14Init(uint16_t TxCCP_PS);
//set duty cycle
#define pwm14SetDC1(dc)			TIM14->CCR1 = (dc)
#define pwm14SetDC2(dc)			TIM14->CCR2 = (dc)
#define pwm14SetDC3(dc)			TIM14->CCR3 = (dc)
#define pwm14SetDC4(dc)			TIM14->CCR4 = (dc)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm15Init(uint16_t TxCCP_PS);
//set duty cycle
#define pwm15SetDC1(dc)			TIM15->CCR1 = (dc)
#define pwm15SetDC2(dc)			TIM15->CCR2 = (dc)
#define pwm15SetDC3(dc)			TIM15->CCR3 = (dc)
#define pwm15SetDC4(dc)			TIM15->CCR4 = (dc)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm16Init(uint16_t TxCCP_PS);
//set duty cycle
#define pwm16SetDC1(dc)			TIM16->CCR1 = (dc)
#define pwm16SetDC2(dc)			TIM16->CCR2 = (dc)
#define pwm16SetDC3(dc)			TIM16->CCR3 = (dc)
#define pwm16SetDC4(dc)			TIM16->CCR4 = (dc)

//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm17Init(uint16_t TxCCP_PS);
//set duty cycle
#define pwm17SetDC1(dc)			TIM17->CCR1 = (dc)
#define pwm17SetDC2(dc)			TIM17->CCR2 = (dc)
#define pwm17SetDC3(dc)			TIM17->CCR3 = (dc)
#define pwm17SetDC4(dc)			TIM17->CCR4 = (dc)

//adc
//adc channel definitions
#define ADC_CH0			(1ul<<0)
#define ADC_CH1			(1ul<<1)
#define ADC_CH2			(1ul<<2)
#define ADC_CH3			(1ul<<3)
#define ADC_CH4			(1ul<<4)
#define ADC_CH5			(1ul<<5)
#define ADC_CH6			(1ul<<6)
#define ADC_CH7			(1ul<<7)
#define ADC_CH8			(1ul<<8)
#define ADC_CH9			(1ul<<9)
#define ADC_CH10		(1ul<<10)
#define ADC_CH11		(1ul<<11)
#define ADC_CH12		(1ul<<12)
#define ADC_CH13		(1ul<<13)
#define ADC_CH14		(1ul<<14)
#define ADC_CH15		(1ul<<15)
#define ADC_CH16		(1ul<<16)
#define ADC_CH17		(1ul<<17)
#define ADC_TS			ADC_CH16				//tempeature sensor on CH16
#define ADC_VREFINT		ADC_CH17				//internal reference on CH17 @ 1.20v

//convert temperature sensor adc reading into temperaturex10
//follow the datasheet. 3.3v Vref (3.0v for my board), 12bit adc
#define Tx10(adc)		(3529 - ((uint32_t) (3300*10*10/43 * (adc)) >> 12))

//rest the adc
//automatic sampling (ASAM=1), manual conversion
void adc1Init(void);
uint16_t adc1Read(uint32_t adc_ch);

//read the adc
#define analogRead(ch)	adc1Read(ch)
//end ADC

//input capture
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic1Init(void);
void ic1AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic1Get(void);							//read buffer value

void ic2Init(void);
void ic2AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic2Get(void);							//read buffer value

void ic3Init(void);
void ic3AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic3Get(void);							//read buffer value

void ic4Init(void);
void ic4AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic4Get(void);							//read buffer value

void ic5Init(void);
void ic5AttachISR(void (*isrptr)(void));		//activate user ptr
uint16_t ic5Get(void);							//read buffer value
//end input capture

//extint
void int0Init(void);							//initialize the module
void int0AttachISR(void (*isrptr) (void));		//attach user isr

void int1Init(void);							//initialize the module
void int1AttachISR(void (*isrptr) (void));		//attach user isr

void int2Init(void);							//initialize the module
void int2AttachISR(void (*isrptr) (void));		//attach user isr

void int3Init(void);							//initialize the module
void int3AttachISR(void (*isrptr) (void));		//attach user isr

void int4Init(void);							//initialize the module
void int4AttachISR(void (*isrptr) (void));		//attach user isr
//end extint

//spi

void spi1Init(uint16_t br);						//reset the spi
void spi1Write(uint8_t dat);					//send data via spi
#define spi1Busy()			(SPI1STATbits.SPITBF)	//transmit buffer full, must wait before writing to SPIxBUF
#define spi1Available()		(!SPI1STATbits.SPIRBE)	//receive buffer not empty -> there is data
#define spi1Read()			(SPI1BUF)			//read from the buffer

void spi2Init(uint16_t br);						//reset the spi
void spi2Write(uint8_t dat);					//send data via spi
#define spi2Busy()			(SPI2STATbits.SPITBF)	//transmit buffer full, must wait before writing to SPIxBUF
#define spi2Available()		(!SPI2STATbits.SPIRBE)	//receive buffer not empty -> there is data
#define spi2Read()			(SPI2BUF)			//read from the buffer

//end spi

//i2c
//end i2c

//rtcc

//end rtcc

//cnint
//pin: one pin only
//gpio: GPIOA..GPIOG
void extiInit(GPIO_TypeDef * gpio, uint16_t pin, uint8_t edge);
void extiAttachISR(uint16_t pin, void (*isr_ptr)(void));		//install user handler

//end cnint


#endif
