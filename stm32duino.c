#include "stm32duino.h"				//stm32duino

//global defines

//global variables
//declare pins
//ALL PINS ARE MAPPED, WHETHER THEY EXIST OR NOT
//SO MAKE SURE THAT THE PINS YOU PICKED ACTUALLY EXIST FOR YOUR PACKAGE
//Pin  0.. 7 -> GPIOA
//Pin  8..15 -> GPIOB
const PIN2GPIO GPIO_PinDef[]={
	{GPIOA, 1<<0},						//STM32duino Pin  0 = RP0/PB0/CHIP PIN4
	{GPIOA, 1<<1},						//STM32duino Pin  1 = RP1/PB1/CHIP PIN5
	{GPIOA, 1<<2},						//STM32duino Pin  2 = RP2/PB2/CHIP PIN6
	{GPIOA, 1<<3},						//STM32duino Pin  3 = RP3/PB3/CHIP PIN7
	{GPIOA, 1<<4},						//STM32duino Pin  4 = RP4/PB4/CHIP PIN11
	{GPIOA, 1<<5},						//STM32duino Pin  5 = RP5/PB5/CHIP PIN14
	{GPIOA, 1<<6},						//STM32duino Pin  6 = RP6/PB6/CHIP PIN15
	{GPIOA, 1<<7},						//STM32duino Pin  7 = RP7/PB7/CHIP PIN16
	{GPIOA, 1<<8},						//STM32duino Pin  8 = RP8/PB8/CHIP PIN17
	{GPIOA, 1<<9},						//STM32duino Pin  9 = RP9/PB9/CHIP PIN18
	{GPIOA, 1<<10},						//STM32duino Pin 10 = RP10/PB10/CHIP PIN21
	{GPIOA, 1<<11},						//STM32duino Pin 11 = RP11/PB11/CHIP PIN22
	{GPIOA, 1<<12},						//STM32duino Pin 12 = RP12/PB12/CHIP PIN23
	{GPIOA, 1<<13},						//STM32duino Pin 13 = RP13/PB13/CHIP PIN24
	{GPIOA, 1<<14},						//STM32duino Pin 14 = RP14/PB14/CHIP PIN25
	{GPIOA, 1<<15},						//STM32duino Pin 15 = RP15/PB15/CHIP PIN26

	{GPIOB, 1<<0},						//STM32duino Pin 16 = RP0/PB0/CHIP PIN4
	{GPIOB, 1<<1},						//STM32duino Pin 17 = RP1/PB1/CHIP PIN5
	{GPIOB, 1<<2},						//STM32duino Pin 18 = RP2/PB2/CHIP PIN6
	{GPIOB, 1<<3},						//STM32duino Pin 19 = RP3/PB3/CHIP PIN7
	{GPIOB, 1<<4},						//STM32duino Pin 20 = RP4/PB4/CHIP PIN11
	{GPIOB, 1<<5},						//STM32duino Pin 21 = RP5/PB5/CHIP PIN14
	{GPIOB, 1<<6},						//STM32duino Pin 22 = RP6/PB6/CHIP PIN15
	{GPIOB, 1<<7},						//STM32duino Pin 23 = RP7/PB7/CHIP PIN16
	{GPIOB, 1<<8},						//STM32duino Pin 24 = RP8/PB8/CHIP PIN17
	{GPIOB, 1<<9},						//STM32duino Pin 25 = RP9/PB9/CHIP PIN18
	{GPIOB, 1<<10},						//STM32duino Pin 26 = RP10/PB10/CHIP PIN21
	{GPIOB, 1<<11},						//STM32duino Pin 27 = RP11/PB11/CHIP PIN22
	{GPIOB, 1<<12},						//STM32duino Pin 28 = RP12/PB12/CHIP PIN23
	{GPIOB, 1<<13},						//STM32duino Pin 29 = RP13/PB13/CHIP PIN24
	{GPIOB, 1<<14},						//STM32duino Pin 30 = RP14/PB14/CHIP PIN25
	{GPIOB, 1<<15},						//STM32duino Pin 31 = RP15/PB15/CHIP PIN26

#if defined(GPIOC)
	{GPIOC, 1<<0},						//STM32duino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOC, 1<<1},						//STM32duino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOC, 1<<2},						//STM32duino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOC, 1<<3},						//STM32duino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOC, 1<<4},						//STM32duino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOC, 1<<5},						//STM32duino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOC, 1<<6},						//STM32duino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOC, 1<<7},						//STM32duino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOC, 1<<8},						//STM32duino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOC, 1<<9},						//STM32duino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOC, 1<<10},						//STM32duino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOC, 1<<11},						//STM32duino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOC, 1<<12},						//STM32duino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOC, 1<<13},						//STM32duino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOC, 1<<14},						//STM32duino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOC, 1<<15},						//STM32duino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOC

#if defined(GPIOD)						//pins 48..63
	{GPIOD, 1<<0},						//STM32duino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOD, 1<<1},						//STM32duino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOD, 1<<2},						//STM32duino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOD, 1<<3},						//STM32duino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOD, 1<<4},						//STM32duino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOD, 1<<5},						//STM32duino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOD, 1<<6},						//STM32duino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOD, 1<<7},						//STM32duino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOD, 1<<8},						//STM32duino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOD, 1<<9},						//STM32duino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOD, 1<<10},						//STM32duino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOD, 1<<11},						//STM32duino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOD, 1<<12},						//STM32duino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOD, 1<<13},						//STM32duino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOD, 1<<14},						//STM32duino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOD, 1<<15},						//STM32duino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOD

#if defined(GPIOE)						//pins 64..79
	{GPIOE, 1<<0},						//STM32duino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOE, 1<<1},						//STM32duino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOE, 1<<2},						//STM32duino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOE, 1<<3},						//STM32duino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOE, 1<<4},						//STM32duino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOE, 1<<5},						//STM32duino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOE, 1<<6},						//STM32duino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOE, 1<<7},						//STM32duino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOE, 1<<8},						//STM32duino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOE, 1<<9},						//STM32duino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOE, 1<<10},						//STM32duino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOE, 1<<11},						//STM32duino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOE, 1<<12},						//STM32duino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOE, 1<<13},						//STM32duino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOE, 1<<14},						//STM32duino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOE, 1<<15},						//STM32duino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOE

#if defined(GPIOF)						//pins 80..96
	{GPIOF, 1<<0},						//STM32duino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOF, 1<<1},						//STM32duino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOF, 1<<2},						//STM32duino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOF, 1<<3},						//STM32duino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOF, 1<<4},						//STM32duino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOF, 1<<5},						//STM32duino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOF, 1<<6},						//STM32duino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOF, 1<<7},						//STM32duino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOF, 1<<8},						//STM32duino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOF, 1<<9},						//STM32duino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOF, 1<<10},						//STM32duino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOF, 1<<11},						//STM32duino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOF, 1<<12},						//STM32duino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOF, 1<<13},						//STM32duino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOF, 1<<14},						//STM32duino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOF, 1<<15},						//STM32duino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOF
};

//set up core timer
//global variables

//for time base off TIMER1 @ 1:1 prescaler
//volatile uint32_t timer1_millis = 0;
volatile uint32_t timer_ticks = 0;
//static uint16_t timer1_fract = 0;

//timer1 overflow isr
//void _ISR_PSV _T2Interrupt(void) {
//	IFS0bits.T2IF=0;					//clear tmr1 interrupt flag
//	timer_ticks+=0x10000ul;				//increment overflow count: 16-bit timer
//}

//configure gpio DDR mode (cnf1..0 + mod1..0 bits)
void GPIO_DDR(GPIO_TypeDef * GPIOx, uint32_t mask, uint32_t mode) {
	uint8_t pos=0, posx2;
	//uint32_t pin_mask;

	while (mask) {								//for (pos=0; pos < 16; pos++) {
		//looking for pin position / mask
		//pin_mask = 1ul << pos;
		if (mask & (1ul << 0)) {				//if (mask & (1ul << pos)) {
			posx2=pos * 2;
			//we have found the pos / pin_mask
			if ((mode & GPIOMODE_OUTPUT) || (mode & GPIOMODE_AF)) {
				GPIOx->OSPEEDR &=~(0x03ul << (posx2));	//clear ospeeder
				GPIOx->OSPEEDR |= (0x01ul << (posx2));	//set to medium speed (0x01)

				GPIOx->OTYPER &=~(1ul << pos);				//clear otyper
				GPIOx->OTYPER |= ((mode & GPIOMODE_PP)?0ul:1ul) << pos;	//0->pp, 1->od
			}

			GPIOx->MODER &=~(0x03 << (2 * pos));			//clear moder
			GPIOx->MODER |= (mode & 0x03) << (posx2);		//set moder

			GPIOx->PUPDR &=~(0x03 << (posx2));			//clear pupdr
			GPIOx->PUPDR |= ((mode >> 4) & 0x03) << (posx2);	//set pupdr
		}
		mask = mask >> 1;
		pos = pos + 1;
	}
}

//global variables
//for time base off SysTick (24-bit counter)
volatile uint32_t systickovf_counter = 1ul<<24;						//time base on Systick -> SysTick->VAL being the lowest 24-bits (SysTick is a downcounter)

//systick handler - to provide time base for millis()/micros()
void SysTick_Handler(void) {
	//clear the flag
	systickovf_counter += 1ul<<24;						//increment systick counter - 24bit, 1:1 prescaler
}

//reset the mcu
void mcu_init(void) {
	//select the clock source
	//or use default clock = 8Mhz FRC
	//SystemCoreClockHSEPLL(RCC_CFGR_PLLMULL12);
	//SystemCoreClockHSE();
	//SystemCoreClockHSI();
	//SystemCoreClockHSIPLL_32Mhz();				//HSIPLL to 32Mhz

	//enable clock to GPIO

	RCC->AHBENR |=
			RCC_AHBENR_GPIOAEN |
			RCC_AHBENR_GPIOBEN |
			RCC_AHBENR_GPIOCEN |
			RCC_AHBENR_GPIODEN |
#if defined(RCC_AHBENR_GPIOEEN)
			RCC_AHBENR_GPIOEEN |
#endif
#if defined(RCC_AHBENR_GPIOFEN)
			RCC_AHBENR_GPIOFEN |
#endif
#if defined(RCC_AHBENR_GPIOGEN)
			RCC_AHBENR_GPIOGEN |
#endif
			0x00;
	//update SystemCoreClock - it must be the last step before exiting mcu_init
	SystemCoreClockUpdate();
	
	//start systick / coretick
	//configure Systick as the time base for millis()/micros()

	systickovf_counter = 1ul<<24;											//SysTick is a 24-bit downcounter
	//for chips where SysTick_Config() is not defined in cmsis
	SysTick->LOAD  = 	(systickovf_counter-1)/*ticks*/ & SysTick_LOAD_RELOAD_Msk;      /* set reload register */
	NVIC_SetPriority 	(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Systick Interrupt */
	SysTick->VAL   = 	0;                                          /* Load the SysTick Counter Value */
	SysTick->CTRL  = 	SysTick_CTRL_CLKSOURCE_Msk |
						SysTick_CTRL_TICKINT_Msk   |
						SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */

	//alternative - for CMSIS-equip'd chips
	//SysTick_Config(SysTick_LOAD_RELOAD_Msk);			//reload all 24 bits

	//enable global interrupts
	ei();										//testing

}

//return timer clicks
//execution time = 30 ticks, no optimization
uint32_t systicks(void) {
	uint32_t m;
	uint32_t f;

	//do a double read
	do {
		m = systickovf_counter;			//read the overflow counter
		f = SysTick->VAL;				//read the least significant 16-bits
	} while (m != systickovf_counter);	//guard against overflow

	return (m - f) << 0;				//systick is a 24-bit downcounter
}

//empty interrupt handler
void empty_handler(void) {
	//do nothing here
}

//C main loop
int main(void) {

	mcu_init();						//reset the mcu
	setup();						//run the setup code
	while (1) {
		loop();						//run the default loop
	}
}


//Arduino Functions: GPIO
//set a pin mode to INPUT or OUTPUT
//no error checking on PIN
inline void pinMode(PIN_TypeDef pin, uint8_t mode) {
	if (mode==INPUT) GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//set / clear a pin
inline void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	if (val==LOW) GIO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//read a pin
inline int digitalRead(PIN_TypeDef pin) {
	return (GIO_GET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask))?HIGH:LOW;
}
//end GPIO

//ticks()
//Arduino Functions: Time
//return timer ticks
	
//delay millisseconds
void delay(uint32_t ms) {
	uint32_t start_time = ticks();
	ms *= cyclesPerMillisecond();
	while (ticks() - start_time < ms) continue;
}

//delay micros seconds
void delayMicroseconds(uint32_t us) {
	uint32_t start_time = ticks();
	us *= cyclesPerMicrosecond();
	while (ticks() - start_time < us) continue;
}
//end Time

//uart1
//initialize usart: high baudrate (brgh=1), 16-bit baudrate (brg16=1)
//baudrate=Fxtal/(4*(spbrg+1))
//spbrg=(Fxtal/4/baudrate)-1
//tx/rx pins to be assumed in gpio mode
//data bits: 	8
//parity: 		none
//stop bits: 	1
//Xon/Xoff:		none
void uart1Init(unsigned long baudrate) {
	uint16_t uartdiv;

	//configure uart1
    //route clock to uart1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    USART1->CR1 &=~(1<<0);			//'0'->disable uart, '1'->enable uart
    USART1->CR1 =	(0<<28) | (0<<12) |	//0b00->1 start bit, 8 data bits, n stop bit; 0b01->1 start bit, 9 data bits, n stop bit, 0b10->1 start bit 7 data bits, n stop bit
    				(0<<27) |		//0->disable end of block interrupt
    				(0<<26) |		//0->receiver timeout interrupt disabled
    				(0x00<<21) |	//0b00000->driver enable assertion time
    				(0x00<<16) |	//0b00000->driver enable disassertion time
    				(0<<15) |		//0->oversampling by 16
    				(0<<14) |		//0->character match interrupt disabled
    				(0<<13) |		//0->receiver in active mode permanently
    				//bit 12 set earlier
    				(0<<11) |		//0->idle line to wake up uart
    				(0<<10) |		//0->no parity
    				(0<<9) |		//0->even parity
    				(0<<8) |		//0->disable PE interrupt
    				(0<<7) |		//0->disable txie)
    				(0<<6) |		//0->disable transmission complete interrupt
    				(0<<5) |		//0->disable receiver buffer not empty interrupt
    				(0<<4) |		//0->disable idle interrupt
    				(1<<3) |		//0->transmitter disabled, 1->transmitter enabled
    				(1<<2) |		//0->receiver disabled, 1->receiver enabled
    				//bit 1 reserved
    				(0<<0) |		//0->disable uart, 1->enable uart
    				0x00;
    USART1->CR2 = 	(0x00<<28) |	//address of the uart
    				(0x00<<24) |	//address of the uart
    				(0<<23) |		//0->disable receiver time out
    				(0x00<<21) |	//00->measurement of the start bit used to detect baud rate
    				(0<<20) |		//auto baud rate disabled
    				(0<<19) |		//0->data bit 0 first
    				(0<<18) |		//0->data transmitted / received in positive logic
    				(0<<17) |		//0->tx in positive logic
    				(0<<16) |		//0->rx in positive logic
    				(0<<15) |		//0->tx/rx pins not swapped, 1->tx/rx pins swapped
    				(0x00<<12) |	//00->1 stop bit, 10->2 stop bit, 11->1.5 stop bit
    				(0<<11) |		//0->sclk disabled
    				(0<<10) |		//0->sclk idles low
    				(0<<9) |		//0->clock on first data capture
    				(0<<8) |		//0->clock on the last bit is not data pulse
    				(0<<4) |		//0->4 bit address detection
    				0x00;
    USART1->CR3 =	(0<<15) |		//0->driver enable signal active high
    				(0<<14) |		//0->disable driver more
    				0x00;			//reset value
    //set the baudrate register
    uartdiv = F_UART / baudrate;
    if (USART1->CR1 & (1<<15)) {		//oversample by 8
    	uartdiv = uartdiv * 2;
    	uartdiv = 	(uartdiv &~0x000f) |	//clear lowest 4 bits
    				(1<<3) |			//bit 3 is always set
    				((uartdiv >> 1) & 0x07);	//keep the lowest 3 bits
    }
    USART1->BRR = uartdiv;
    //USART1->BRR = F_UART / baudrate * ((USART1->CR1 & (1<<15))?2:1);		//per datasheet, for OVER8=0 or 1

    //USART1->DR = 0;					//reset the data register
    //USART1->SR = 0;					//reset the data register

    //enable uart1
    USART1->CR1 |= (1<<0);			//'0'->disable uart, '1'->enable uart

    //configure the TX-PA9/RX-PA10 pins - GPIO clock assumed enabled here previously
    //RX as floating input/AF input, AF1
    IO_INFL(GPIOA, 1<<10); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * (10-8)))) | (1<<(4 * (10-8)));
	//TX as AFPP, AF1
    IO_AFPP(GPIOA, 1<< 9); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * ( 9-8)))) | (1<<(4 * ( 9-8)));

}

void uart1Putch(char ch) {
    //while (!(USART1->SR & USART_SR_TXE));    	//wait for the transmission buffer to be empty
    while (uart1Busy()) continue;    			//wait for the transmission buffer to be empty
    USART1->TDR = ch;                        	//load the data buffer
    //while (!(USART1->SR & (1<<6)));    		//wait for the transmission to complete
}

//put a string
void uart1Puts(char *str) {
	while (*str) uart1Putch(*str++);
}

/*

Writes a line of text to USART and goes to new line
The new line is Windows style CR/LF pair.

This will work on Hyper Terminal Only NOT on Linux

*/

//put a line termined with ln
void uart1Putline(char *ln) {
	//USARTWriteString(ln);
	uart1Puts(ln);
	//USARTWriteString("\r\n");
	uart1Puts((char *)"\r\n");
}

//get the received char
uint8_t uart1Getch(void) {
	   //while (!(USART1->SR & USART_SR_RXNE));  	//wait for the receipt buffer to be empty
	    return USART1->RDR;                       	//save the transmission buffer
}

//test if data rx is available
uint16_t uart1Available(void) {
	//return (USART1->SR & USART_SR_TC)?true:false;
	return (USART1->ISR & USART_ISR_RXNE);
}

//test if uart tx is busy
uint16_t uart1Busy(void) {
    return !(USART1->ISR & USART_ISR_TXE);    	//return 1 if TX buffer is empty
}

#if defined(USART2)
//uart2
//initialize usart: high baudrate (brgh=1), 16-bit baudrate (brg16=1)
//baudrate=Fxtal/(4*(spbrg+1))
//spbrg=(Fxtal/4/baudrate)-1
//tx/rx pins to be assumed in gpio mode
//data bits: 	8
//parity: 		none
//stop bits: 	1
//Xon/Xoff:		none
void uart2Init(unsigned long baudrate) {
	uint16_t uartdiv;

	//configure uart1
    //route clock to uart1
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->CR1 &=~(1<<0);			//'0'->disable uart, '1'->enable uart
    USART2->CR1 =	(0<<28) | (0<<12) |	//0b00->1 start bit, 8 data bits, n stop bit; 0b01->1 start bit, 9 data bits, n stop bit, 0b10->1 start bit 7 data bits, n stop bit
    				(0<<27) |		//0->disable end of block interrupt
    				(0<<26) |		//0->receiver timeout interrupt disabled
    				(0x00<<21) |	//0b00000->driver enable assertion time
    				(0x00<<16) |	//0b00000->driver enable disassertion time
    				(0<<15) |		//0->oversampling by 16
    				(0<<14) |		//0->character match interrupt disabled
    				(0<<13) |		//0->receiver in active mode permanently
    				//bit 12 set earlier
    				(0<<11) |		//0->idle line to wake up uart
    				(0<<10) |		//0->no parity
    				(0<<9) |		//0->even parity
    				(0<<8) |		//0->disable PE interrupt
    				(0<<7) |		//0->disable txie)
    				(0<<6) |		//0->disable transmission complete interrupt
    				(0<<5) |		//0->disable receiver buffer not empty interrupt
    				(0<<4) |		//0->disable idle interrupt
    				(1<<3) |		//0->transmitter disabled, 1->transmitter enabled
    				(1<<2) |		//0->receiver disabled, 1->receiver enabled
    				//bit 1 reserved
    				(0<<0) |		//0->disable uart, 1->enable uart
    				0x00;
    USART2->CR2 = 	(0x00<<28) |	//address of the uart
    				(0x00<<24) |	//address of the uart
    				(0<<23) |		//0->disable receiver time out
    				(0x00<<21) |	//00->measurement of the start bit used to detect baud rate
    				(0<<20) |		//auto baud rate disabled
    				(0<<19) |		//0->data bit 0 first
    				(0<<18) |		//0->data transmitted / received in positive logic
    				(0<<17) |		//0->tx in positive logic
    				(0<<16) |		//0->rx in positive logic
    				(0<<15) |		//0->tx/rx pins not swapped, 1->tx/rx pins swapped
    				(0x00<<12) |	//00->1 stop bit, 10->2 stop bit, 11->1.5 stop bit
    				(0<<11) |		//0->sclk disabled
    				(0<<10) |		//0->sclk idles low
    				(0<<9) |		//0->clock on first data capture
    				(0<<8) |		//0->clock on the last bit is not data pulse
    				(0<<4) |		//0->4 bit address detection
    				0x00;
    USART2->CR3 =	(0<<15) |		//0->driver enable signal active high
    				(0<<14) |		//0->disable driver more
    				0x00;			//reset value
    //set the baudrate register
    uartdiv = F_UART / baudrate;
    if (USART2->CR1 & (1<<15)) {		//oversample by 8
    	uartdiv = uartdiv * 2;
    	uartdiv = 	(uartdiv &~0x000f) |	//clear lowest 4 bits
    				(1<<3) |			//bit 3 is always set
    				((uartdiv >> 1) & 0x07);	//keep the lowest 3 bits
    }
    USART2->BRR = uartdiv;
    //USART2->BRR = F_UART / baudrate * ((USART2->CR1 & (1<<15))?2:1);		//per datasheet, for OVER8=0 or 1

    //USART2->DR = 0;					//reset the data register
    //USART2->SR = 0;					//reset the data register

    //enable uart1
    USART2->CR1 |= (1<<0);			//'0'->disable uart, '1'->enable uart

    //configure the TX-PA9/RX-PA10 pins - GPIO clock assumed enabled here previously
    //RX as floating input/AF input, AF1
    IO_INFL(GPIOA, 1<<10); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * (10-8)))) | (1<<(4 * (10-8)));
	//TX as AFPP, AF1
    IO_AFPP(GPIOA, 1<< 9); GPIOA->AFR[1] = (GPIOA->AFR[1] &~(0x0f << (4 * ( 9-8)))) | (1<<(4 * ( 9-8)));

}

void uart2Putch(char ch) {
	   //while (!(USART2->SR & USART_SR_TXE));    	//wait for the transmission buffer to be empty
	    while (uart2Busy()) continue;    			//wait for the transmission buffer to be empty
	    USART2->TDR = ch;                        	//load the data buffer
	    //while (!(USART2->SR & (1<<6)));    		//wait for the transmission to complete
}

void uart2Puts(char *str) {
	while (*str) uart2Putch(*str++);
}

/*

Writes a line of text to USART and goes to new line
The new line is Windows style CR/LF pair.

This will work on Hyper Terminal Only NOT on Linux

*/

void uart2Putline(char *ln) {
	//USARTWriteString(ln);
	uart2Puts(ln);
	//USARTWriteString("\r\n");
	uart2Puts((char *)"\r\n");
}

uint8_t uart2Getch(void) {
	   //while (!(USART2->SR & USART_SR_RXNE));  	//wait for the receipt buffer to be empty
	    return USART2->RDR;                       	//save the transmission buffer
}

//test if data rx is available
uint16_t uart2Available(void) {
	//return (USART2->SR & USART_SR_TC)?true:false;
	return (USART2->ISR & USART_ISR_RXNE);
}

//test if uart tx is busy
uint16_t uart2Busy(void) {
	   return !(USART2->ISR & USART_ISR_TXE);    	//return 1 if TX buffer is empty
}
#endif		//USART2
//end Serial


//tmr1
//global variables
static void (* _tim1_oc1isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_oc2isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_oc3isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default
static void (* _tim1_oc4isrptr)(void)=empty_handler;				//tim1_ptr pointing to empty_handler by default

static uint16_t _tim1_oc1=0;				//output compare registers
static uint16_t _tim1_oc2=0;
static uint16_t _tim1_oc3=0;
static uint16_t _tim1_oc4=0;

//isr for timer1 capture / compare
void TIM1_CC_IRQHandler(void) {
	//oc1 portion
	if (TIM1->SR & TIM_SR_CC1IF) {		//output compare 1 flag is set
		TIM1->SR &=~TIM_SR_CC1IF;		//clear the flag
		TIM1->CCR1 += _tim1_oc1;			//update the output compare register
		_tim1_oc1isrptr();				//execute user handler
	}

	//oc2 portion
	if (TIM1->SR & TIM_SR_CC2IF) {		//output compare 2 flag is set
		TIM1->SR &=~TIM_SR_CC2IF;		//clear the flag
		TIM1->CCR2 += _tim1_oc2;			//update the output compare register
		_tim1_oc2isrptr();				//execute user handler
	}

	//oc3 portion
	if (TIM1->SR & TIM_SR_CC3IF) {		//output compare 2 flag is set
		TIM1->SR &=~TIM_SR_CC3IF;		//clear the flag
		TIM1->CCR3 += _tim1_oc3;			//update the output compare register
		_tim1_oc3isrptr();				//execute user handler
	}

	//oc4 portion
	if (TIM1->SR & TIM_SR_CC4IF) {		//output compare 2 flag is set
		TIM1->SR &=~TIM_SR_CC4IF;		//clear the flag
		TIM1->CCR4 += _tim1_oc4;			//update the output compare register
		_tim1_oc4isrptr();				//execute user handler
	}
}

//initialize the timer1 (16bit)
void tmr1Init(uint16_t ps) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	//source from internal clock -> disable slave mode
	TIM1->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM1->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM1->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM1->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM1->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM1->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM1->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
	//disable the interrupt by clearing the enable bits
	TIM1->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE);

	//set the prescaler
	TIM1->PSC = ps - 1;			//set the prescaler
	TIM1->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM1->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM1->CNT = 0;						//reset the counter

	//enable the timer.
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim1_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr1SetPR1(uint16_t pr) {
	//save the period value
	_tim1_oc1 = pr - 0;
	TIM1->CCR1 = _tim1_oc1;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr1AttachISR1(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_oc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr1SetPR2(uint16_t pr) {
	//save the period value
	_tim1_oc2 = pr - 0;
	TIM1->CCR2 = _tim1_oc2;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr1AttachISR2(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_oc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr1SetPR3(uint16_t pr) {
	//save the period value
	_tim1_oc3 = pr - 0;
	TIM1->CCR3 = _tim1_oc3;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr1AttachISR3(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_oc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//set tim1_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr1SetPR4(uint16_t pr) {
	//save the period value
	_tim1_oc4 = pr - 0;
	TIM1->CCR4 = _tim1_oc4;

	//clear the flag
	//TIM1->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM1->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr1AttachISR4(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM1_CC_IRQn);		//disable irq

	_tim1_oc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM1->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM1->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM1_CC_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//tmr3
//global variables
static void (* _tim3_oc1isrptr)(void)=empty_handler;				//TIM3_ptr pointing to empty_handler by default
static void (* _tim3_oc2isrptr)(void)=empty_handler;				//TIM3_ptr pointing to empty_handler by default
static void (* _tim3_oc3isrptr)(void)=empty_handler;				//TIM3_ptr pointing to empty_handler by default
static void (* _tim3_oc4isrptr)(void)=empty_handler;				//TIM3_ptr pointing to empty_handler by default

static uint16_t _tim3_oc1=0;				//output compare registers
static uint16_t _tim3_oc2=0;
static uint16_t _tim3_oc3=0;
static uint16_t _tim3_oc4=0;

//isr for timer1 capture / compare
void TIM3_IRQHandler(void) {
	//oc1 portion
	if (TIM3->SR & TIM_SR_CC1IF) {		//output compare 1 flag is set
		TIM3->SR &=~TIM_SR_CC1IF;		//clear the flag
		TIM3->CCR1 += _tim3_oc1;		//update the output compare register
		_tim3_oc1isrptr();				//execute user handler
	}

	//oc2 portion
	if (TIM3->SR & TIM_SR_CC2IF) {		//output compare 2 flag is set
		TIM3->SR &=~TIM_SR_CC2IF;		//clear the flag
		TIM3->CCR2 += _tim3_oc2;		//update the output compare register
		_tim3_oc2isrptr();				//execute user handler
	}

	//oc3 portion
	if (TIM3->SR & TIM_SR_CC3IF) {		//output compare 2 flag is set
		TIM3->SR &=~TIM_SR_CC3IF;		//clear the flag
		TIM3->CCR3 += _tim3_oc3;		//update the output compare register
		_tim3_oc3isrptr();				//execute user handler
	}

	//oc4 portion
	if (TIM3->SR & TIM_SR_CC4IF) {		//output compare 2 flag is set
		TIM3->SR &=~TIM_SR_CC4IF;		//clear the flag
		TIM3->CCR4 += _tim3_oc4;		//update the output compare register
		_tim3_oc4isrptr();				//execute user handler
	}
}

//initialize the timer1 (16bit)
void tmr3Init(uint16_t ps) {
	//route the clock to timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//source from internal clock -> disable slave mode
	TIM3->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM3->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM3->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM3->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM3->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM3->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM3->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
	//disable the interrupt by clearing the enable bits
	TIM3->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE);

	//set the prescaler
	TIM3->PSC = ps - 1;					//set the prescaler
	TIM3->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM3->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM3->CNT = 0;						//reset the counter

	//enable the timer.
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set TIM3_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3SetPR1(uint16_t pr) {
	//save the period value
	_tim3_oc1 = pr - 0;
	TIM3->CCR1 = _tim3_oc1;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr3AttachISR1(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);			//disable irq

	_tim3_oc1isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC1IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM3_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3SetPR2(uint16_t pr) {
	//save the period value
	_tim3_oc2 = pr - 0;
	TIM3->CCR2 = _tim3_oc2;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr3AttachISR2(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);			//disable irq

	_tim3_oc2isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC2IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC2IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM3_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3SetPR3(uint16_t pr) {
	//save the period value
	_tim3_oc3 = pr - 0;
	TIM3->CCR3 = _tim3_oc3;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr3AttachISR3(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);			//disable irq

	_tim3_oc3isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC3IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC3IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//set TIM3_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr3SetPR4(uint16_t pr) {
	//save the period value
	_tim3_oc4 = pr - 0;
	TIM3->CCR4 = _tim3_oc4;

	//clear the flag
	//TIM3->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM3->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//activate the isr handler
void tmr3AttachISR4(void (*isrptr)(void)) {
	NVIC_DisableIRQ(TIM3_IRQn);			//disable irq

	_tim3_oc4isrptr = isrptr;			//install user handler

	//clear the flag
	TIM3->SR &=~TIM_SR_CC4IF;			//clear the interrupt flag
	TIM3->DIER |= TIM_DIER_CC4IE;		//enable the isr

	NVIC_EnableIRQ(TIM3_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//global variables
static void (* _tim14_isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

//isr for timer14
void TIM14_IRQHandler(void) {
	TIM14->SR &=~TIM_SR_UIF;			//clear the flag
	_tim14_isrptr();					//execute user handler
}

//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tmr14Init(uint16_t ps) {
	//route the clock to timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	//source from internal clock -> disable slave mode
	TIM14->SMCR &=~TIM_SMCR_SMS;		//clear sms->use internal clock

	//stop the timer to configure it
	TIM14->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM14->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM14->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM14->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM14->CR1 = 0;					//much easier

	//clear the status register bits for capture / compare flags
	TIM14->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM14->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//set the prescaler
	TIM14->PSC = ps - 1;				//set the prescaler
	TIM14->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM14->ARR = -1;					//auto reload register / period = 0; - need to change for downcounters
	TIM14->CNT = 0;						//reset the counter

	//enable the timer.
	TIM14->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr14SetPR(uint16_t pr) {
	//save the period value
	TIM14->ARR = pr - 1;

	//clear the flag
	//TIM14->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM14->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tmr14AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM14_IRQn);		//disable irq

	_tim14_isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM14->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM14->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM14_IRQn);			//enable irq
	//priorities not set -> default values used.
}

//global variables
static void (* _tim16_isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

//isr for timer1 capture / compare
void TIM16_IRQHandler(void) {
	TIM16->SR &=~TIM_SR_UIF;		//clear the flag
	_tim16_isrptr();				//execute user handler
}

//initialize tim4 to use compare channels as timers
//16-bit prescaler. 32-bit used for compatability
void tmr16Init(uint16_t ps) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	//source from internal clock -> disable slave mode
	TIM16->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM16->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM16->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM16->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM16->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM16->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM16->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM16->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//set the prescaler
	TIM16->PSC = ps - 1;					//set the prescaler
	TIM16->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM16->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM16->CNT = 0;						//reset the counter

	//enable the timer.
	TIM16->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim4_oc1 period
//pr is 16-bit. 32-bit used for compatability;
void tmr16SetPR(uint16_t pr) {
	//save the period value
	TIM16->ARR = pr - 1;

	//clear the flag
	//TIM16->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM16->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tmr16AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM16_IRQn);		//disable irq

	_tim16_isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM16->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM16->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM16_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//global variables
static void (* _tim17_isrptr)(void)=empty_handler;				//tim4_ptr pointing to empty_handler by default

//isr for timer1 capture / compare
void TIM17_IRQHandler(void) {
	TIM17->SR &=~TIM_SR_UIF;		//clear the flag
	_tim17_isrptr();				//execute user handler
}

//initialize tim4 to use compare channels as timers
//17-bit prescaler. 32-bit used for compatability
void tmr17Init(uint16_t ps) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;

	//source from internal clock -> disable slave mode
	TIM17->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	TIM17->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	TIM17->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	TIM17->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	TIM17->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM17->CR1 = 0;						//much easier

	//clear the status register bits for capture / compare flags
	TIM17->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM17->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//set the prescaler
	TIM17->PSC = ps - 1;					//set the prescaler
	TIM17->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM17->ARR = -1;						//auto reload register / period = 0; - need to change for downcounters
	TIM17->CNT = 0;						//reset the counter

	//enable the timer.
	TIM17->CR1 |= TIM_CR1_CEN;			//enable the timer
}

//set tim4_oc1 period
//pr is 17-bit. 32-bit used for compatability;
void tmr17SetPR(uint16_t pr) {
	//save the period value
	TIM17->ARR = pr - 1;

	//clear the flag
	//TIM17->SR &=~TIM_SR_CC1IF;			//clear the interrupt flag
	//TIM17->DIER &=~TIM_DIER_CC1IE;		//disable the isr
}

//install user handler
void tmr17AttachISR(void (*isr_ptr)(void)) {
	NVIC_DisableIRQ(TIM17_IRQn);		//disable irq

	_tim17_isrptr = isr_ptr;			//install user handler

	//clear the flag
	TIM17->SR &=~TIM_SR_UIF;			//clear the interrupt flag
	TIM17->DIER |= TIM_DIER_UIE;		//enable the isr

	NVIC_EnableIRQ(TIM17_IRQn);		//enable irq
	//priorities not set -> default values used.
}

//end Timer

/*
MAPR Bits 7:6 TIM1_REMAP[1:0]: TIM1 remapping
These bits are set and cleared by software. They control the mapping of TIM1 channels 1 to 4, 1N to 3N, external trigger (ETR) and Break input (BKIN) on the GPIO ports.
00: No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15)
01: Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1)
10: not used
11: Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12)
*/
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm1Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	//source from internal clock -> disable slave mode
	TIM1->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	//TIM1->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	//TIM1->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	//TIM1->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	//TIM1->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM1->CR1 = 0;						//much easier
	TIM1->CR1 =	(0<<8) |				//0->1:1 clock, 1->2:1 clock, 2->4:1 clock, 3->reserved
				(0<<7) |				//1->APR buffered, 0->APR not buffered
				(0<<5) |				//0->edge-aligned, 1->center-aligned mode 1, 2->center-aligned mode 2, 3->center-aligned mode 3
				(0<<4) |				//0->upcounter, 1->downcounter
				(0<<3) |				//0->continous mode, 1->one pulse mode
				(0<<2) |				//update request source
				(0<<1) |				//0->UEV enabled, 1->UEV disabled
				(0<<0) |				//0->counter disabled, 1->counter enabled
				0x00;
	TIM1->CR2 = 0;						//default value
	TIM1->SMCR= 0;						//default value

	//set the prescaler
	TIM1->PSC = TxCCP_PS - 1;					//set the prescaler
	TIM1->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM1->ARR = PWM_PR;						//auto reload register / period = 0; - need to change for downcounters
	TIM1->CNT = 0;						//reset the counter

	//clear the status register bits for capture / compare flags
	TIM1->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM1->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//configure CCP1..4
#if defined(TIM1CH1toGPIO)
	//configure CCP1
	TIM1->CCMR1 = 	(TIM1->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM1->CCER = 	(TIM1->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM1->CCR1 = 0;						//0% duty cycle

	//configure gpio
	TIM1CH1toGPIO();					//route TIM1CH1 to GPIO
#endif

#if defined(TIM1CH2toGPIO)
	//configure CCP2
	TIM1->CCMR1 = 	(TIM1->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM1->CCER = 	(TIM1->CCER &~(0x0f << 4)) |
					(0<< 7) |			//0->normal polarity for CC2N, 1->inverse polarity
					(0<< 6) |			//0->disable CC2N, 1->enable CC2N
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM1->CCR2 = 0;						//0% duty cycle

	//configure gpio
	TIM1CH2toGPIO();					//route TIM1CH1 to GPIO
#endif

#if defined(TIM1CH3toGPIO)
	//configure CCP3
	TIM1->CCMR2 = 	(TIM1->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM1->CCER = 	(TIM1->CCER &~(0x0f << 8)) |
					(0<<11) |			//0->normal polarity for CC3N, 1->inverse polarity
					(0<<10) |			//0->disable CC3N, 1->enable CC3N
					(0<< 9) |			//0->normal polarity for CC3, 1->inverse polarity
					(1<< 8) |			//1->enable CC3, 0->disable CC3
					0x00;
	TIM1->CCR3 = 0;						//0% duty cycle

	//configure gpio
	TIM1CH3toGPIO();					//route TIM1CH1 to GPIO
#endif

#if defined(TIM1CH4toGPIO)
	//configure CCP4
	TIM1->CCMR2 = 	(TIM1->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM1->CCER = 	(TIM1->CCER &~(0x0f << 12)) |
					(0<<15) |			//0->normal polarity for CC4N, 1->inverse polarity
					(0<<14) |			//0->disable CC4N, 1->enable CC4N
					(0<<13) |			//0->normal polarity for CC4, 1->inverse polarity
					(1<<12) |			//1->enable CC4, 0->disable CC4
					0x00;
	TIM1->CCR4 = 0;						//0% duty cycle

	//configure gpio
	TIM1CH4toGPIO();					//route TIM1CH1 to GPIO
#endif

	TIM1->EGR = 0xff;					//force an update
	TIM1->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM1->CR1 |= TIM_CR1_CEN;			//enable the timer

}


//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm3Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//source from internal clock -> disable slave mode
	//TIM3->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	//TIM3->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	//TIM3->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	//TIM3->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	//TIM3->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM3->CR1 = 0;						//much easier
	TIM3->CR1 =	(0<<8) |				//0->1:1 clock, 1->2:1 clock, 2->4:1 clock, 3->reserved
				(0<<7) |				//1->APR buffered, 0->APR not buffered
				(0<<5) |				//0->edge-aligned, 1->center-aligned mode 1, 2->center-aligned mode 2, 3->center-aligned mode 3
				(0<<4) |				//0->upcounter, 1->downcounter
				(0<<3) |				//0->continous mode, 1->one pulse mode
				(0<<2) |				//update request source
				(0<<1) |				//0->UEV enabled, 1->UEV disabled
				(0<<0) |				//0->counter disabled, 1->counter enabled
				0x00;
	TIM3->CR2 = 0;						//default value
	TIM3->SMCR= 0;						//default value - use internal clock

	//set the prescaler
	TIM3->PSC = TxCCP_PS - 1;					//set the prescaler
	TIM3->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM3->ARR = PWM_PR;						//auto reload register / period = 0; - need to change for downcounters
	TIM3->CNT = 0;						//reset the counter

	//clear the status register bits for capture / compare flags
	TIM3->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM3->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//configure CCP1..4
#if defined(TIM3CH1toGPIO)
	//configure CCP1
	TIM3->CCMR1 = 	(TIM3->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM3->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
	TIM3CH1toGPIO();
#endif

#if defined(TIM3CH2toGPIO)
	//configure CCP2
	TIM3->CCMR1 = 	(TIM3->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 4)) |
					(0<< 7) |			//0->normal polarity for CC2N, 1->inverse polarity
					(0<< 6) |			//0->disable CC2N, 1->enable CC2N
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM3->CCR2 = 0;						//0% duty cycle

	//configure the gpio for PWM output
	TIM3CH2toGPIO();
#endif

#if defined(TIM3CH3toGPIO)
	//configure CCP3
	TIM3->CCMR2 = 	(TIM3->CCMR2 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 8)) |
					(0<<11) |			//0->normal polarity for CC3N, 1->inverse polarity
					(0<<10) |			//0->disable CC3N, 1->enable CC3N
					(0<< 9) |			//0->normal polarity for CC3, 1->inverse polarity
					(1<< 8) |			//1->enable CC3, 0->disable CC3
					0x00;
	TIM3->CCR3 = 0;						//0% duty cycle

	//configure the gpio for PWM output
	TIM3CH3toGPIO();
#endif

#if defined(TIM3CH4toGPIO)
	//configure CCP4
	TIM3->CCMR2 = 	(TIM3->CCMR2 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM3->CCER = 	(TIM3->CCER &~(0x0f << 12)) |
					(0<<15) |			//0->normal polarity for CC4N, 1->inverse polarity
					(0<<14) |			//0->disable CC4N, 1->enable CC4N
					(0<<13) |			//0->normal polarity for CC4, 1->inverse polarity
					(1<<12) |			//1->enable CC4, 0->disable CC4
					0x00;
	TIM3->CCR4 = 0;						//0% duty cycle

	//configure the gpio for PWM output
	TIM3CH4toGPIO();
#endif

	TIM3->EGR = 0xff;					//force an update
	TIM3->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM3->CR1 |= TIM_CR1_CEN;			//enable the timer

}


//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm14Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	//source from internal clock -> disable slave mode
	//TIM14->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	//TIM14->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	//TIM14->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	//TIM14->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	//TIM14->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM14->CR1 = 0;						//much easier
	TIM14->CR1 =	(0<<8) |				//0->1:1 clock, 1->2:1 clock, 2->4:1 clock, 3->reserved
				(0<<7) |				//1->APR buffered, 0->APR not buffered
				(0<<5) |				//0->edge-aligned, 1->center-aligned mode 1, 2->center-aligned mode 2, 3->center-aligned mode 3
				(0<<4) |				//0->upcounter, 1->downcounter
				(0<<3) |				//0->continous mode, 1->one pulse mode
				(0<<2) |				//update request source
				(0<<1) |				//0->UEV enabled, 1->UEV disabled
				(0<<0) |				//0->counter disabled, 1->counter enabled
				0x00;
	TIM14->CR2 = 0;						//default value
	TIM14->SMCR= 0;						//default value - use internal clock

	//set the prescaler
	TIM14->PSC = TxCCP_PS - 1;			//set the prescaler
	TIM14->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM14->ARR = PWM_PR;				//auto reload register / period = 0; - need to change for downcounters
	TIM14->CNT = 0;						//reset the counter

	//clear the status register bits for capture / compare flags
	TIM14->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM14->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//configure CCP1..4
#if defined(TIM14CH1toGPIO)
	//configure CCP1
	TIM14->CCMR1 = 	(TIM14->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM14->CCER = 	(TIM14->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM14->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
	TIM14CH1toGPIO();
#endif

	TIM14->EGR = 0xff;					//force an update
	TIM14->BDTR|= TIM_BDTR_MOE;			//enable the main output
	//enable the timer.
	TIM14->CR1 |= TIM_CR1_CEN;			//enable the timer

}


//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm15Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

	//source from internal clock -> disable slave mode
	//TIM15->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	//TIM15->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	//TIM15->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	//TIM15->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	//TIM15->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM15->CR1 = 0;						//much easier
	TIM15->CR1 =	(0<<8) |				//0->1:1 clock, 1->2:1 clock, 2->4:1 clock, 3->reserved
				(0<<7) |				//1->APR buffered, 0->APR not buffered
				(0<<5) |				//0->edge-aligned, 1->center-aligned mode 1, 2->center-aligned mode 2, 3->center-aligned mode 3
				(0<<4) |				//0->upcounter, 1->downcounter
				(0<<3) |				//0->continous mode, 1->one pulse mode
				(0<<2) |				//update request source
				(0<<1) |				//0->UEV enabled, 1->UEV disabled
				(0<<0) |				//0->counter disabled, 1->counter enabled
				0x00;
	TIM15->CR2 = 0;						//default value
	TIM15->SMCR= 0;						//default value - use internal clock

	//set the prescaler
	TIM15->PSC = TxCCP_PS - 1;					//set the prescaler
	TIM15->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM15->ARR = PWM_PR;						//auto reload register / period = 0; - need to change for downcounters
	TIM15->CNT = 0;						//reset the counter

	//clear the status register bits for capture / compare flags
	TIM15->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM15->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//configure CCP1..4
#if defined(TIM15CH1toGPIO)
	//configure CCP1
	TIM15->CCMR1 = 	(TIM15->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM15->CCER = 	(TIM15->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM15->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
	TIM15CH1toGPIO();
#endif

#if defined(TIM15CH2toGPIO)
	//configure CCP2
	TIM15->CCMR1 = 	(TIM15->CCMR1 &~0xff00) |
					(0<<15) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<12) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<11) |			//0->preload disabled, 1->preload enabled
					(0<<10) |			//0->fast disabled, 1->fast enabled
					(0<<8) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM15->CCER = 	(TIM15->CCER &~(0x0f << 4)) |
					(0<< 7) |			//0->normal polarity for CC2N, 1->inverse polarity
					(0<< 6) |			//0->disable CC2N, 1->enable CC2N
					(0<< 5) |			//0->normal polarity for CC2, 1->inverse polarity
					(1<< 4) |			//1->enable CC2, 0->disable CC2
					0x00;
	TIM15->CCR2 = 0;						//0% duty cycle

	//configure the gpio for PWM output
	TIM15CH2toGPIO();
#endif

	TIM15->EGR = 0xff;					//force an update
	TIM15->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM15->CR1 |= TIM_CR1_CEN;			//enable the timer

}

/*
MAPR2 Bit 1 TIM16_REMAP: TIM16 remapping
This bit is set and cleared by software. It controls the mapping of the alternate functions of TIM16 channel 1 onto the GPIO ports.
0: No remap (CH1/PB8)
1: Remap (CH1/PA6)
*/
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm16Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	//source from internal clock -> disable slave mode
	//TIM16->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	//TIM16->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	//TIM16->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	//TIM16->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	//TIM16->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM16->CR1 = 0;						//much easier
	TIM16->CR1 =	(0<<8) |				//0->1:1 clock, 1->2:1 clock, 2->4:1 clock, 3->reserved
				(0<<7) |				//1->APR buffered, 0->APR not buffered
				(0<<5) |				//0->edge-aligned, 1->center-aligned mode 1, 2->center-aligned mode 2, 3->center-aligned mode 3
				(0<<4) |				//0->upcounter, 1->downcounter
				(0<<3) |				//0->continous mode, 1->one pulse mode
				(0<<2) |				//update request source
				(0<<1) |				//0->UEV enabled, 1->UEV disabled
				(0<<0) |				//0->counter disabled, 1->counter enabled
				0x00;
	TIM16->CR2 = 0;						//default value
	TIM16->SMCR= 0;						//default value - use internal clock

	//set the prescaler
	TIM16->PSC = TxCCP_PS - 1;					//set the prescaler
	TIM16->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM16->ARR = PWM_PR;						//auto reload register / period = 0; - need to change for downcounters
	TIM16->CNT = 0;						//reset the counter

	//clear the status register bits for capture / compare flags
	TIM16->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM16->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//configure CCP1..4
#if defined(TIM16CH1toGPIO)
	//configure CCP1
	TIM16->CCMR1 = 	(TIM16->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM16->CCER = 	(TIM16->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM16->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
	TIM16CH1toGPIO();
#endif

	TIM16->EGR = 0xff;					//force an update
	TIM16->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM16->CR1 |= TIM_CR1_CEN;			//enable the timer

}

/*
MAPR2 Bit 2 TIM17_REMAP: TIM17 remapping
This bit is set and cleared by software. It controls the mapping of the alternate functions of TIM17 channel 1 onto the GPIO ports.
0: No remap (CH1/PB9)
1: Remap (CH1/PA7)
*/
//initialize pwm to TxCCP_PS (prescaler) and TxCCP_PR (period)
void pwm17Init(uint16_t TxCCP_PS) {
	//route the clock to timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;

	//source from internal clock -> disable slave mode
	//TIM17->SMCR &=~TIM_SMCR_SMS;			//clear sms->use internal clock

	//stop the timer to configure it
	//TIM17->CR1 &=~TIM_CR1_CEN;			//clear cen. 0=disable the timer, 1=enable the timer
	//TIM17->CR1 &=~TIM_CR1_CKD;			//clear CKD0..1. 0b00->1x clock; 0b01->2:1 clock, 0b10->4:1 clk; 0b11->reserved
	//TIM17->CR1 &=~TIM_CR1_DIR;			//clear DIR bit. 0=upcounter, 1=downcounter
	//TIM17->CR1 &=~TIM_CR1_OPM;			//clear opm bit. 0=periodic timer, 1=one-shot timer
	//or to simply zero the register
	//TIM17->CR1 = 0;						//much easier
	TIM17->CR1 =	(0<<8) |				//0->1:1 clock, 1->2:1 clock, 2->4:1 clock, 3->reserved
				(0<<7) |				//1->APR buffered, 0->APR not buffered
				(0<<5) |				//0->edge-aligned, 1->center-aligned mode 1, 2->center-aligned mode 2, 3->center-aligned mode 3
				(0<<4) |				//0->upcounter, 1->downcounter
				(0<<3) |				//0->continous mode, 1->one pulse mode
				(0<<2) |				//update request source
				(0<<1) |				//0->UEV enabled, 1->UEV disabled
				(0<<0) |				//0->counter disabled, 1->counter enabled
				0x00;
	TIM17->CR2 = 0;						//default value
	TIM17->SMCR= 0;						//default value - use internal clock

	//set the prescaler
	TIM17->PSC = TxCCP_PS - 1;					//set the prescaler
	TIM17->RCR = 0;						//repetition counter = 0 (=no repetition)
	TIM17->ARR = PWM_PR;						//auto reload register / period = 0; - need to change for downcounters
	TIM17->CNT = 0;						//reset the counter

	//clear the status register bits for capture / compare flags
	TIM17->SR &=~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_UIF);
	//disable the interrupt by clearing the enable bits
	TIM17->DIER &=~(TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_UIE);

	//configure CCP1..4
#if defined(TIM17CH1toGPIO)
	//configure CCP1
	TIM17->CCMR1 = 	(TIM17->CCMR1 &~0x00ff) |
					(0<<7) |			//0->OC1REF not affedted by ETRF, 1->OC1REF cleared by ETRF high
					(6<<4) |			//0->frozen (for time base), 1->active on match, 2->inactive on match, 3->toggle, 4->inactive, 5->active, 6->pwm mode 1, 7->pwm mode 2
					(0<<3) |			//0->preload disabled, 1->preload enabled
					(0<<2) |			//0->fast disabled, 1->fast enabled
					(0<<0) |			//0->ch1 as output, 1->ch1 as input, 2->ch1 as input, 3->ch1 as input
					0x00;
	TIM17->CCER = 	(TIM17->CCER &~(0x0f << 0)) |
					(0<< 3) |			//0->normal polarity for CC1N, 1->inverse polarity
					(0<< 2) |			//0->disable CC1N, 1->enable CC1N
					(0<< 1) |			//0->normal polarity for CC1, 1->inverse polarity
					(1<< 0) |			//1->enable CC1, 0->disable CC1
					0x00;
	TIM17->CCR1 = 0;						//0% duty cycle

	//configure the gpio for PWM output
	TIM17CH1toGPIO();
#endif
	TIM17->EGR = 0xff;					//force an update
	TIM17->BDTR|= TIM_BDTR_MOE;				//enable the main output
	//enable the timer.
	TIM17->CR1 |= TIM_CR1_CEN;			//enable the timer

}


//adc module
//global variables
static uint32_t _adc_calfactor;				//adc calibration factor
//initialize adc
void adc1Init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;		//enable clock to ADC1

	//configure adc
	ADC1->CFGR1 = 	(0x00<<26) |			//0->awd on channel 0
					(0<<23) |				//0->AWD disabled
					(0<<22) |				//0->AWD on all channels (but disabled by bit 22
					(1<<16) |				//1->enable discontinuous conversion
					(0<<15) |				//0->auto off disabled
					(0<<14) |				//0->wait conversion disabled
					(0<<13) |				//0->single conversion mode, 1->continuous mode
					(0<<12) |				//0->adc data register preserved in overrun
					(0x00<<10) |			//00->hardware external trigger disabled
					(0x00<<6) |				//0000->external on TRG0 - but disabled
					(0<<5) |				//0->right aligned, 1->left aligned
					(0x00<<3) |				//00->data resolution = 12bit, 01->10bit, 10->8bit, 11->6bit
					(0<<2) |				//0->upward scan
					(0<<1) |				//0->DMA one shot mode selected
					(0<<0) |				//0->DMA disabled
					0x00;
	ADC1->CFGR2 = 	(0x02 << 30);			//00->adc clock, 01->PCLK/2, 10->PCLK/4 -> no jitter
	//set adc sample time
	//0b111->239.5 cycles for all channels
	ADC1->SMPR = 	(ADC_SMPR1_SMPR << (3 * 0)) |
					0x00;
	//set adc channel sequence
	//ADC1->SQR3 = ADC1->SQR2 = ADC1->SQR1 = 0;							//0->1 conversion

	//start self-calibration
	ADC1->CR =	0;							//reset CR
	ADC1->CR = (1<<15);						//start the calibration
	while (ADC1->CR & (1<<15)) continue;	//wait for ADC calibration to finish
	_adc_calfactor = ADC1->DR;				//save adc calibration factor

	//optional: enable temperature sensors
	ADC->CCR |= 	(1ul<<23) |				//1->enable temperature sensor
					(1ul<<22) |				//1->enable Vrefint. 1.20v nominal
					0x00;

	ADC1->CR = 	(1<<0);						//enable aden
	while ((ADC1->ISR & (1<<0)) == 0) continue;	//wait for the adc to be ready
}

//analog to digital converter on ADC1
//ain/analog input channel: ain valid values: 0..15, 16=temperature sensor, 17=Vrefint
//***user needs to be configured as floating input***
uint16_t adc1Read(uint32_t ain) {
	//ADC1->ISR &= ~(1<<2);					//clear the eoc flag
	//ADC1->CR1 = (ADC1->CR1 &~0x1f) | (ain & 0x1f);	//pick the adc channel
	//ADC1->CR2|= (1<<0);					//start the conversion
	ADC1->CHSELR = ain & 0x03fffful;				//define the first (and only) adc ch
	ADC1->CR |= (1<<2);						//start conversion
	while ((ADC1->ISR & (1<<2)) == 0) continue;	//wait for conversion to end (EOC set at end of conversion)
	return ADC1->DR;						//return adc result and clears the EOC bit
}


//end ADC

#if 0
//input capture
static void (*_ic1_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC1DAT=0;						//buffer 

//input capture ISR
void _ISR_PSV _IC1Interrupt(void) {				//for PIC32
	//clear the flag
	IC1DAT = IC1BUF;							//read the captured value
	IFS0bits.IC1IF = 0;							//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic1_isrptr();								//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic1Init(void) {
	_ic1_isrptr = empty_handler;				//reset user handler

	IC12RP();									//assign pin to IC
	PMD2bits.IC1MD = 0;							//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC1CON1 = 0;						//reset to default value
	IC1CON2 = 0;						//reset to default value
	IC1CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC1BUF;								//read the buffer to clear the flag
	IFS0bits.IC1IF   = 0;				//0->clear the flag
	IEC0bits.IC1IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC1CON1bits.ON = 1;					//1->enable the module, 0->disable the module
#else
	//for ga002 devices
	IC1CON = 0;						//reset to default value
	//IC1CON2 = 0;						//reset to default value
	IC1CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC1BUF;								//read the buffer to clear the flag
	IFS0bits.IC1IF   = 0;				//0->clear the flag
	IEC0bits.IC1IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC1CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
	//input capture running now
}

//activate user ptr
void ic1AttachISR(void (*isrptr)(void)) {
	_ic1_isrptr = isrptr;				//install user ptr
	IC1BUF;								//read the buffer to clear the flag
	IFS0bits.IC1IF   = 0;				//0->clear the flag
	IEC0bits.IC1IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic1Get(void) {
	return IC1DAT;
}
	
static void (*_ic2_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC2DAT=0;				//buffer 

//input capture ISR
void _ISR_PSV _IC2Interrupt(void) {		//for PIC32
	//clear the flag
	IC2DAT = IC2BUF;					//read the captured value
	IFS0bits.IC2IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic2_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic2Init(void) {
	_ic2_isrptr = empty_handler;		//reset user handler

	IC22RP();							//assign pin to IC
	PMD2bits.IC2MD = 0;					//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC2CON1 = 0;						//reset to default value
	IC2CON2 = 0;						//reset to default value
	IC2CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC2IF   = 0;				//0->clear the flag
	IEC0bits.IC2IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC2CON1bits.ON = 1;				//1->enable the module, 0->disable the module
#else
	//for ga002 devices
	IC2CON = 0;							//reset to default value
	//IC2CON2 = 0;						//reset to default value
	IC2CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC2IF   = 0;				//0->clear the flag
	IEC0bits.IC2IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC2CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
}

//activate user ptr
void ic2AttachISR(void (*isrptr)(void)) {
	_ic2_isrptr = isrptr;				//install user ptr
	IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC2IF   = 0;				//0->clear the flag
	IEC0bits.IC2IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic2Get(void) {
	return IC2DAT;
}
	
static void (*_ic3_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC3DAT=0;				//buffer 

//input capture ISR
void _ISR_PSV _IC3Interrupt(void) {		//for PIC32
	//clear the flag
	IC3DAT = IC3BUF;					//read the captured value
	IFS2bits.IC3IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic3_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic3Init(void) {
	_ic3_isrptr = empty_handler;		//reset user handler

	IC32RP();							//assign pin to IC
	PMD2bits.IC3MD = 0;					//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC3CON1 = 0;						//reset to default value
	IC3CON2 = 0;						//reset to default value
	IC3CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC3BUF;								//read the buffer to clear the flag
	IFS2bits.IC3IF   = 0;				//0->clear the flag
	IEC2bits.IC3IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC3CON1bits.ON = 1;				//1->enable the module, 0->disable the module
	//input capture running now
#else
	//for ga002 devices
	IC3CON = 0;							//reset to default value
	//IC3CON2 = 0;						//reset to default value
	IC3CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC3BUF;								//read the buffer to clear the flag
	IFS2bits.IC3IF   = 0;				//0->clear the flag
	IEC2bits.IC3IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC3CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
}

//activate user ptr
void ic3AttachISR(void (*isrptr)(void)) {
	_ic3_isrptr = isrptr;				//install user ptr
	IC3BUF;								//read the buffer to clear the flag
	IFS2bits.IC3IF   = 0;				//0->clear the flag
	IEC2bits.IC3IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic3Get(void) {
	return IC3DAT;
}
	
static void (*_ic4_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC4DAT=0;				//buffer 

//input capture ISR
void _ISR_PSV _IC4Interrupt(void) {		//for PIC32
	//clear the flag
	IC4DAT = IC4BUF;					//read the captured value
	IFS2bits.IC4IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic4_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic4Init(void) {
	_ic4_isrptr = empty_handler;		//reset user handler

	IC42RP();							//assign pin to IC
	PMD2bits.IC4MD = 0;					//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC4CON1 = 0;						//reset to default value
	IC4CON2 = 0;						//reset to default value
	IC4CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC4BUF;								//read the buffer to clear the flag
	IFS2bits.IC4IF   = 0;				//0->clear the flag
	IEC2bits.IC4IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC4CON1bits.ON = 1;				//1->enable the module, 0->disable the module
	//input capture running now
#else
	//for ga002 devices
	IC4CON = 0;						//reset to default value
	//IC4CON2 = 0;						//reset to default value
	IC4CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC4BUF;								//read the buffer to clear the flag
	IFS2bits.IC4IF   = 0;				//0->clear the flag
	IEC2bits.IC4IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC4CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
}

//activate user ptr
void ic4AttachISR(void (*isrptr)(void)) {
	_ic4_isrptr = isrptr;				//install user ptr
	IC4BUF;								//read the buffer to clear the flag
	IFS2bits.IC4IF   = 0;				//0->clear the flag
	IEC2bits.IC4IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic4Get(void) {
	return IC4DAT;
}
	
static void (*_ic5_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
volatile uint16_t IC5DAT=0;				//buffer 

//input capture ISR
void _ISR_PSV _IC5Interrupt(void) {		//for PIC32
	//clear the flag
	IC5DAT = IC5BUF;					//read the captured value
	IFS2bits.IC5IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic5_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic5Init(void) {
	_ic5_isrptr = empty_handler;		//reset user handler

	IC52RP();							//assign pin to IC
	PMD2bits.IC5MD = 0;					//0->enable power to input capture

#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
	IC5CON1 = 0;						//reset to default value
	IC5CON2 = 0;						//reset to default value
	IC5CON1  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC5BUF;								//read the buffer to clear the flag
	IFS2bits.IC5IF   = 0;				//0->clear the flag
	IEC2bits.IC5IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC5CON1bits.ON = 1;				//1->enable the module, 0->disable the module
	//input capture running now
#else
	//for ga002 devices
	IC5CON = 0;							//reset to default value
	//IC5CON2 = 0;						//reset to default value
	IC5CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
                (0<<13) |				//0->operates in idle, 1->don't operate in idle
                (1<<9) |				//1-.capture rising edge first (only used for ICM110)
                (0<<8) |				//1->32-bit mode, 0->16-bit mode
                (1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
                (0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
                (0<<4) |				//0->ICx no overflow, 1->ICx overflow
                (0<<3) |				//0->buffer is empty, 1->buffer is not empty
                (3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
                0x00;

	IC5BUF;								//read the buffer to clear the flag
	IFS2bits.IC5IF   = 0;				//0->clear the flag
	IEC2bits.IC5IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC5CONbits.ON = 1;				//1->enable the module, 0->disable the module
#endif
}

//activate user ptr
void ic5AttachISR(void (*isrptr)(void)) {
	_ic5_isrptr = isrptr;				//install user ptr
	IC5BUF;								//read the buffer to clear the flag
	IFS2bits.IC5IF   = 0;				//0->clear the flag
	IEC2bits.IC5IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//read buffer value
uint16_t ic5Get(void) {
	return IC5DAT;
}
//end input capture

//extint
//extint0
void (* _int0_isrptr) (void)=empty_handler;

void _ISR_PSV _INT0Interrupt(void) {
	IFS0bits.INT0IF = 0;				//clera the flag
	_int0_isrptr();						//run the isr
}

void int0Init(void) {
	//INT02RP();						//map int0_pin - int0 cannot be remapped
	_int0_isrptr = empty_handler;		//initialize int isr ptr
	IFS0bits.INT0IF = 0;				//clear int0 flag
	IEC0bits.INT0IE = 0;				//1->enable int0 interrupt, 0->disable the interrupt
	//INTCONbits.INT0EP = 1;			//1=triggered on the falling edge. 0 = rising edge
}

void int0AttachISR(void (*isrptr) (void)) {
	_int0_isrptr = isrptr;
	IFS0bits.INT0IF = 0;				//clear int0 flag
	IEC0bits.INT0IE = 1;				//1->enable int0 interrupt, 0->disable the interrupt
}

//extint1
void (* _int1_isrptr) (void)=empty_handler;

void _ISR_PSV _INT1Interrupt(void) {
	IFS1bits.INT1IF = 0;				//clera the flag
	_int1_isrptr();						//run the isr
}

void int1Init(void) {
	INT12RP();							//map int1_pin
	_int1_isrptr = empty_handler;		//initialize int isr ptr
	IFS1bits.INT1IF = 0;				//clear int0 flag
	IEC1bits.INT1IE = 0;				//1->enable int0 interrupt, 0->disable the interrupt
	//INTCONbits.INT1EP = 1;			//1=triggered on the falling edge. 0 = rising edge
}

void int1AttachISR(void (*isrptr) (void)) {
	_int1_isrptr = isrptr;
	IFS1bits.INT1IF = 0;				//clear int0 flag
	IEC1bits.INT1IE = 1;				//1->enable int0 interrupt, 0->disable the interrupt
}

//extint2
void (* _int2_isrptr) (void)=empty_handler;

void _ISR_PSV _INT2Interrupt(void) {
	IFS1bits.INT2IF = 0;				//clera the flag
	_int2_isrptr();						//run the isr
}

void int2Init(void) {
	INT22RP();							//map int2_pin
	_int2_isrptr = empty_handler;		//initialize int isr ptr
	IFS1bits.INT2IF = 0;				//clear int0 flag
	IEC1bits.INT2IE = 0;				//1->enable int0 interrupt, 0->disable the interrupt
	//INTCONbits.INT2EP = 1;			//1=triggered on the falling edge. 0 = rising edge
}

void int2AttachISR(void (*isrptr) (void)) {
	_int2_isrptr = isrptr;
	IFS1bits.INT2IF = 0;				//clear int0 flag
	IEC1bits.INT2IE = 1;				//1->enable int0 interrupt, 0->disable the interrupt
}


//end extint

//spi
//rest spi1
void spi1Init(uint16_t br) {
	PMD1bits.SPI1MD = 0;				//0->enable the module
	
	//map the pins
	
	//initialize the spi module
	//master mode, PBCLK as clock, 8-bit data, enhanced buffer mode
	SPI1CON1 = SPI1CON2 = 0; 			//reset the spi module
	SPI1CON1bits.MSTEN = 1;				//1->master mode, 0->slave mode
	//SPI1CON1bits.ENHBUF= 1;			//1->enable enhanced buffer mode, 0->disable enhanced buffer mode
	SPI1CON1bits.PPRE = br;				//set the baudrate generator
	//need to deal with secondary as well
	SPI1BUF;							//read the buffer to reset the flag
	IFS0bits.SPI1IF = 0;				//0->reset the flag
	IEC0bits.SPI1IE = 0;				//0->disable the interrupt, 1->enable the interrupt
	//SPI1CONbits.ON = 1;				//1->enable the module, 0->disable the module
}

//send data via spi
void spi1Write(uint8_t dat) {
	while (spi1Busy()) continue;		//tx buffer is full
	SPI1BUF = dat;						//load the data
}

//rest spi2
void spi2Init(uint16_t br) {
	PMD1bits.SPI2MD = 0;				//0->enable the module
	
	//map the pins
	
	//initialize the spi module
	//master mode, PBCLK as clock, 8-bit data, enhanced buffer mode
	SPI2CON1 = SPI2CON2 = 0; 			//reset the spi module
	SPI2CON1bits.MSTEN = 1;				//1->master mode, 0->slave mode
	//SPI2CON1bits.ENHBUF= 1;			//1->enable enhanced buffer mode, 0->disable enhanced buffer mode
	SPI2CON1bits.PPRE = br;				//set the baudrate generator
	//need to deal with 2nd ary as well
	SPI2BUF;							//read the buffer to reset the flag
	IFS2bits.SPI2IF = 0;				//0->reset the flag
	IEC2bits.SPI2IE = 0;				//0->disable the interrupt, 1->enable the interrupt
	//SPI2CONbits.ON = 1;				//1->enable the module, 0->disable the module
}

//send data via spi
void spi2Write(uint8_t dat) {
	while (spi2Busy()) continue;		//tx buffer is full
	SPI2BUF = dat;						//load the data
}
//end spi

//i2c

//end i2c

//rtcc
//end rtcc
#endif	//xxxxx

//cnint
//global variables
static void (* _exti0_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti1_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti2_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti3_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti4_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti5_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti6_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti7_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti8_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti9_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti10_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti11_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti12_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti13_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti14_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default
static void (* _exti15_isrptr)(void)=empty_handler;				//exti0 isrpointer pointing to empty_handler by default

//EXTI0-1 ISR
void EXTI0_1_IRQHandler(void) {
	//clear the flag and execute user isr
	switch (EXTI->PR) {
		case (1<<0): EXTI->PR |= (1<<0); _exti0_isrptr(); break;
		case (1<<1): EXTI->PR |= (1<<1); _exti1_isrptr(); break;
	}
}

//EXTI2-3 ISR
void EXTI2_3_IRQHandler(void) {
	//clear the flag and execute user isr
	switch (EXTI->PR) {
		case (1<<2): EXTI->PR |= (1<<2); _exti2_isrptr(); break;
		case (1<<3): EXTI->PR |= (1<<3); _exti3_isrptr(); break;
	}
}

//EXTI15-10 ISR
void EXTI4_15_IRQHandler(void) {
	//clear the flag and execute user isr
	switch (EXTI->PR) {
		case (1<< 4): EXTI->PR |= (1<< 4); _exti4_isrptr(); break;
		case (1<< 5): EXTI->PR |= (1<< 5); _exti5_isrptr(); break;
		case (1<< 6): EXTI->PR |= (1<< 6); _exti6_isrptr(); break;
		case (1<< 7): EXTI->PR |= (1<< 7); _exti7_isrptr(); break;
		case (1<< 8): EXTI->PR |= (1<< 8); _exti8_isrptr(); break;
		case (1<< 9): EXTI->PR |= (1<< 9); _exti9_isrptr(); break;
		case (1<<10): EXTI->PR |= (1<<10); _exti10_isrptr(); break;
		case (1<<11): EXTI->PR |= (1<<11); _exti11_isrptr(); break;
		case (1<<12): EXTI->PR |= (1<<12); _exti12_isrptr(); break;
		case (1<<13): EXTI->PR |= (1<<13); _exti13_isrptr(); break;
		case (1<<14): EXTI->PR |= (1<<14); _exti14_isrptr(); break;
		case (1<<15): EXTI->PR |= (1<<15); _exti15_isrptr(); break;
	}
}

//global defines
#define EXTI_FALLING	FALLING				//trigger on falling edge
#define EXTI_RISING		RISING				//trigger on rising edge
#define EXTI_BOTH		CHANGE				//trigger on falling and rising edges

//global variables
//initialize the exti
//pin: one pin only
//gpio: GPIOA..GPIOG
void extiInit(GPIO_TypeDef * gpio, uint16_t pin, uint8_t edge) {
	uint16_t tmp;
	//enable afio
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	//enable the clock
	switch ((uint32_t) gpio) {
#if defined(RCC_AHBENR_GPIOAEN)
		case (uint32_t) GPIOA: RCC->AHBENR |= RCC_AHBENR_GPIOAEN; tmp = 0x00; break;
#endif
#if defined(RCC_AHBENR_GPIOBEN)
		case (uint32_t) GPIOB: RCC->AHBENR |= RCC_AHBENR_GPIOBEN; tmp = 0x01; break;
#endif
#if defined(RCC_AHBENR_GPIOCEN)
		case (uint32_t) GPIOC: RCC->AHBENR |= RCC_AHBENR_GPIOCEN; tmp = 0x02; break;
#endif
#if defined(RCC_AHBENR_GPIODEN)
		case (uint32_t) GPIOD: RCC->AHBENR |= RCC_AHBENR_GPIODEN; tmp = 0x03; break;
#endif
#if defined(RCC_AHBENR_GPIOEEN)
		case (uint32_t) GPIOE: RCC->AHBENR |= RCC_AHBENR_GPIOEEN; tmp = 0x04; break;
#endif
#if defined(RCC_AHBENR_GPIOFEN)
		case (uint32_t) GPIOF: RCC->AHBENR |= RCC_AHBENR_GPIOFEN; tmp = 0x05; break;
#endif
#if defined(RCC_AHBENR_GPIOGEN)
		case (uint32_t) GPIOG: RCC->AHBENR |= RCC_AHBENR_GPIOGEN; tmp = 0x06; break;
#endif
	}
	//configure exti edges
	//1->falling/rising edge enabled, 0->falling/rising edge disabled
	switch (edge) {
		case EXTI_FALLING: 	EXTI->FTSR |= pin; EXTI->RTSR &=~pin; break;
		case EXTI_RISING: 	EXTI->FTSR &=~pin; EXTI->RTSR |= pin; break;
		case EXTI_BOTH: 	EXTI->FTSR |= pin; EXTI->RTSR |= pin; break;
	}
	//configure the port
	switch (pin) {
		case (1<< 0): SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] &~0x000f) | ((tmp) <<  0); break;
		case (1<< 1): SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] &~0x00f0) | ((tmp) <<  4); break;
		case (1<< 2): SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] &~0x0f00) | ((tmp) <<  8); break;
		case (1<< 3): SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] &~0xf000) | ((tmp) << 12); break;
		case (1<< 4): SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] &~0x000f) | ((tmp) <<  0); break;
		case (1<< 5): SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] &~0x00f0) | ((tmp) <<  4); break;
		case (1<< 6): SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] &~0x0f00) | ((tmp) <<  8); break;
		case (1<< 7): SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] &~0xf000) | ((tmp) << 12); break;
		case (1<< 8): SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] &~0x000f) | ((tmp) <<  0); break;
		case (1<< 9): SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] &~0x00f0) | ((tmp) <<  4); break;
		case (1<<10): SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] &~0x0f00) | ((tmp) <<  8); break;
		case (1<<11): SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] &~0xf000) | ((tmp) << 12); break;
		case (1<<12): SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] &~0x000f) | ((tmp) <<  0); break;
		case (1<<13): SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] &~0x00f0) | ((tmp) <<  4); break;
		case (1<<14): SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] &~0x0f00) | ((tmp) <<  8); break;
		case (1<<15): SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] &~0xf000) | ((tmp) << 12); break;
	};
}

//install user handler
void extiAttachISR(uint16_t pin, void (*isr_ptr)(void)) {
	//clear the flag
	EXTI->PR |= pin;						//1->clear the flag
	//enable the interrupt
	EXTI->IMR |= pin;							//1->enable the interrupt, 0->disable the interrupt
	EXTI->EMR |= pin;							//1->enable the event trigger, 0->disable the event trigger

	//enable nvic
	switch (pin) {
		case (1<< 0): NVIC_EnableIRQ(EXTI0_1_IRQn); _exti0_isrptr = isr_ptr; break;
		case (1<< 1): NVIC_EnableIRQ(EXTI0_1_IRQn); _exti1_isrptr = isr_ptr; break;
		case (1<< 2): NVIC_EnableIRQ(EXTI2_3_IRQn); _exti2_isrptr = isr_ptr; break;
		case (1<< 3): NVIC_EnableIRQ(EXTI2_3_IRQn); _exti3_isrptr = isr_ptr; break;
		case (1<< 4): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti4_isrptr = isr_ptr; break;
		case (1<< 5): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti5_isrptr = isr_ptr; break;
		case (1<< 6): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti6_isrptr = isr_ptr; break;
		case (1<< 7): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti7_isrptr = isr_ptr; break;
		case (1<< 8): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti8_isrptr = isr_ptr; break;
		case (1<< 9): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti9_isrptr = isr_ptr; break;
		case (1<<10): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti10_isrptr = isr_ptr; break;
		case (1<<11): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti11_isrptr = isr_ptr; break;
		case (1<<12): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti12_isrptr = isr_ptr; break;
		case (1<<13): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti13_isrptr = isr_ptr; break;
		case (1<<14): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti14_isrptr = isr_ptr; break;
		case (1<<15): NVIC_EnableIRQ(EXTI4_15_IRQn); _exti15_isrptr = isr_ptr; break;
	}
}

//end cnint
