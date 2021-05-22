//STM32duino code
// - using STM32F0
// - version history
// - v2, 5/22/2021: original port
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

#include "stm32duino.h"					//we use pic24duino
#include "string.h"						//we use strcpy

//hardware configuration
#define GND			PA3					//pin grounded
#define LED			PA4					//led pin
#define LED_DLY		(F_PHB / 2)			//half a second
//end hardware configuration


//print to uart1
void u1Print(char *str, int32_t dat) {
	char uRAM[40];						//transmission buffer, 40-1 char max
	
	strcpy(uRAM, str);					//copy to uarm
	if (dat < 0) {uRAM[6]='-'; dat = -dat;}
	uRAM[19]='0'+(dat % 10); dat /= 10;
	uRAM[18]='0'+(dat % 10); dat /= 10;
	uRAM[17]='0'+(dat % 10); dat /= 10;
	uRAM[16]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[15]='0'+(dat % 10); dat /= 10;
	uRAM[14]='0'+(dat % 10); dat /= 10;
	uRAM[13]='0'+(dat % 10); dat /= 10;
	uRAM[12]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[11]='0'+(dat % 10); dat /= 10;
	uRAM[10]='0'+(dat % 10); dat /= 10;
	uRAM[ 9]='0'+(dat % 10); dat /= 10;
	uRAM[ 8]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[ 7]='0'+(dat % 10); dat /= 10;
	uart1Puts(uRAM);	//send a message on uart1
}	

//print to uart2
void u2Print(char *str, int32_t dat) {
	char uRAM[40];						//transmission buffer, 40-1 char max
	
	strcpy(uRAM, str);					//copy to uarm
	if (dat < 0) {uRAM[6]='-'; dat = -dat;}
	uRAM[19]='0'+(dat % 10); dat /= 10;
	uRAM[18]='0'+(dat % 10); dat /= 10;
	uRAM[17]='0'+(dat % 10); dat /= 10;
	uRAM[16]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[15]='0'+(dat % 10); dat /= 10;
	uRAM[14]='0'+(dat % 10); dat /= 10;
	uRAM[13]='0'+(dat % 10); dat /= 10;
	uRAM[12]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[11]='0'+(dat % 10); dat /= 10;
	uRAM[10]='0'+(dat % 10); dat /= 10;
	uRAM[ 9]='0'+(dat % 10); dat /= 10;
	uRAM[ 8]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[ 7]='0'+(dat % 10); dat /= 10;
	uart2Puts(uRAM);	//send a message on uart1
}	


//flip the led
void led_flp(void) {
	digitalWrite(LED, !digitalRead(LED));	//flip te led
}

//smoothing filter
#define EXP_TGT			400					//target value
#define EXP_LENGTH		16					//length of the memory
int32_t exp_smooth(int32_t dat) {
	static int32_t dat_acc=EXP_TGT * EXP_LENGTH;
	static int32_t dat_smooth = EXP_TGT;
	
	dat_acc += dat - dat_smooth;			//update the accumulator
	return dat_smooth = (dat_acc + EXP_LENGTH/2) / EXP_LENGTH;	//for round-up
}
	
uint16_t adc_min, adc_max, adc_avg;
#define avg(v1, v2)		((v1 + v2) / 2)

//user defined set up code
void setup(void) {
	pinMode(GND, OUTPUT); digitalWrite(GND, LOW);	//gnd pin grounded
	pinMode(LED, OUTPUT);				//led as output pin
	
	//initialize the timer
	//tmr3Init(100);
	//tmr3SetPR1(1000); tmr3AttachISR1(led_flp);
	//tmr3SetPR4(1010); tmr3AttachISR4(led_flp);
	
	//tmr17Init(100);
	//tmr17SetPR(1000); tmr17AttachISR(led_flp);

	//tmr16Init(100);
	//tmr16SetPR(1010); tmr16AttachISR(led_flp);

	//initialize the uart
	uart1Init(UART_BR9600);			//initialize uart1
	//uart2Init(UART_BR9600);			//initialize uart2
	
	//analog write / pwm output
	//pwm14Init(1);						//reset pwm
	//pwm14SetDC1(0xffff/10);
	
	//analog read
	adc1Init();						//reset the adc
	
	//input capture
	//ic1Init();						//reset the ic
	//ic1AttachISR(led_flp);
	
	//extint
	//int2Init();						//reset the extint
	//int2AttachISR(led_flp);
	
	//change notification
	//cnInit((1ul<<2) | (1ul<<23));		//reset the module
	//cnAttachISR(led_flp);

	analogRead(ADC_VREFINT);
	adc_min=adc_max=adc_avg=analogRead(ADC_TS);
}

//user defined main loop
void loop(void) {
	static uint32_t tick0=0;
	uint32_t tmp0, tmp1, tmp;
	uint16_t adc_vbg, adc_vbg2;

	tmp=analogRead(ADC_TS);
	adc_min=min(adc_min, tmp);
	adc_max=max(adc_max, tmp);
	adc_avg=avg(adc_avg, tmp);
	//u2Print("start                        \r\n", tick0);
	//if enough time has elapsed
	if (ticks() - tick0 > LED_DLY) {
		tick0 += LED_DLY;						//advance to the next match point
		digitalWrite(LED, !digitalRead(LED));	//flip led, 105 ticks
		//pwm5SetDC(pwm5GetDC() + 1000);
		tmp0=ticks(); 
		//digitalRead(PA5);						//read a pin, 59 ticks (base case = 29 ticks, if there is nothing)
		//uart1Init(UART_BR9600);					//initial uart, 1328 ticks
		//digitalWrite(LED, !digitalRead(LED));	//flip led, 105 ticks
		//tmp=analogRead(ADC_TS);
		//for (tmp=0; tmp<1000; tmp++) digitalWrite(LED, !digitalRead(LED));	//flip led, 89100/1000 ticks
		//for (tmp=0; tmp<1000; tmp++) IO_FLP(LATB, 1<<7);					//flip led, 16040/1000 ticks
		tmp0=ticks() - tmp0;
		u1Print("tick0=                    ", tmp);
		//u1Print("adc  =                    ", analogRead(ADC_VBG));
		//u1Print("adc_e=                    \r\n", analogRead(ADC_VBG2));
		//u1Print("tmp0 =                    ", tmp0);
		u1Print("ticks=                    ", tmp0=ticks());
		u1Print("ticks=                    ", tmp1=ticks());
		u1Print("ticks=                    \r\n", tmp1-tmp0);
	}	
	
	//delay(500);
}
