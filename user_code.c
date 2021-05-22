//PIC32duino code
// - using PIC32MX1xx/2xx
// - version history
// - v2, 5/13/2021: original port
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

#include "pic32duino.h"					//we use pic32duino

//hardware configuration
#define LED			PB7					//led pin
#define LED_DLY		(F_CPU / 2)
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

//print
void uprint(void) {
	u2Print("IC3:                    \r\n", ic3Get());
}

//smoothing filter
#define EXP_TGT			400			//target value
#define EXP_LENGTH		16			//length of the memory
int32_t exp_smooth(int32_t dat) {
	static int32_t dat_acc=EXP_TGT * EXP_LENGTH;
	static int32_t dat_smooth = EXP_TGT;
	
	dat_acc += dat - dat_smooth;
	return dat_smooth = dat_acc / EXP_LENGTH;
}
	
//user defined set up code
void setup(void) {
	pinMode(LED, OUTPUT);			//led as output pin
	
	//initialize the timer
	//tmr4Init(TMR_PS256x, 10000+000); tmr4AttachISR(led_flp);
	//tmr5Init(TMR_PS2x, 1000+002); tmr5AttachISR(led_flp);
	
	//initialize the uart
	uart1Init(UART_BR9600);			//initialize uart1
	uart2Init(UART_BR9600);			//initialize uart2
	
	//analog write / pwm output
	//wm1Init();						//reset pwm
	//pwm1SetDC(-1000);
	
	//analog read
	adcInit();						//reset the adc
	
	//input capture
	//ic3Init();						//reset the ic
	//ic3AttachISR(uprint);
	
	//extint
	//int0Init();						//reset the extint
	//int0AttachISR(led_flp);
	
	//cn interrupt
	//cnbInit((1<<8));					//CN pin on PB8
	//cnbAttachISR(uprint);
}

//user defined main loop
void loop(void) {
	static uint32_t tick0=0;
	uint32_t tmp0, tmp1, tmp;
	
	//if enough time has elapsed
	if (ticks() - tick0 > LED_DLY) {
		tick0 += LED_DLY;						//advance to the next match point
		digitalWrite(LED, !digitalRead(LED));	//flip led, 96 - 100 ticks
		tmp0=ticks(); 
		//digitalRead(PA5);						//read a pin, 40-4 ticks (base case = 4 ticks, if there is nothing)
		//uart1Init(UART_BR9600);					//initial uart, 450-4 ticks
		//tmp=analogRead(ADC_AN14);					//analog read. 74-4 ticks
		//digitalWrite(LED, !digitalRead(LED));	//flip led, 96 - 100 ticks
		//for (tmp=0; tmp<1000; tmp++) digitalWrite(LED, !digitalRead(LED));	//flip led, 35505/1000 ticks
		//for (tmp=0; tmp<1000; tmp++) IO_FLP(LATB, 1<<7);						//flip led, 7006/1000 ticks
		for (tmp=0; tmp<1000; tmp++) LATBINV = 1<<7;							//flip led, 4008/1000 ticks
		tmp0=ticks() - tmp0;
		u2Print("tick0=                    ", tmp);
		u2Print("ticks=                    ", tmp0);
		//u2Print("ticks=                    ", tmp1=ticks());
		u2Print("ticks=                    \r\n", tmp1-tmp0);
	}	
	//delay(500);
}
