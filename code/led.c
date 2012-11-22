#include <avr/io.h>
#include <avr/interrupt.h>



/* ********************************************************************** */
extern volatile unsigned char XiaoShuDian;
/* ********************************************************************** */


/* ********************************************************************** */
//LED contrel Bin
#define bSDI   2
#define bLCLK  1
#define bSCLK  0
#define WEIBUS_0(x) PORTB &=~(1<<x)
#define WEIBUS_1(x) PORTB |= (1<<x)
#define bDUAN1  0
#define bDUAN2  1
#define bDUAN3  2
#define bDUAN4  3
#define DUANBUS_0(x) PORTC &=~(1<<x)
#define DUANBUS_1(x) PORTC |= (1<<x)
#define DUANALL_0 PORTC &=~((1<<bDUAN1)|(1<<bDUAN2)|(1<<bDUAN3)|(1<<bDUAN4))
#define DUANALL_1 PORTC |= ((1<<bDUAN1)|(1<<bDUAN2)|(1<<bDUAN3)|(1<<bDUAN4))

#define T_LEDRefresh    (2)    //(mS)
#define Timer1ClkSel	(5)    //0:none,1:CLK,2:CLK/8,3:CLK/64,4:CLK/256,5:CLK/1024,6:T0 pin falling edge,7:T0 pin rising edge
#define Timer1Count     (0xFF - (T_LEDRefresh * 1000)/(1024/8))
/* ********************************************************************** */

/* ********************************************************************** */
//dangqian Duan Addr
volatile unsigned char DuanAddr = 0;
#define LEDGONGYANG
#ifdef LEDGONGYANG
//gong yang duan ma
unsigned char LEDMA[16]={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,
			 0x80,0x90,0x88,0x83,0xC6,0xA1,0x86,0x8E};
#define LED_FU (~0x40)
#else
//gong yin  duan ma
unsigned char LEDMA[16]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,
			 0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71};
#define LED_FU (0x40)
#endif
//LED display data
volatile unsigned char LEDBUFFER[4]={0x40,0x40,0x40,0x40};
/* ********************************************************************** */

//send data to LED
void SPISend(unsigned char Data)
{
	int i;

	WEIBUS_0(bSCLK);
	WEIBUS_0(bLCLK);
	for(i=0;i<8;i++)
	{
	    if(Data & (0x80>>i))
		WEIBUS_1(bSDI);
	    else
		WEIBUS_0(bSDI);
	    WEIBUS_1(bSCLK);
	    WEIBUS_0(bSCLK);
	}
	WEIBUS_1(bLCLK);
	WEIBUS_0(bLCLK);
}
SIGNAL(SIG_OVERFLOW0)
{
	switch(DuanAddr)
	{
	case 0:	DUANALL_1;	
		SPISend(LEDBUFFER[0]);
		DUANBUS_0(bDUAN1);
		break;
	case 1:	DUANALL_1;	
		SPISend(LEDBUFFER[1]);
		DUANBUS_0(bDUAN2);
		break;
	case 2:	DUANALL_1;	
		SPISend(LEDBUFFER[2]);
		DUANBUS_0(bDUAN3);
		break;
	case 3:	DUANALL_1;	
		SPISend(LEDBUFFER[3]);
		DUANBUS_0(bDUAN4);
	}
	DuanAddr++;
	if(DuanAddr>3)
		DuanAddr = 0;
	TCNT0 = Timer1Count;
}
unsigned char OpenDisplayLED(void)
{
	/* clock select clk/32 */
	TCCR0 = Timer1ClkSel;
	/* enabile Timer0 */
	TIMSK |= 0x01;
	TCNT0 = Timer1Count;
	DuanAddr = 0;
	return 0;
}
/* ********************************************************************** */
void DisplayHex(unsigned int TempData)
{
	LEDBUFFER[3] = LEDMA[(unsigned char)(TempData & 0x0F)];
	TempData = TempData>>4;
	LEDBUFFER[2] = LEDMA[(unsigned char)(TempData & 0x0F)];
	TempData = TempData>>4;
	LEDBUFFER[1] = LEDMA[(unsigned char)(TempData & 0x0F)];
	TempData = TempData>>4;
	LEDBUFFER[0] = LEDMA[(unsigned char)(TempData & 0x0F)];
	TempData = TempData>>4;
}
void Display10(unsigned int TempData)
{
    if((TempData & 0x8000) != 0){
        LEDBUFFER[0] = LED_FU;
        TempData = 0xFFFF - TempData;
	LEDBUFFER[3] = LEDMA[(unsigned char)(TempData % 10)];
	TempData = TempData/10;
	LEDBUFFER[2] = LEDMA[(unsigned char)(TempData % 10)];
	TempData = TempData/10;
	LEDBUFFER[1] = LEDMA[(unsigned char)(TempData % 10)];
	TempData = TempData/10;
    }else{
    	LEDBUFFER[3] = LEDMA[(unsigned char)(TempData % 10)];
	TempData = TempData/10;
	LEDBUFFER[2] = LEDMA[(unsigned char)(TempData % 10)];
	TempData = TempData/10;
	LEDBUFFER[1] = LEDMA[(unsigned char)(TempData % 10)];
	TempData = TempData/10;
	LEDBUFFER[0] = LEDMA[(unsigned char)(TempData % 10)];
	TempData = TempData/10;	
    }


	if((XiaoShuDian < 5) && XiaoShuDian){
#ifdef LEDGONGYANG
		LEDBUFFER[4 - XiaoShuDian] &= 0x7F;
#else
		LEDBUFFER[4 - XiaoShuDian] |= 0x80;
#endif		
	}
}
void DisplayFloat(int FZData,int FMData)
{
	
}




