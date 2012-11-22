/* ********************************************************************** */
/* 
 * 
 * 
 */
/* ********************************************************************** */

//
/* ********************************************************************** */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>

#include "led.h"
#include "CRC16.h"
/* ********************************************************************** */

//
/* ********************************************************************** */
#define uchar unsigned char

#ifndef NULL
#define NULL (void *)(0)
#endif
/* ********************************************************************** */

//#define MYDEBUG

//
/* ********************************************************************** */
void ErrorDisplay(unsigned int ErrCode);
/* ********************************************************************** */


/* ********************************************************************** */
#define CODE_CID        (0x88)
#define CODE_PID        (0x80)
#define CODE_VID        (0x00)

#define CHAR_COMPANY    ("BOSSUN\r\n")
#define CHAR_DESCRIBE   ("Change 4~20mA to 485, ADC And Display!\r\n")
#define def_DeviceAddr  (0x01)
#define UartBaud        (9600)
#define Timer2ClkSel    (5)    //0:none,1:CLK,2:CLK/8,3:CLK/32,4:CLK/64,5:CLK/128,6:CLK/256,7:CLK/1024
#define FrameInterval   (0xFF - (10000/(UartBaud/100) * 28)/(128/8))   //3.5 byte
#define RXARRAYTEMP_MAX (20)
#define RX485()         (PORTD &= ~(1<<2))
#define TX485()         (PORTD |= (1<<2))

#define ADC_ADS1110

#define AUTO_ZERO (-120)

#define I_RangeMAX        (2000L)
#define I_RangeMIN        (400L)
#define I_DEF_XiaoShuDian (0)
#ifdef ADC_ADS1110
#define I_ZERO            AUTO_ZERO
#else
#define I_ZERO            (0)
#endif

#define T_RangeMAX        (800L)
#define T_RangeMIN        (0L)
#define T_DEF_XiaoShuDian (1)
#ifdef ADC_ADS1110
#define T_ZERO            AUTO_ZERO
#else
#define T_ZERO            (0)
#endif

#define Y_RangeMAX        (2000L)
#define Y_RangeMIN        (400L)
#define Y_DEF_XiaoShuDian (2)
#ifdef ADC_ADS1110
#define Y_ZERO            AUTO_ZERO
#else
#define Y_ZERO            (0)
#endif

#define L_RangeMAX        (2000L)
#define L_RangeMIN        (400L)
#define L_DEF_XiaoShuDian (1)
#ifdef ADC_ADS1110
#define L_ZERO            AUTO_ZERO
#else
#define L_ZERO            (0)
#endif



#ifdef MYDEBUG
#ifdef ADC_ADS1110
#undef ADC_ADS1110
#endif
#else
#include "ads1110.h"
#endif

#ifdef ADC_ADS1110

#define ADC_VREF_mV     (2048)
#define ADC_JINGDU      (32768)
//Range       xx     RMIN     RMAX      xx
//DEF_VALUE   VMIN   RMIN     RMAX      VMAX
//I_uA        0(uA)  4000(uA) 20000(uA) 20480(uA)
//U_mV        0(mV)  400(mV)  2000(mV)  2048(mV)
//ADC_RESULT  0                         65535
#define DEF_VALUE_MAX(MAX,MIN)   ( (long)MIN + ((long)MAX - (long)MIN) * (long)(2048 - 400) / (long)(2000 - 400) )
#define DEF_VALUE_MIN(MAX,MIN)   ( (long)MIN - ((long)MAX - (long)MIN) / (long)4 )
signed long Out_MAX(signed int max,signed int min)
{
    signed long temp;

    temp = max - min;
    temp *= (signed long)(2048-400);
    temp /= (signed long)(2000-400);
    temp += min;
    return temp;
}
signed long Out_MIN(signed int max,signed int min)
{
    signed long temp;

    temp = max - min;
    temp /= (signed long)(4);
    temp = min - temp;
    return temp;
}
#else

#define ADC_VREF_mV     (2500)
#define ADC_JINGDU      (1024)
//Range       xx     RMIN     RMAX     xx
//DEF_VALUE   VMIN   RMIN     RMAX     VMAX
//I_mA        0(mA)  4(mA)    20(mA)   25(mA)
//U_V         0(V)   0.4(V)   2(V)     2.5(V)
//ADC_RESULT  0                        1024
#define DEF_VALUE_MAX(MAX,MIN)   ( (long)MIN + ((long)MAX - (long)MIN) * (25 - 4) / (20 - 4) )
#define DEF_VALUE_MIN(MAX,MIN)   ( (long)MIN - ((long)MAX - (long)MIN) / 4 )
signed long Out_MAX(signed int max,signed int min)
{
    signed long temp;

    temp = max - min;
    temp *= (signed long)(25-4);
    temp /= (signed long)(20-4);
    temp += min;
    return temp;
}
signed long Out_MIN(signed int max,signed int min)
{
    signed long temp;

    temp = max - min;
    temp /= (signed long)(4);
    temp = min - temp;
    return temp;
}
#endif

/* ********************************************************************** */
#ifndef MYDEBUG
unsigned char ADDR_EEP __attribute__((section(".eeprom"))) = def_DeviceAddr;
unsigned char MEASURE_MODE_EEP __attribute__((section(".eeprom"))) = (-1);


unsigned int  I_MAX_EEP  __attribute__((section(".eeprom"))) = DEF_VALUE_MAX(I_RangeMAX,I_RangeMIN);
unsigned int  I_MIN_EEP  __attribute__((section(".eeprom"))) = DEF_VALUE_MIN(I_RangeMAX,I_RangeMIN);
unsigned char I_XSD_EEP  __attribute__((section(".eeprom"))) = I_DEF_XiaoShuDian;
unsigned int I_ZERO_EEP  __attribute__((section(".eeprom"))) = I_ZERO;

unsigned int  T_MAX_EEP  __attribute__((section(".eeprom"))) = DEF_VALUE_MAX(T_RangeMAX,T_RangeMIN);
unsigned int  T_MIN_EEP  __attribute__((section(".eeprom"))) = DEF_VALUE_MIN(T_RangeMAX,T_RangeMIN);
unsigned char T_XSD_EEP  __attribute__((section(".eeprom"))) = T_DEF_XiaoShuDian;
unsigned int T_ZERO_EEP  __attribute__((section(".eeprom"))) = T_ZERO;

unsigned int  Y_MAX_EEP  __attribute__((section(".eeprom"))) = DEF_VALUE_MAX(Y_RangeMAX,Y_RangeMIN);
unsigned int  Y_MIN_EEP  __attribute__((section(".eeprom"))) = DEF_VALUE_MIN(Y_RangeMAX,Y_RangeMIN);
unsigned char Y_XSD_EEP  __attribute__((section(".eeprom"))) = Y_DEF_XiaoShuDian;
unsigned int Y_ZERO_EEP  __attribute__((section(".eeprom"))) = Y_ZERO;

unsigned int  L_MAX_EEP  __attribute__((section(".eeprom"))) = DEF_VALUE_MAX(L_RangeMAX,L_RangeMIN);
unsigned int  L_MIN_EEP  __attribute__((section(".eeprom"))) = DEF_VALUE_MIN(L_RangeMAX,L_RangeMIN);
unsigned char L_XSD_EEP  __attribute__((section(".eeprom"))) = L_DEF_XiaoShuDian;
unsigned int L_ZERO_EEP  __attribute__((section(".eeprom"))) = L_ZERO;


static unsigned int  * pMAX_EEP;
static unsigned int  * pMIN_EEP;
static unsigned char * pXSD_EEP;
static unsigned int  * pADJ_ZERO;
#endif
static unsigned char MEASURE_MODE;
static unsigned int  ADJ_ZERO;
/* ********************************************************************** */
unsigned int READ_WORD_EEP(unsigned int * addr) {eeprom_busy_wait();return eeprom_read_word(addr);}
void WRITE_WORD_EEP(unsigned int x,unsigned int * addr) {eeprom_busy_wait();eeprom_write_word(addr,x);}
unsigned char READ_BYTE_EEP(unsigned char * addr) {eeprom_busy_wait();return eeprom_read_byte(addr);}
void WRITE_BYTE_EEP(unsigned char x,unsigned char * addr) {eeprom_busy_wait();eeprom_write_byte(addr,x);}
/* ********************************************************************** */
typedef enum _ErrorCode{
    ERRCODE_DeviceAddrErr = 1,
    ERRCODE_CRC16Error,
    ERRCODE_OnlyReceiveAddr,
    ERRCODE_ReceiveDataErr,
    ERRCODE_FunCodeInvalid,
    ERRCODE_ReceiveBitErr,
    ERRCODE_RXARRAYTEMP_MAXErr,
} ERRORCODE;
typedef enum _FunCode{
    GET_DATA = 3,
    GET_VALUE = 4,
    SET_DEVICEADDR = 6,
    CommunicationTest = 0x64, 
    SET_VALUE = 0x65,
    GET_ID = 0x66,
    GET_CHAR,
} FUNCODE;
#if 0
enum SELECT_GET_DATA{
    SEL_DATA_ADC,
    SEL_DATA_XSD,
    SEL_DATA_FREQ,
};
enum SELECT_SET_CODE{
    SEL_SET_ADDR,
    SEL_SET_VALUE,
    SEL_SET_XSD,
};
#endif
enum{
    ADDR_DEV = 0x0010,
    ADDR_CID,
    ADDR_PID,
    ADDR_VID,
    ADDR_FID,
    ADDR_XSD = 0x0022,
    ADDR_F1_VALUE,
    ADDR_F2_VALUE,
    ADDR_FW_MAX = 0x1016,
    ADDR_FW_MIN = 0x1018,
    ADDR_VALUE = 0x1020,
};
enum SELECT_GET_CHAR{
    SEL_CHAR_COMPANY,
    SEL_CHAR_DESCRIBE
};
typedef struct _MODBUSFrame{
    unsigned char Addr;
    FUNCODE FunCode;
    unsigned char *Data;
    unsigned int  Len;
    unsigned char CRCH;
    unsigned char CRCL;
} MODBUSFRAME;

typedef union {
    unsigned char ch[4];
    float F;
    //double D;
} FLOAT;
typedef union {
    unsigned char ch[4];
    long L;
    //double D;
} LONG;
static LONG V_MAX;
static LONG V_MIN;
static FLOAT V_DATA;

/* ********************************************************************** */
volatile unsigned char RxMode = 0;
volatile unsigned char RxFrameComplete = 0;
volatile unsigned char WorkMode = 0;
volatile unsigned char RXDataTemp = 0;
volatile MODBUSFRAME RxFrame;
volatile unsigned char DeviceAddr;
unsigned char RxArrayTemp[RXARRAYTEMP_MAX];
volatile unsigned int  ADCResult = 0;
volatile unsigned int  ErrFlag;
volatile unsigned int  ErrCodeDisplay;
volatile unsigned int  VALUE_MAX;
volatile unsigned int  VALUE_MIN;
volatile unsigned char XiaoShuDian;
volatile unsigned int  T_C;
/* ********************************************************************** */



/* ********************************************************************** */
#define TIMERDELAY
#ifdef TIMERDELAY
/* ---------------------------------------------------------------------- */
#define TimerEnable()       do{TIMSK |= (1 << TOIE1);}while(0)
#define TimerDisable()      do{TIMSK &= ~(1 << TOIE1);}while(0)

#define TimerStop()         do{TCCR1B &= ~((1 << CS12)|(1 << CS11)|1 << CS10);}while(0)
#define TimerStart()        do{TCCR1B |= (1 << CS11);}while(0)

#define SetTimerCount(v)    do{\
                            	TCNT1H = (unsigned char)((unsigned int)(v)>>8);\
								TCNT1L = (unsigned char)((unsigned int)(v)& 0xFF);\
							}while(0)
/* ---------------------------------------------------------------------- */
volatile unsigned int   TimerCycleI;//
volatile unsigned int   TimerCycleF;//
volatile unsigned char  TimerOver;//
/* ---------------------------------------------------------------------- */
void TimerDelayMs(unsigned int s)
{
	TimerCycleI = s >> 6;                   //TimerCycleN = s/64
	TimerCycleF = 65535 - (s & 0x3F)*1000;  //TimerCycleF = s%64
	TimerOver = 0;

	switch(TimerCycleI)
	{
		case 0:
			SetTimerCount(TimerCycleF);
			break;
		case -1:
			TimerDisable();
			TimerOver = 1;
			break; 
		default:
			SetTimerCount(1535);
			//TCNT1H = 1535/256;
			//TCNT1L = 1535%256;
			break;	
	}
	TimerEnable();
	TimerStart();
}
/* ---------------------------------------------------------------------- */
SIGNAL(SIG_OVERFLOW1)
{
	
	TimerCycleI--;
	switch(TimerCycleI)
	{
		case 0:
			SetTimerCount(TimerCycleF);
			break;
		case -1:
			TimerStop();
			TimerDisable();
			TimerOver = 1;
			return;
		default:
			SetTimerCount(1535);
			break;	
	}
}
#endif
/* ********************************************************************** */






/* ********************************************************************** */
void USART_Init(unsigned int baud)
{
    /* set baud rate */
    UBRRH = (F_CPU/baud/16-1)/256;
    UBRRL = (F_CPU/baud/16-1)%256;
    /* Enable Receiver and Transmitter Enable Receiver interrupt */
    UCSRB =(1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
    /* Set frame format: 8data,2stop bit */
    UCSRC = (1<<URSEL)|
        (0<<UMSEL)|    /* 0:Asynchronous 1:Synchronous */
        (0<<UPM0)|    /* 0:Disable 1:Reserved 2:Even Parity 3:Odd Parity */
        (0<<USBS)|    /* Stop Bit 0:1-bit 1:2-bit */
        (3<<UCSZ0);    /* Data bit 0 ~ 7 : 5,6,7,8,-,-,-,9 */
}
void USART_Transmit(unsigned char data)
{
    while(!(UCSRA & (1<<UDRE)));
    UDR = data;
}

#define putchar USART_Transmit
void printk(char *s)
{
    if(s == NULL || *s == '\0')
        return;
    TX485();
    while(*s != '\0'){
        putchar(*s++);
    }
    RX485();
}
/* ********************************************************************** */
void RxOverrunTimer(void)
{
    /* enabile Timer0 */
    TCNT2 = FrameInterval;
    TIFR |= (1<<TOV2);
    TIMSK |= (1<<TOIE2);
    RxFrameComplete = 0;
}
SIGNAL(SIG_OVERFLOW2)
{
    TIMSK &= ~(1<<TOIE2);
    TIFR |= (1<<TOV2);
#if 1
    switch(RxMode)
    {
        case 1: /* receive Addr only */
            //ErrorDisplay(ERRCODE_OnlyReceiveAddr);
            break;
        case 2: /* receive Addr and FunCode only */
            //ErrorDisplay(ERRCODE_ReceiveDataErr);
            break;
        case 3: /* receive All */
            if(RxFrame.Len < 2){
                ErrorDisplay(ERRCODE_ReceiveDataErr);
                RxFrame.Data = RxArrayTemp + 2;
                break;
            }
            RxFrameComplete = 1;
            RxFrame.CRCL = *(RxFrame.Data - 2);
            RxFrame.CRCH = *(RxFrame.Data - 1);
            RxFrame.Data = RxArrayTemp + 2;
            break;
        default :
            ErrorDisplay(ERRCODE_ReceiveDataErr);
    }
    /* Stop Timer2 */
    TIMSK &= ~(1<<TOIE2);
    TCNT2 = 0x00;
    RxMode = 0;
#else
    PORTD ^= (1<<6);
    RxOverrunTimer();
#endif
}
/* ********************************************************************** */
SIGNAL(SIG_UART_RECV)
{
    if(! (UCSRA & (1<<RXC))){
        return;
    }
    cli();
    TIMSK &= ~(1<<TOIE2);
    if(UCSRA & ((1<<FE)|(1<<DOR)|(1<<PE))){
        ErrorDisplay(ERRCODE_ReceiveBitErr);
    }
    RXDataTemp = UDR;
#if 0
    TX485();
    putchar(RXDataTemp);
    RX485();
#endif
    switch(RxMode)
    {
        case 0x00:
            RxFrame.Addr = RXDataTemp;
            RxArrayTemp[0] = RXDataTemp;
            RxMode = 0x01;
            break;
        case 0x01:
            RxFrame.FunCode = RXDataTemp;
            RxArrayTemp[1] = RXDataTemp;
            RxFrame.Data = RxArrayTemp + 2;
            RxFrame.Len = 0;
            RxMode = 0x02;
            break;
        case 0x02:
        case 0x03:
            *(RxFrame.Data++) = RXDataTemp;
            RxFrame.Len++;
            if(RxFrame.Len == (RXARRAYTEMP_MAX-2)){
                RxFrame.Data = RxArrayTemp + 2;
                RxFrame.Len = 0;
                ErrorDisplay(ERRCODE_RXARRAYTEMP_MAXErr);
            }
            RxMode = 0x03;
            break;
    }
    RxOverrunTimer();
    sei();
}
/* ********************************************************************** */
void ADCInit(void)
{
#ifdef ADC_ADS1110	
     Init_ADS1110();
#else
    ADMUX =  (0<<REFS0)|    /* 0:Aref,1:AVCC,2:-,3:2.56V */
             (1<<ADLAR)|    /* 0:right adjust,1:left adjust */
             (7<<MUX0);     /* 0~7 channel */
    ADCSRA = (1<<ADEN)|     /* ADC enable */
             (0<<ADSC)|     /* ADC start */
             (0<<ADFR)|     /* ADC Free Running Select */
             (0<<ADIF)|     /* ADC Interrupt Flag ,1:clear */
             (0<<ADIE)|     /* ADC Interrupt Enable */
             (0<<ADPS1);    /* ADC Prescaler Select Bits */
#endif
}
unsigned int ADCOutOne(void)
{
    unsigned int i;
    unsigned long sum = 0;

#ifdef ADC_ADS1110	
    #define ADC_COUNT 2
    for(i = 0;i < ADC_COUNT; i++){
        sum += ReadADS1110();
    }
    return ((unsigned int)(sum>>1));
#else
    #define ADC_COUNT 64
    unsigned int ret = 0;
    unsigned char tmpH,tmpL;

    for(i = 0;i < ADC_COUNT; i++)
    {
        ADCSRA |= (1<<ADSC);  //Æô¶¯×ª»»
        while(ADCSRA & (1<<ADSC));
        tmpL = ADCL;
        tmpH = ADCH;
        ret =(unsigned int)(tmpH << 2);
        ret = ret + (tmpL>>6 & 0x3);
	sum = sum + ret;
    }

    return sum>>6;//
#endif

}
unsigned int ADCOut(void)
{
    unsigned int temp;
    unsigned int tempMAX = 0,tempMIN = 0;
    unsigned long SUM = 0;
    unsigned char i;
    
    tempMAX = ADCOutOne();
    tempMIN = ADCOutOne();
    
    for(i = 0;i < 34;i++)
    {
        temp = ADCOutOne();
	if(temp < tempMIN)
	    tempMIN = temp;
        if(temp > tempMAX)
	    tempMAX = temp;
	SUM = SUM + temp;
    }
    SUM = SUM - tempMAX - tempMIN;
    temp = (unsigned int)(SUM>>5);
    return  temp;
}
unsigned int ChangeADCResult(void)
{
    long temp;

    ADCResult = ADCOut();
    ADCResult += ADJ_ZERO;
    temp = ((long) ADCResult * (long)(VALUE_MAX - VALUE_MIN)) /(long)ADC_JINGDU + VALUE_MIN ;
    ADCResult = (unsigned int) temp;

    return ADCResult;
}
void RefreshLED(void)
{
    unsigned int T_C_temp;
    T_C_temp = ChangeADCResult();
    if(T_C > T_C_temp){
        if( (T_C - T_C_temp) > 2 )
	{
            T_C = T_C_temp;
    	    Display10(T_C_temp);
	}
    } else {
        if( (T_C_temp - T_C) > 2 )
	{
            T_C = T_C_temp;
    	    Display10(T_C_temp);
	}
    }
#ifdef TIMERDELAY
    if(TimerOver != 0)
    {
        /* change T_C value 1S */
        T_C = ChangeADCResult();
        Display10(T_C);
        /*  */
        TimerOver = 0;
        TimerDelayMs(1000);
	/* D2 LED 1S flicker */
        PORTD ^= (1<<7);
	if(ErrFlag !=0)
            ErrFlag ++;
    }
#endif
}
void SendDataToPC(FUNCODE Fcode, unsigned char * SendTemp, unsigned int Len)
{
#if 0
    MODBUSFRAME ModbusTemp;
    unsigned short RxCRC16;

    ModbusTemp.Addr = DeviceAddr;
    ModbusTemp.FunCode = Fcode;
    ModbusTemp.Data = SendTemp;
    ModbusTemp.Len = Len;
    RxCRC16 = CRC16(ModbusTemp.Data, ModbusTemp.Len);
    ModbusTemp.CRCH = (unsigned char) (RxCRC16 >> 8);
    ModbusTemp.CRCL = (unsigned char) (RxCRC16 & 0xFF);
    
    TX485();
    putchar(ModbusTemp.Addr);
    putchar(ModbusTemp.FunCode);
    //putchar( (unsigned char)(Len >> 8) );
    if(Fcode != SET_DEVICEADDR)
        putchar( (unsigned char)(Len & 0xFF) );
    while(Len--){
        putchar(*(SendTemp++));
    }
    putchar(ModbusTemp.CRCL);
    putchar(ModbusTemp.CRCH);
    _delay_ms(10);
    RX485();
#else
    unsigned char ModbusTemp[RXARRAYTEMP_MAX];
    unsigned short RxCRC16;
    unsigned char i;
    unsigned char count;
    ModbusTemp[0] = DeviceAddr;
    ModbusTemp[1] = Fcode;

    if(Fcode != SET_DEVICEADDR){
        count = 3;
        ModbusTemp[2] = (unsigned char)(Len & 0xFF);
    }else{
        count = 2;
    }
    for(i = 0;i < Len;i++){
        ModbusTemp[i+count] = SendTemp[i];
    }
    RxCRC16 = CRC16(ModbusTemp, Len + count);

    TX485();

    for(i = 0;i < Len + count;i++){
        putchar(ModbusTemp[i]);
    }
    
    putchar((unsigned char) (RxCRC16 & 0xFF));
    putchar((unsigned char) (RxCRC16 >> 8));
    
    _delay_ms(10);
    RX485();
#endif
}
unsigned char FrameCRC16IsOK(MODBUSFRAME Temp)
{
    unsigned char i;
    unsigned int CRCtemp;
    unsigned char sch[RXARRAYTEMP_MAX];
    sch[0] = Temp.Addr;
    sch[1] = Temp.FunCode;
    for(i=0;i<(Temp.Len-2);i++,Temp.Data++)
        sch[i+2] = *(Temp.Data);
    CRCtemp = CRC16(sch, Temp.Len);

    if(Temp.CRCL != (unsigned char)CRCtemp)
        return 0;
    if(Temp.CRCH != (unsigned char)(CRCtemp>>8))
        return 0;
    return 1;
}
void ErrorDisplay(unsigned int ErrCode)
{
    ErrFlag = 1;
    ErrCode &= 0x0FFF;
    ErrCodeDisplay = ErrCode | 0xE000;

}
void IsErrorOrNot(void)
{
    if(ErrFlag)
    {
        if(ErrFlag > 2);
            ErrFlag = 0;
        DisplayHex(ErrCodeDisplay);
    }
}
/* ********************************************************************** */
void SystemInit(void)
{
//MODE

#ifdef MYDEBUG
    MEASURE_MODE = 0;
    DeviceAddr  = def_DeviceAddr;
    VALUE_MAX   = DEF_VALUE_MAX(I_RangeMAX,I_RangeMIN);
    VALUE_MIN   = DEF_VALUE_MIN(I_RangeMAX,I_RangeMIN);
    XiaoShuDian = I_DEF_XiaoShuDian;
    ADJ_ZERO    = 0;
#else
    MEASURE_MODE = READ_BYTE_EEP(&MEASURE_MODE_EEP);
    MEASURE_MODE ++;
    if(MEASURE_MODE>3)
        MEASURE_MODE = 0;
    DeviceAddr   = READ_BYTE_EEP(&ADDR_EEP);

    switch(MEASURE_MODE){
    case 0:
	pMAX_EEP = &I_MAX_EEP;
	pMIN_EEP = &I_MIN_EEP;
	pXSD_EEP = &I_XSD_EEP;
        pADJ_ZERO = &I_ZERO_EEP;
	break;
    case 1:
	pMAX_EEP = &T_MAX_EEP;
	pMIN_EEP = &T_MIN_EEP;
	pXSD_EEP = &T_XSD_EEP;
        pADJ_ZERO = &T_ZERO_EEP;
	break;
    case 2:
	pMAX_EEP = &Y_MAX_EEP;
	pMIN_EEP = &Y_MIN_EEP;
	pXSD_EEP = &Y_XSD_EEP;
	pADJ_ZERO = &Y_ZERO_EEP;
	break;
    case 3:
	pMAX_EEP = &L_MAX_EEP;
	pMIN_EEP = &L_MIN_EEP;
	pXSD_EEP = &L_XSD_EEP;
	pADJ_ZERO = &L_ZERO_EEP;
	break;
    default:
	MEASURE_MODE = 0;
        break;
    }
    WRITE_BYTE_EEP(MEASURE_MODE,&MEASURE_MODE_EEP);

    VALUE_MAX   = READ_WORD_EEP(pMAX_EEP);
    VALUE_MIN   = READ_WORD_EEP(pMIN_EEP);
    XiaoShuDian = READ_BYTE_EEP(pXSD_EEP);
    ADJ_ZERO    = READ_WORD_EEP(pADJ_ZERO);
#endif    
//IO Direction
    DDRC = 0x0F;
    DDRB = 0x07;
    DDRD = 0xFE;
//enable interrupt 
    sei();
//Display Open
    OpenDisplayLED();
//485 enabile
    RX485();
//Open Uart
    USART_Init(UartBaud);
//Set Timer2 clk
    /* clock select clk/128 */
    TCCR2 = Timer2ClkSel;
//Enable ADC
    ADCInit();
//Enable watchdog
    wdt_enable(WDTO_1S);
//
#ifdef TIMERDELAY
    TimerOver = 0;
    TimerDelayMs(1000);
#endif

}

int main(void)
{
    unsigned char SendTemp[10];
    unsigned int tempint;
    long tempA = 0;
    long tempB = 0;
    unsigned char CRCYesOrNo;
    unsigned char tempch;
    //unsigned int i = 0;
    /* 0:wait receive frame 1:receive frame finish */
    RxFrameComplete = 0;
    /* Init Receive status mode*/
    RxMode = 0;
    /* Error Flag clear */
    ErrFlag = 0;
    /* System init */
    SystemInit();
    //RxOverrunTimer();
#ifndef MYDEBUG
    ErrCodeDisplay = MEASURE_MODE + 0xD000;
    DisplayHex(ErrCodeDisplay);
    for(tempint = 0;tempint < 10;tempint ++)
    {
    	_delay_ms(200);
    	wdt_reset();
    }
#endif
    while(1)
    {
        if(!RxFrameComplete)
        {
	    //if(ErrFlag == 0)
 	        RefreshLED();
            IsErrorOrNot();
            wdt_reset();
            continue;
        }
	
        /* clear receive finish flag */
        RxFrameComplete = 0;
        /* Judge Addr is OK */
        if(RxFrame.Addr != DeviceAddr)
        {
            //ErrorDisplay(ERRCODE_DeviceAddrErr);
            continue;
        }
        /* Judge CRC16 is OK */
        if(!FrameCRC16IsOK(RxFrame))
        {
            ErrorDisplay(ERRCODE_CRC16Error);
             CRCYesOrNo = 0x80;
        }else{
             CRCYesOrNo = 0;
	}
        tempint = (unsigned int)RxFrame.Data[0];
        tempint <<= 8;
        tempint |= (unsigned int)RxFrame.Data[1];
        switch(RxFrame.FunCode){
            case CommunicationTest:
		SendDataToPC(CommunicationTest, RxFrame.Data, RxFrame.Len - 2);
                break;
	    case GET_VALUE:
                switch(tempint){
                    case ADDR_VALUE:/* Get ADC data */
                        V_DATA.F = (float)T_C;
                        SendTemp[0] = V_DATA.ch[3];
                        SendTemp[1] = V_DATA.ch[2];
                        SendTemp[2] = V_DATA.ch[1];
                        SendTemp[3] = V_DATA.ch[0];
			SendDataToPC(GET_VALUE | CRCYesOrNo, SendTemp , 4);
                        break;
		    default:
			SendTemp[0] = 0x00;
                        SendDataToPC(GET_VALUE | 0x80, SendTemp , 1);
                        break;
		}
		break;
            case GET_DATA:
                switch(tempint)
                {
                    case ADDR_XSD:/* Get XiaoShuDian Data */
                        SendTemp[0] = 0;
                        SendTemp[1] = XiaoShuDian;
			SendDataToPC(GET_DATA | CRCYesOrNo, SendTemp , 2);
                        break;
                    case ADDR_F1_VALUE:/* Get Freq Data */
                        SendTemp[0] = 0x00;
                        SendTemp[1] = 0x00;
			SendDataToPC(GET_DATA | CRCYesOrNo, SendTemp , 2);
                        break;
                    case ADDR_F2_VALUE:/* Get Freq Data */
                        SendTemp[0] = 0x00;
                        SendTemp[1] = 0x00;
			SendDataToPC(GET_DATA | CRCYesOrNo, SendTemp , 2);
                        break;
                    case ADDR_FW_MAX:
			SendTemp[0] = V_MAX.ch[3];
			SendTemp[1] = V_MAX.ch[2];
			SendTemp[2] = V_MAX.ch[1];
			SendTemp[3] = V_MAX.ch[0];

                        SendTemp[4] = V_MIN.ch[3];
			SendTemp[5] = V_MIN.ch[2];
			SendTemp[6] = V_MIN.ch[1];
			SendTemp[7] = V_MIN.ch[0];
			SendDataToPC(GET_DATA | CRCYesOrNo, SendTemp , 8);
                        break;
		    default:
			SendTemp[0] = 0x00;
                        CRCYesOrNo = 0x80;
			SendDataToPC(GET_DATA | CRCYesOrNo, SendTemp , 1);
                        break;
                }
                break;
            case SET_DEVICEADDR:
                switch(tempint)
                {
                    case ADDR_DEV:/* change device addr */
                        if(RxFrame.Data[3] == 0x00){
                            break;
                        }
                        tempch = RxFrame.Data[3];
                        //SendTemp[0] = DeviceAddr;
                        SendTemp[0] = RxFrame.Data[0];
                        SendTemp[1] = RxFrame.Data[1];
                        SendTemp[2] = 0x00;
			SendTemp[3] = tempch;
                        SendDataToPC(SET_DEVICEADDR | CRCYesOrNo, SendTemp , 4);
                        DeviceAddr = tempch;
#ifndef MYDEBUG
                        WRITE_BYTE_EEP(DeviceAddr,&ADDR_EEP);
#endif
			break;
     		    case ADDR_XSD:
			XiaoShuDian =  RxFrame.Data[3];
			
			//SendTemp[0] = XiaoShuDian;
			SendTemp[0] = RxFrame.Data[0];
			SendTemp[1] = RxFrame.Data[1];
			SendTemp[2] = 0x00;
			SendTemp[3] = XiaoShuDian;
			SendDataToPC(SET_DEVICEADDR | CRCYesOrNo, SendTemp , 4);
#ifndef MYDEBUG			
			WRITE_BYTE_EEP(XiaoShuDian,pXSD_EEP);
#endif
			break;
                    default:
                        SendTemp[0] = 0x00;
                        SendDataToPC(SET_DEVICEADDR | 0x80, SendTemp , 1);
                        break;
                }
                break;
            case SET_VALUE:
#if 1
		switch(tempint){
                    case ADDR_FW_MAX:

			V_MAX.ch[0] = RxFrame.Data[5];
			V_MAX.ch[1] = RxFrame.Data[6];
			V_MAX.ch[2] = RxFrame.Data[7];
			V_MAX.ch[3] = RxFrame.Data[8];

			V_MIN.ch[0] = RxFrame.Data[9];
			V_MIN.ch[1] = RxFrame.Data[10];
			V_MIN.ch[2] = RxFrame.Data[11];
			V_MIN.ch[3] = RxFrame.Data[12];

			tempA =(unsigned int) V_MAX.L;
			tempB =(unsigned int) V_MIN.L;

		        VALUE_MAX = Out_MAX(tempA,tempB);
			VALUE_MIN = Out_MIN(tempA,tempB);

			SendTemp[0] = RxFrame.Data[0];
			SendTemp[1] = RxFrame.Data[1];
			SendTemp[2] = 0x08;

                        SendDataToPC(SET_DEVICEADDR | CRCYesOrNo, SendTemp , 3);
#ifndef MYDEBUG
			WRITE_WORD_EEP(VALUE_MAX,pMAX_EEP);
			WRITE_WORD_EEP(VALUE_MIN,pMIN_EEP);			
			WRITE_WORD_EEP(VALUE_MAX,pMAX_EEP);
			WRITE_WORD_EEP(VALUE_MIN,pMIN_EEP);
#endif		
		}
		break;
#endif
     	    case GET_ID:
		SendTemp[0] = 0x00;
                switch(tempint)
                {
                    case ADDR_CID:
                        SendTemp[1] = CODE_CID;
                        break;
                    case ADDR_PID:
                        SendTemp[1] = CODE_PID;
                        break;
                    case ADDR_VID:
                        SendTemp[1] = CODE_VID;
                        break;
                    case ADDR_FID:
                        SendTemp[1] = MEASURE_MODE;
                        break;
		    default:
                        SendTemp[0] = 0x00;
                        CRCYesOrNo = 0x80;
                        break;
                }
                SendDataToPC(GET_ID | CRCYesOrNo, SendTemp , 2);
                break;
#if 0
            case GET_CHAR:
                switch(tempint){
                    case SEL_CHAR_COMPANY:
                        printk(CHAR_COMPANY);
                        break;
                    case SEL_CHAR_DESCRIBE:
                        printk(CHAR_DESCRIBE);
                        break;
                    default:
                        SendTemp[0] = 0x00;
                        SendDataToPC(GET_CHAR | 0x80, SendTemp , 1);
                        break;
                }
                break;
#endif
	    default:
                RxFrame.FunCode |= 0x80;
                SendDataToPC(RxFrame.FunCode, RxFrame.Data, RxFrame.Len - 2);
                ErrorDisplay(ERRCODE_FunCodeInvalid);
                PORTD ^= (1<<6);
		break;
        }
        
    }
    return 0;
}





