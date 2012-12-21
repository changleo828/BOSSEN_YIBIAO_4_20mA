#include <avr\io.h>



typedef unsigned char    BYTE;
typedef unsigned short   WORD;
typedef unsigned long    DWORD;
#define uchar  unsigned char
volatile struct ADS1110_READ_REG
{
    unsigned char Addr;
    unsigned int DataREG;
    union
    {
        unsigned char ConfigREG;
        struct
	{
	    unsigned char ST_DRDY:1;
	    unsigned char :2;
	    unsigned char SC:1;
	    unsigned char DR1:1;
	    unsigned char DR0:1;
	    unsigned char PGA1:1;
	    unsigned char PGA0:1;
        };
    };
} ADS1110_READ_DATA;
volatile struct ADS1110_SETUP_REG
{
    unsigned char Addr;
    union
    {
        unsigned char ConfigREG;
        struct
	{
	    unsigned char ST_DRDY:1;
	    unsigned char :2;
	    unsigned char SC:1;
	    unsigned char DR1:1;
	    unsigned char DR0:1;
	    unsigned char PGA1:1;
	    unsigned char PGA0:1;
        };
    };
} ADS1110_SETUP_DATA;


#define ADS1110_ADDR  0x90

#define MSB(word)      		(BYTE)(((WORD)(word) >> 8) & 0xff)
#define LSB(word)      		(BYTE)((WORD)(word) & 0xff)


#define SDA_SET_OUT DDRC |= (1<<4)
#define SDA_SET_IN  DDRC &= ~(1<<4)
#define SCL_SET_OUT DDRC |= (1<<5)
#define SCL_SET_IN  DDRC &= ~(1<<5)

#define READ_SDA    (PINC & (1<<4))
#if 0
#define SDA0  asm("NOP");\
	      PORTC &= ~(1<<4);\
              asm("NOP");\
              asm("NOP")
#define SDA1  asm("NOP");\
	      PORTC |= (1<<4);\
              asm("NOP");\
              asm("NOP")
#define SCL0  asm("NOP");\
	      PORTC &= ~(1<<5);\
              asm("NOP");\
              asm("NOP");\
              asm("NOP");\
              asm("NOP");\
              asm("NOP");\
              asm("NOP")

#define SCL1  asm("NOP");\
	      PORTC |= (1<<5);\
              asm("NOP");\
              asm("NOP");\
              asm("NOP");\
              asm("NOP");\
              asm("NOP");\
              asm("NOP")
#else
#define SDA0  asm("NOP");\
	      PORTC &= ~(1<<4);\
              asm("NOP");\
              asm("NOP")
#define SDA1  asm("NOP");\
	      PORTC |= (1<<4);\
              asm("NOP");\
              asm("NOP")
#define SCL0  asm("NOP");\
	      PORTC &= ~(1<<5);\
              asm("NOP")

#define SCL1  asm("NOP");\
	      PORTC |= (1<<5);\
              asm("NOP")

#endif

//IIC起始信号
void IIC_Start(void)
{
    SDA1;
    SCL1;
    SDA0;
    SCL0;
}
//IIC结束信号
void IIC_Stop(void)
{
    SCL0;
    SDA0;
    SCL1;
    SDA1;
}
void IIC_Send_Byte(uchar Data)
{
    unsigned char cnt;
    for(cnt=0;cnt<8;cnt++){
        if((Data<<cnt)&0x80){
	    SDA1;
	}else{
	    SDA0;
	}
        SCL1;
	asm("NOP");
	asm("NOP");
	SCL0;
    }
    SCL0;
    SDA0;
    SCL1;
    asm("NOP");
    asm("NOP");
    SCL0;
}
void IIC_Receive_Byte(uchar * Data)
{
    unsigned char cnt;
    *Data = 0;
    SCL0;
    SDA_SET_IN;
    SDA0;
    for(cnt=0;cnt<8;cnt++){
    	SCL0;
        SCL1;
        (*Data) <<= 1;
	if(READ_SDA){
	    (*Data) |= 0x01;
	}

    }
    SCL0;
    SDA0;
    SDA_SET_OUT;
    SCL1;
    asm("NOP");
    asm("NOP");
    SCL0;
}
void SetUpADS1110(unsigned char x)
{
    IIC_Start();
    IIC_Send_Byte(ADS1110_ADDR & 0xFE);
    IIC_Send_Byte(x);
    IIC_Stop();
}
#if 1
void Init_ADS1110(void)
{
    SDA_SET_OUT;
    SCL_SET_OUT;
    SetUpADS1110(0x8C);
}
unsigned int ReadADS1110(void)
{
    unsigned int data = 0;
    unsigned char tmpH = 0;
    unsigned char tmpL = 0;
    unsigned char setup = 0;
    IIC_Start();
    IIC_Send_Byte(ADS1110_ADDR | 0x01);
    IIC_Receive_Byte(&tmpH);
    IIC_Receive_Byte(&tmpL);
    IIC_Receive_Byte(&setup);
    IIC_Stop();
    data = (unsigned int)tmpH << 8;
    data += tmpL;
    return data;
}
#else
void Init_ADS1110(void)
{
    SDA_SET_OUT;
    SCL_SET_OUT;
    SetUpADS1110(0x1C);
}
unsigned int ReadADS1110(void)
{
    unsigned int data = 0;
    unsigned char tmpH = 0;
    unsigned char tmpL = 0;
    unsigned char setup = 0;
    int i;
    SetUpADS1110(0x1C | 0x80);
    for(i = 0;i < 500;i++);
    do{
    i--;
    IIC_Start();
    IIC_Send_Byte(ADS1110_ADDR | 0x01);
    IIC_Receive_Byte(&tmpH);
    IIC_Receive_Byte(&tmpL);
    IIC_Receive_Byte(&setup);
    IIC_Stop();
    }while(((setup | 0x80) == 0x80) && (i > 0));
    if(i<0)
        return 0;
    data = (unsigned int)tmpH << 8;
    data += tmpL;
    return data;
}

#endif








