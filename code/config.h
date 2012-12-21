#ifndef _config_h_
#define _config_h_

#include <avr/io.h>
//config
/* ********************************************************************** */
#define CODE_CID        (0x88)
#define CODE_PID        (0x80)
#define CODE_VID        (0x00)

#define ID0        (0x1234)
#define ID1        (0x5678)

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

#define AUTO_ZERO (-75)

#define I_RangeMAX        (2000L)
#define I_RangeMIN        (400L)
#define I_DEF_XiaoShuDian (3)
#ifdef ADC_ADS1110
#define I_ZERO            AUTO_ZERO
#else
#define I_ZERO            (0)
#endif

#define T_RangeMAX        (800L)
#define T_RangeMIN        (0L)
#define T_DEF_XiaoShuDian (2)
#ifdef ADC_ADS1110
#define T_ZERO            AUTO_ZERO
#else
#define T_ZERO            (0)
#endif

#define Y_RangeMAX        (2000L)
#define Y_RangeMIN        (400L)
#define Y_DEF_XiaoShuDian (3)
#ifdef ADC_ADS1110
#define Y_ZERO            AUTO_ZERO
#else
#define Y_ZERO            (0)
#endif

#define L_RangeMAX        (2000L)
#define L_RangeMIN        (400L)
#define L_DEF_XiaoShuDian (2)
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
#endif//_config_h_
