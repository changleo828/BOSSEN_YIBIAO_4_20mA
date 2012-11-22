#ifndef _LED_H_
#define _LED_H_

#include <avr/io.h>

unsigned char OpenDisplayLED(void);
void DisplayHex(unsigned int TempData);
void Display10(unsigned int TempData);
#endif
