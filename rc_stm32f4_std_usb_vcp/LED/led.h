#ifndef __LED_H
#define __LED_H

#include "sys.h"

#define LED0 PCout(0)
#define LED1 PCout(2)
#define LED2 PCout(3)

void LEDInit(void);

#endif
