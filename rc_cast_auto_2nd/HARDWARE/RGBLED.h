#ifndef __RGBLED_H
#define __RGBLED_H

#include "Includes.h"

#define RGBShutdown()    RGBLED_Set(0, 0, 0)
#define RGBShowRed()     RGBLED_Set(1, 0, 0)
#define RGBShowGreen()   RGBLED_Set(0, 1, 0)
#define RGBShowBlue()    RGBLED_Set(0, 0, 1)
#define RGBShowYellow()  RGBLED_Set(1, 1, 0)
#define RGBShowPurple()  RGBLED_Set(1, 0, 1)
#define RGBShowCyan()    RGBLED_Set(0, 1, 1)
#define RGBShowWhite()   RGBLED_Set(1, 1, 1)

void RGBLED_Init(void);
void RGBLED_Set(uint8_t R, uint8_t G, uint8_t B);

#endif
