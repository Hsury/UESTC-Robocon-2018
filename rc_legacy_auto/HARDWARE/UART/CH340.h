#ifndef __CH340_H
#define __CH340_H

#include "Includes.h"

void CH340_Init(uint32_t Baudrate);
void USART3_IRQHandler(void);

#endif
