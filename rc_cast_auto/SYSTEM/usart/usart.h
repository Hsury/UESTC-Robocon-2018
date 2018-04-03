#ifndef __USART_H
#define __USART_H

#define AirUART USART2
#define CH340   USART3
#define ESP8266 UART4

#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"

void UART_Init(uint32_t Baudrate);
void Printf(USART_TypeDef* USARTx, const char * format, ...);

#endif
