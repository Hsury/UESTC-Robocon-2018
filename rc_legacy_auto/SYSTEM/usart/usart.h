#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"

#define EN_USART1_RX 1 //使能（1）/禁止（0）串口1接收

void uart_init(u32 bound);

#endif
