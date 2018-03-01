#ifndef __AirUART_H
#define __AirUART_H

#include "stm32f4xx_conf.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

enum AirUART_STATEMACHINE
{
    HEAD,
    SEPERATOR,
    RST,
    ACK,
    DATA_TAIL,
    RST_TAIL,
    ACK_TAIL,
    FIN
};

enum AirUART_RESPONSE
{
    UNKNOWN,
    SUCC,
    FAIL
};

void AirUART_Init(uint32_t Baudrate);

void AirUART_Send(uint8_t *Data, uint8_t Len);
uint8_t AirUART_Send_And_Wait(uint8_t *Data, uint8_t Len, uint8_t RetryTimes);
uint8_t AirUART_Get_Response(void);

uint8_t AirUART_Available(void);
uint8_t AirUART_Receive(uint8_t *Data);

static void AirUART_Receive_Callback(uint8_t *Data, uint8_t Len);
void USART2_IRQHandler(void);

static void AirUART_Send_Raw(uint8_t Byte);
static void AirUART_Send_RST(void);
static void AirUART_Send_ACK(void);
static uint16_t AirUART_Calc_CRC(uint8_t *Data, uint16_t Len);

#endif
