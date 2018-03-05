#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx.h"

extern CanTxMsg CAN1TxMessage;
extern CanRxMsg CAN1RxMessage;
extern CanTxMsg CAN2TxMessage;
extern CanRxMsg CAN2RxMessage;

void Dual_CAN_Init(void);
uint8_t CAN1_Send(void);
uint8_t CAN2_Send(void);
void CAN1_RX0_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);

#endif
