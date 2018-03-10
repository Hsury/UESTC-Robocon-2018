#ifndef __CAN_H
#define __CAN_H

#include "Includes.h"

#define ELMO1_CAN_ID    0x701
#define ELMO2_CAN_ID    0x702
#define ELMO3_CAN_ID    0x703

#define GE_POS_CAN_ID   0x12
#define GE_ANG_CAN_ID   0x11
#define GE_CTRL_CAN_ID  0x15

#define KEYBOARD_CAN_ID 0x30

void Dual_CAN_Init(void);
uint8_t CAN1_Send(CanTxMsg* TxMessage);
uint8_t CAN2_Send(CanTxMsg* TxMessage);
void CAN1_RX0_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);

#endif
