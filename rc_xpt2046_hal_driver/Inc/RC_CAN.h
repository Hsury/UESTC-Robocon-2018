#ifndef __RC_CAN_H
#define __RC_CAN_H

#include "can.h"
#include "string.h"

#define DEVICE_CAN_ID 0x15

void CAN_Init(CAN_HandleTypeDef* CAN, CanTxMsgTypeDef* TxMsg, CanRxMsgTypeDef* RxMsg);
uint8_t CAN_Send(CAN_HandleTypeDef* CAN, uint8_t* Data, uint8_t Length);

#endif
