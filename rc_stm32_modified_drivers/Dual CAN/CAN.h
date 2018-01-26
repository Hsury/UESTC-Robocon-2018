#ifndef __CAN_H
#define __CAN_H

extern CanTxMsg CAN1TxMessage;
extern CanRxMsg CAN1RxMessage;
extern CanTxMsg CAN2TxMessage;
extern CanRxMsg CAN2RxMessage;

void CAN_Init();
u8 CAN1_Send();
u8 CAN2_Send();
void CAN1_RX0_IRQHandler();
void CAN2_RX1_IRQHandler();

#endif
