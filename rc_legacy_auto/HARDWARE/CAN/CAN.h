#ifndef __CAN_H
#define __CAN_H

#include "Includes.h"

#define ELMO1_CAN_ID                0x701
#define ELMO2_CAN_ID                0x702
#define ELMO3_CAN_ID                0x703
#define ELMO4_CAN_ID                0x704
#define ELMO5_CAN_ID                0x705

#define SLIDER_POS_CAN_ID           0x284
#define TONG_POS_CAN_ID             0x285

#define GE_POS_CAN_ID               0x12
#define GE_ANG_CAN_ID               0x11
#define GE_CTRL_CAN_ID              0x15

#define GY53_A_CAN_ID               0x21
#define GY53_B_CAN_ID               0x22

#define KEYBOARD_CAN_ID             0x30

#define CRADLE_CAN_ID               0x72

#define PROBE_ONLINE_CAN_ID         0xA0
#define PROBE_USER_CAN_ID           0xA1
#define PROBE_TIMER_CAN_ID          0xA2
#define PROBE_INTERACT_CAN_ID       0x72

void Dual_CAN_Init(void);
uint8_t CAN1_Send(CanTxMsg* TxMessage);
uint8_t CAN2_Send(CanTxMsg* TxMessage);
void CAN1_RX0_IRQHandler(void);
void CAN2_RX0_IRQHandler(void);

#endif
