#ifndef __CAN_H
#define __CAN_H

#include "Includes.h"

#define ELMO1_CAN_ID                0x701
#define ELMO2_CAN_ID                0x702
#define ELMO3_CAN_ID                0x703

#define GE_POS_CAN_ID               0x12
#define GE_ANG_CAN_ID               0x11
#define GE_CTRL_CAN_ID              0x15

#define DT35_X_CAN_ID               0x26
#define DT35_Y_CAN_ID               0x24

#define KEYBOARD_CAN_ID             0x30

#define CRADLE_CAN_ID               0x72

#define START_DASH_CAN_ID           0x73
#define EXIT_DASH_CAN_ID            0x74

#define PROBE_ONLINE_CAN_ID         0xA0
#define PROBE_USER_CAN_ID           0xA1
#define PROBE_TIMER_CAN_ID          0xA2

void Dual_CAN_Init(void);
uint8_t CAN1_Send(CanTxMsg* TxMessage);
uint8_t CAN2_Send(CanTxMsg* TxMessage);

#endif
