#include "RM820R.h"
#include "RC_CAN.h"

RM820R_TypeDef RM820R[4];
CAN_HandleTypeDef* RM820R_CAN;

void RM820R_Init(CAN_HandleTypeDef* CAN, CanTxMsgTypeDef* TxMsg, CanRxMsgTypeDef* RxMsg)
{
    CAN_Init(CAN, TxMsg, RxMsg);
    RM820R_CAN = CAN;
}

void RM820R_SetCurrentAll(int16_t Current1=0, int16_t Current2=0, int16_t Current3=0, int16_t Current4=0)
{
    uint8_t Buffer[8];
    memcpy(&Buffer[0], &Current1, 2);
    memcpy(&Buffer[2], &Current2, 2);
    memcpy(&Buffer[4], &Current3, 2);
    memcpy(&Buffer[6], &Current4, 2);
    RM820R[0].Current = Current1;
    RM820R[1].Current = Current2;
    RM820R[2].Current = Current3;
    RM820R[3].Current = Current4;
    CAN_Send(RM820R_CAN, &Buffer, 8)
}

void RM820R_SetCurrentSingle(uint8_t ID, int16_t Current=0)
{
    if (ID < 1 || ID > 4) return;
    uint8_t Buffer[8];
    memcpy(&Buffer[0], (ID == 1) ? &Current : &RM820R[0].Current, 2);
    memcpy(&Buffer[2], (ID == 2) ? &Current : &RM820R[1].Current, 2);
    memcpy(&Buffer[4], (ID == 3) ? &Current : &RM820R[2].Current, 2);
    memcpy(&Buffer[6], (ID == 4) ? &Current : &RM820R[3].Current, 2);
    RM820R[ID - 1].Current = Current;
    CAN_Send(RM820R_CAN, &Buffer, 8)
}
