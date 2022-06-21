#include "Cradle.h"

void Cradle_ArriveNotify(uint8_t TZx)
{
    if (TZx < 1 || TZx > 3) return;
    CanTxMsg TxMessage;
    TxMessage.StdId = CRADLE_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0xAA;
    TxMessage.Data[1] = TZx;
    CAN2_Send(&TxMessage);
}

void Cradle_RestartNotify(uint8_t TZx)
{
    if (TZx < 1 || TZx > 3) return;
    CanTxMsg TxMessage;
    TxMessage.StdId = CRADLE_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0xDD;
    TxMessage.Data[1] = TZx;
    CAN2_Send(&TxMessage);
}

void Cradle_RetryNotify(uint8_t TZx)
{
    if (TZx < 1 || TZx > 2) return;
    CanTxMsg TxMessage;
    TxMessage.StdId = CRADLE_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = TZx;
    CAN2_Send(&TxMessage);
}

void Cradle_ReturnNotify()
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CRADLE_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0xCC;
    TxMessage.Data[1] = 0x01;
    CAN2_Send(&TxMessage);
}