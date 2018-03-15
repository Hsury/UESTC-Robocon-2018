#include "Probe.h"

void Probe_SetTimer(uint8_t Index, uint32_t Timer)
{
    if (Index > 7) return;
    CanTxMsg TxMessage;
    TxMessage.StdId = PROBE_TIMER_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 5;
    TxMessage.Data[0] = Index;
    memcpy(&TxMessage.Data[1], &Timer, 4);
    CAN2_Send(&TxMessage);
}

void Probe_SetArrive(uint8_t TZx)
{
    if (TZx < 1 || TZx > 3) return;
    CanTxMsg TxMessage;
    TxMessage.StdId = PROBE_INTERACT_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0xAA;
    TxMessage.Data[1] = TZx;
    CAN2_Send(&TxMessage);
}
