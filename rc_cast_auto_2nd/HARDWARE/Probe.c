#include "Probe.h"

void Probe_ReportOnline()
{
    CanTxMsg TxMessage;
    TxMessage.StdId = PROBE_ONLINE_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 4;
    memcpy(&TxMessage.Data[0], &Online, 4);
    CAN2_Send(&TxMessage);
}

void Probe_SetUser(float Data)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = PROBE_USER_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = sizeof(Data);
    memcpy(&TxMessage.Data[0], &Data, sizeof(Data));
    CAN2_Send(&TxMessage);
}

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
    Cradle_ArriveNotify(TZx);
}
