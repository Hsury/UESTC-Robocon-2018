#include "GyroEncoder.h"

void GyroEncoder_Clear()
{
    CanTxMsg TxMessage;
    TxMessage.StdId = GE_CTRL_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0x55;
    TxMessage.Data[1] = 0xEE;
    CAN2_Send(&TxMessage);
}

void GyroEncoder_Reset()
{
    CanTxMsg TxMessage;
    TxMessage.StdId = GE_CTRL_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0x55;
    TxMessage.Data[1] = 0xAA;
    CAN2_Send(&TxMessage);
}

void GyroEncoder_SetPos()
{
    CanTxMsg TxMessage;
    uint32_t tmp;
    TxMessage.StdId = GE_CTRL_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    tmp = Real2Encoder_X;
    memcpy(&TxMessage.Data[0], &tmp, 4);
    tmp = Real2Encoder_Y;
    memcpy(&TxMessage.Data[4], &tmp, 4);
    CAN2_Send(&TxMessage);
}

void GyroEncoder_SetAng()
{
    CanTxMsg TxMessage;
    float tmp;
    TxMessage.StdId = GE_CTRL_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 4;
    tmp = Real2Gyro_Z;
    memcpy(&TxMessage.Data[0], &tmp, 4);
    CAN2_Send(&TxMessage);
}
