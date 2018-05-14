#include "GyroEncoder.h"

bool GESwitch = true;

uint32_t GyroTS = 0;
uint32_t EncoderTS = 0;

bool GEError = false;
uint32_t GEErrorTS = 0;
uint32_t GEErrorDuration = 0;

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
    PosX = 0;
    PosY = 0;
    PosZ = 0;
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
    PosX = 0;
    PosY = 0;
    PosZ = 0;
}

void GyroEncoder_SetPos()
{
    CanTxMsg TxMessage;
    uint32_t tmp;
    TxMessage.StdId = GE_CTRL_CAN_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    tmp = Real2EncoderX;
    memcpy(&TxMessage.Data[0], &tmp, 4);
    tmp = Real2EncoderY;
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
    tmp = Real2GyroZ;
    memcpy(&TxMessage.Data[0], &tmp, 4);
    CAN2_Send(&TxMessage);
}

void GyroEncoder_On()
{
    GESwitch = true;
}

void GyroEncoder_Off()
{
    GESwitch = false;
}

void GyroEncoder_SetFlag(bool isOK)
{
    if (GEError && isOK)
    {
        GEError = false;
        GEErrorDuration = fmin(GyroTS, EncoderTS) - GEErrorTS;
    }
    else if (!GEError && !isOK)
    {
        GEError = true;
        GEErrorTS = fmin(GyroTS, EncoderTS);
    }
}

bool GyroEncoder_ReadFlag()
{
    if (millis() - fmin(GyroTS, EncoderTS) <= GE_MAX_HEARTBEAT_DELAY) GyroEncoder_SetFlag(true);
    else GyroEncoder_SetFlag(false);
    //if (GEError) GEErrorDuration = millis() - GEErrorTS;
    return !GEError;
}
