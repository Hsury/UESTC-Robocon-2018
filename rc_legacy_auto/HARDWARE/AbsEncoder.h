#ifndef __ABS_ENCODER_H
#define __ABS_ENCODER_H

#include "Includes.h"

void AbsEncoder_SSI_Init(void);
void AbsEncoder_CAN_Query(uint8_t NodeID);
void AbsEncoder_CAN_SetID(uint8_t NodeID, uint8_t NewNodeID);
void AbsEncoder_CAN_SetMode(uint8_t NodeID, bool AutoSend);
void AbsEncoder_CAN_SetInterval(uint8_t NodeID, uint16_t Interval);
void AbsEncoder_CAN_SetZero(uint8_t NodeID);

#endif
