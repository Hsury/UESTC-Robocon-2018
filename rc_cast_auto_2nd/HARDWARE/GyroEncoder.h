#ifndef __GYRO_ENCODER_H
#define __GYRO_ENCODER_H

#include "Includes.h"

#define GE_MAX_HEARTBEAT_DELAY 25

extern bool GESwitch;

extern uint32_t GyroTS;
extern uint32_t EncoderTS;

extern bool GEError;
extern uint32_t GEErrorTS;
extern uint32_t GEErrorDuration;

void GyroEncoder_Clear(void);
void GyroEncoder_Reset(void);
void GyroEncoder_SetPos(void);
void GyroEncoder_SetAng(void);
void GyroEncoder_On(void);
void GyroEncoder_Off(void);
void GyroEncoder_SetFlag(bool isOK);
bool GyroEncoder_ReadFlag(void);

#endif
