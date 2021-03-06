#ifndef __GYRO_ENCODER_H
#define __GYRO_ENCODER_H

#include "Includes.h"

extern bool GESwitch;

extern uint32_t GyroTS;
extern uint32_t EncoderTS;

void GyroEncoder_Clear(void);
void GyroEncoder_Reset(void);
void GyroEncoder_SetPos(void);
void GyroEncoder_SetAng(void);
void GyroEncoder_On(void);
void GyroEncoder_Off(void);

#endif
