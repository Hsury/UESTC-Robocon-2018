#ifndef __MOVE_H
#define __MOVE_H

#include "includes.h"

#define MAX_SPEED    (2.4f)
#define SAFE_DIST    (0.8f)
#define COEF_LINEAR  (2.5f)
#define COEF_ANGULAR (2.5f)

extern PID_t PIDX;
extern PID_t PIDY;
extern PID_t PIDZ;

extern float PIDOutX;
extern float PIDOutY;
extern float PIDOutZ;

void Move_PID_Init(void);
void Move_PID_Start(void);
void Move_PID_Stop(void);
void Move_PID_Compute(void);
void Move_PID_Apply(void);
void Move_PID_SetTunings(float KpX, float KiX, float KdX, 
                         float KpY, float KiY, float KdY, 
                         float KpZ, float KiZ, float KdZ);
void Move_PID_SetLimits(float MaxX, float MaxY, float MaxZ);
float CubicBezier(float P0, float P1, float P2, float P3, float T);

#endif
