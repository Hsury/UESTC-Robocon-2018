#ifndef __MOVE_H
#define __MOVE_H

#include "includes.h"

#define PIN_WHEN_ARRIVE 1

enum ZONE
{
    SZ = 0, 
    TZ1 = 1, 
    TZ2 = 2, 
    TZ3 = 3, 
    MRA = 4
};

enum PATH
{
    LOCKPOINT = 0, 
    SZ_TZ1 = 1, 
    TZ1_TZ2 = 2, 
    TZ2_TZ3 = 3, 
    SZ_TZ2 = 4, 
    SZ_TZ3 = 5, 
    TZ1_SZ = 6, 
    TZ2_SZ = 7, 
    TZ3_SZ = 8
};

typedef struct
{
    float P0[3];
    float P1[3];
    float P2[3];
    float P3[3];
    float Kp[3];
    float Ki[3];
    float Kd[3];
    float Limit[3];
    uint32_t Duration;
}
PathParam_t;

extern PathParam_t PathParam_LockPoint;
extern PathParam_t PathParam_SZ_TZ1;
extern PathParam_t PathParam_TZ1_TZ2;
extern PathParam_t PathParam_TZ2_TZ3;
extern PathParam_t PathParam_SZ_TZ2;
extern PathParam_t PathParam_SZ_TZ3;
extern PathParam_t PathParam_TZ1_SZ;
extern PathParam_t PathParam_TZ2_SZ;
extern PathParam_t PathParam_TZ3_SZ;

extern PID_t PIDX;
extern PID_t PIDY;
extern PID_t PIDZ;

extern float PIDOutX;
extern float PIDOutY;
extern float PIDOutZ;

void Move_Init(void);
bool Move_UpdateZone(void);
bool Move_Relocate(void);
void Move_SetGoal(PathParam_t* PathParamStruct, uint32_t TimePassed);
void Move_AddConstantFiducial(float ResVel);
void Move_AddDerivativeFiducial(PathParam_t* PathParamStruct, uint32_t TimePassed, float MaxCoef, float TP1, float TP2);
void Move_DeadzoneCtrl(float XY, float Z);
void Move_PID_Start(void);
void Move_PID_Stop(void);
void Move_PID_Compute(void);
void Move_PID_Apply(void);
void Move_PID_SetTunings(PathParam_t* PathParamStruct);
void Move_PID_SetLimits(PathParam_t* PathParamStruct);
float CubicBezier(float P0, float P1, float P2, float P3, uint32_t T, uint32_t Duration);
float CubicBezierDt(float P0, float P1, float P2, float P3, uint32_t T, uint32_t Duration);

#endif
