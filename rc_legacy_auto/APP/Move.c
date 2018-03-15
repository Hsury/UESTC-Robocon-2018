#include "Move.h"

PID_t PIDX;
PID_t PIDY;
PID_t PIDZ;

float PIDOutX = 0;
float PIDOutY = 0;
float PIDOutZ = 0;

void Move_PID_Init()
{
    PIDX = PID(&PosX, &PIDOutX, &GoalX, 2.5, 0, 0, PID_P_ON_E, PID_DIRECT);
    PIDY = PID(&PosY, &PIDOutY, &GoalY, 2.5, 0, 0, PID_P_ON_E, PID_DIRECT);
    PIDZ = PID(&PosZ, &PIDOutZ, &GoalZ, 5.0, 0, 0, PID_P_ON_E, PID_DIRECT);
    PID_SetOutputLimits(&PIDX, -4, 4);
    PID_SetOutputLimits(&PIDY, -4, 4);
    PID_SetOutputLimits(&PIDZ, -90, 90);
    PID_SetSampleTime(&PIDX, 5);
    PID_SetSampleTime(&PIDY, 5);
    PID_SetSampleTime(&PIDZ, 5);
}

void Move_PID_Start()
{
    PID_SetMode(&PIDX, PID_AUTOMATIC);
    PID_SetMode(&PIDY, PID_AUTOMATIC);
    PID_SetMode(&PIDZ, PID_AUTOMATIC);
}

void Move_PID_Stop()
{
    PID_SetMode(&PIDX, PID_MANUAL);
    PID_SetMode(&PIDY, PID_MANUAL);
    PID_SetMode(&PIDZ, PID_MANUAL);
}

void Move_PID_Compute()
{
    PID_Compute(&PIDX);
    PID_Compute(&PIDY);
    PID_Compute(&PIDZ);
}

void Move_PID_Apply()
{
    VelX = PIDOutX;
    VelY = PIDOutY;
    VelZ = PIDOutZ;
}

void Move_PID_SetTunings(float KpX, float KiX, float KdX, 
                         float KpY, float KiY, float KdY, 
                         float KpZ, float KiZ, float KdZ)
{
    PID_SetTunings(&PIDX, KpX, KiX, KdX, PID_P_ON_E);
    PID_SetTunings(&PIDY, KpY, KiY, KdY, PID_P_ON_E);
    PID_SetTunings(&PIDZ, KpZ, KiZ, KdZ, PID_P_ON_E);
}

void Move_PID_SetLimits(float MaxX, float MaxY, float MaxZ)
{
    if (MaxX > 0) PID_SetOutputLimits(&PIDX, -MaxX, MaxX);
    if (MaxY > 0) PID_SetOutputLimits(&PIDY, -MaxY, MaxY);
    if (MaxZ > 0) PID_SetOutputLimits(&PIDZ, -MaxZ, MaxZ);
}

float CubicBezier(float P0, float P1, float P2, float P3, float T)
{
    T = clamp(T, 0, 1);
    return pow(1 - T, 3) * P0 + 
           3 * pow(1 - T, 2) * T * P1 + 
           3 * (1 - T) * pow(T, 2) * P2 + 
           pow(T, 3) * P3;
}

uint32_t UniformPlusP(float GoalX, float GoalY, float GoalZ, uint32_t BlockTime)
{
    uint32_t StartTS = millis();
    float DistX, DistY, DistZ, DistRes;
    do
    {
        DistX = GoalX - PosX;
        DistY = GoalY - PosY;
        DistZ = DeltaAng(GoalZ - PosZ);
        DistRes = sqrt(DistX * DistX + DistY * DistY);
        if (DistRes > SAFE_DIST)
        {
            VelX = DistX * MAX_SPEED / DistRes;
            VelY = DistY * MAX_SPEED / DistRes;
        }
        else
        {
            VelX = DistX * COEF_LINEAR;
            VelY = DistY * COEF_LINEAR;
        }
        VelZ = DistZ * COEF_ANGULAR;
        Omni_Elmo_PVM();
        delay_ms(5);
    }
    while (abs(DistX) > 0.01 || abs(DistY) > 0.01 || abs(DistZ) > 1 || millis() - StartTS < BlockTime);
    Omni_Elmo_Stop();
    return millis() - StartTS;
}
