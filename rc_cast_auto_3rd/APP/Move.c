#include "Move.h"

//                                        X     Y     Z
PathParam_t PathParam_LockPoint = {.Kp = {4.75, 4.75, 6.00}, 
                                   .Ki = {0.02, 0.02, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {1.50, 1.25, 60}};

PathParam_t PathParam_SZ_TZ1 =    {.P0 = {0.55, 7.54, -90.00}, 
                                   .P1 = {1.20, 3.75, -60.00}, 
                                   .P2 = {1.70, 3.00, -20.00}, 
                                   .P3 = {3.05, 3.00, -14.50}, 
                                   .Kp = {2.75, 2.50, 6.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {4.75, 4.75, 90}, 
                                   .Duration = 2150, // 2400 2150
                                   .Timeout = 3200}; // Total: 3070ms 3200

PathParam_t PathParam_TZ1_TZ2 =   {.P0 = {3.05, 3.00, -14.50}, 
                                   .P1 = {0.80, 2.90, -14.50}, 
                                   .P2 = {0.80, 1.10, -9.00}, 
                                   .P3 = {3.05, 1.00, -9.00}, 
                                   .Kp = {2.75, 2.25, 5.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {4.75, 4.75, 90}, 
                                   .Duration = 2000, // 1800 1450
                                   .Timeout = 3500}; // Total: 3035ms 3500

PathParam_t PathParam_TZ2_TZ3 =   {.P0 = {3.05, 1.00, -9.00}, 
                                   .P1 = {3.35, 1.00, -8.00}, 
                                   .P2 = {7.20, 1.00, -1.50}, 
                                   .P3 = {7.30, 1.00, -0.50}, 
                                   .Kp = {2.30, 2.50, 6.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {4.75, 4.75, 90}, 
                                   .Duration = 1800, // 1550 2000
                                   .Timeout = 3000};// Total: 2705ms 3000

PathParam_t PathParam_SZ_TZ2 =    {.P0 = {0.55, 7.54, -90.00}, 
                                   .P1 = {1.20, 2.00, -90.00}, 
                                   .P2 = {1.60, 1.00, -10.00}, 
                                   .P3 = {3.05, 1.00, -10.00}, 
                                   .Kp = {2.25, 2.50, 6.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {4.75, 4.75, 90}, 
                                   .Duration = 2900, //4500 //5000
                                   .Timeout = 4000};// Total: 2705ms

PathParam_t PathParam_SZ_TZ3 =    {.P0 = {0.60, 7.54, -90.00}, 
                                   .P1 = {0.80, 0.60, -90.00}, 
                                   .P2 = {1.00, 1.00, -0.50}, 
                                   .P3 = {7.30, 1.00, -0.50}, 
                                   .Kp = {2.25, 2.50, 6.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {4.75, 4.75, 90}, 
                                   .Duration = 5000, //4500 //5000
                                   .Timeout = 8000};// Total: 2705ms

PathParam_t PathParam_TZ3_SZ =    {.P0 = {7.30, 1.00, -0.50}, 
                                   .P1 = {0.00, 1.00, -0.50}, 
                                   .P2 = {0.60, 0.00, -90.00}, 
                                   .P3 = {0.60, 7.54, -90.00}, 
                                   .Kp = {2.00, 2.00, 5.50}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {2.00, 2.00, 60}, 
                                   .Duration = 12000, 
                                   .Timeout = 15000};

PathParam_t PathParam_Dash =      {.Kp = {75.00, 50.00, 6.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {2.50, 2.00, 60}};

PID_t PIDX;
PID_t PIDY;
PID_t PIDZ;

float PIDOutX = 0;
float PIDOutY = 0;
float PIDOutZ = 0;

void Move_Init()
{
    PIDX = PID(&PosX, &PIDOutX, &GoalX, 2.5, 0, 0, PID_P_ON_E, PID_DIRECT);
    PIDY = PID(&PosY, &PIDOutY, &GoalY, 2.5, 0, 0, PID_P_ON_E, PID_DIRECT);
    PIDZ = PID(&PosZ, &PIDOutZ, &GoalZ, 5.0, 0, 0, PID_P_ON_E, PID_DIRECT);
    PID_SetOutputLimits(&PIDX, -4.25, 4.25);
    PID_SetOutputLimits(&PIDY, -4.25, 4.25);
    PID_SetOutputLimits(&PIDZ, -90, 90);
    PID_SetSampleTime(&PIDX, 5);
    PID_SetSampleTime(&PIDY, 5);
    PID_SetSampleTime(&PIDZ, 5);
    PID_SetMode(&PIDX, PID_AUTOMATIC);
    PID_SetMode(&PIDY, PID_AUTOMATIC);
    PID_SetMode(&PIDZ, PID_AUTOMATIC);
    Elmo_Reinit(0);
}

bool Move_UpdateZone()
{
    if (PosX >= 2.4f && PosX <= 6.0f && PosY >= 2.5f && PosY <= 3.5f)
    {
        if (Zone != TZ1)
        {
            Zone = TZ1;
            return true;
        }
    }
    else if (PosX >= 2.4f && PosX <= 6.0f && PosY >= 0.5f && PosY <= 1.5f)
    {
        if (Zone != TZ2)
        {
            Zone = TZ2;
            return true;
        }
    }
    else if (PosX >= 6.4f && PosX <= 10.0f && PosY >= 0.5f && PosY <= 1.5f)
    {
        if (Zone != TZ3)
        {
            Zone = TZ3;
            return true;
        }
    }
    else
    {
        if (Zone != MRA)
        {
            Zone = MRA;
            return true;
        }
    }
    return false;
}

bool Move_Relocate()
{
    #if USE_DT35
    float DT352RealXReg = DT352RealX;
    Printf(ESP8266, "X-Axis Relocation Attempt: %.3f -> %.3f ---- ", PosX, DT352RealXReg);
    Probe_SetUser(PosX - DT352RealXReg);
    if (fabs(PosX - DT352RealXReg) < 0.15)
    {
        GyroEncoder_Off();
        delay_ms(5);
        PosX = DT352RealXReg;
        GyroEncoder_SetPos();
        delay_ms(5);
        GyroEncoder_On();
        Printf(ESP8266, "PASS\r\n");
        return true;
    }
    Printf(ESP8266, "FAIL\r\n");
    return false;
    #else
    return true;
    #endif
}

void Move_SetGoal(PathParam_t* PathParamStruct, uint32_t TimePassed)
{
    GoalX = CubicBezier(PathParamStruct->P0[0], 
                        PathParamStruct->P1[0], 
                        PathParamStruct->P2[0], 
                        PathParamStruct->P3[0], 
                        TimePassed, 
                        PathParamStruct->Duration);
    GoalY = CubicBezier(PathParamStruct->P0[1], 
                        PathParamStruct->P1[1], 
                        PathParamStruct->P2[1], 
                        PathParamStruct->P3[1], 
                        TimePassed, 
                        PathParamStruct->Duration);
    GoalZ = CubicBezier(PathParamStruct->P0[2], 
                        PathParamStruct->P1[2], 
                        PathParamStruct->P2[2], 
                        PathParamStruct->P3[2], 
                        TimePassed, 
                        PathParamStruct->Duration);
}

void Move_AddConstantFiducial(float ResVel)
{
    float ResDist = DeltaPos(GoalX - PosX, GoalY - PosY);
    VelX += (GoalX - PosX) / ResDist * ResVel;
    VelY += (GoalY - PosY) / ResDist * ResVel;
}

void Move_AddDerivativeFiducial(PathParam_t* PathParamStruct, uint32_t TimePassed, float MaxCoef, float TP1, float TP2)
{
    if (!(0 <= MaxCoef && MaxCoef <= 1 && 0 < TP1 && TP1 < TP2 && TP2 < 1)) return;
    float Fraction = 1.0f * clamp(TimePassed, 0, PathParamStruct->Duration) / PathParamStruct->Duration;
    float Coef;
    if (Fraction < TP1) Coef = Fraction * MaxCoef / TP1;
    else if (Fraction < TP2) Coef = MaxCoef;
    else Coef = (1 - Fraction) * MaxCoef / (1 - TP2);
    VelX += Coef * CubicBezierDt(PathParamStruct->P0[0], 
                                 PathParamStruct->P1[0], 
                                 PathParamStruct->P2[0], 
                                 PathParamStruct->P3[0], 
                                 TimePassed, 
                                 PathParamStruct->Duration);
    VelY += Coef * CubicBezierDt(PathParamStruct->P0[1], 
                                 PathParamStruct->P1[1], 
                                 PathParamStruct->P2[1], 
                                 PathParamStruct->P3[1], 
                                 TimePassed, 
                                 PathParamStruct->Duration);
    //Printf(ESP8266, "T=%u, X=%f, Y=%f\r\n", TimePassed, VelX, VelY);
}

void Move_DeadzoneCtrl(float XY, float Z)
{
    if (DeltaPos(GoalX - PosX, GoalY - PosY) < XY)
    {
        VelX = 0;
        VelY = 0;
    }
    if (fabs(GoalZ - PosZ) < Z) VelZ = 0;
}

void Move_EnterDash(float IncX, float IncY)
{
    Move_UpdateZone();
    switch (Zone)
    {
        case TZ1:
        GoalX = clamp(PosX + IncX, PathParam_SZ_TZ1.P3[0] - 0.20f, PathParam_SZ_TZ1.P3[0] + 0.20f);
        GoalY = clamp(PosY + IncY, PathParam_SZ_TZ1.P3[1] - 0.15f, PathParam_SZ_TZ1.P3[1] + 0.10f);
        break;
        
        case TZ2:
        GoalX = clamp(PosX + IncX, PathParam_TZ1_TZ2.P3[0] - 0.20f, PathParam_TZ1_TZ2.P3[0] + 0.20f);
        GoalY = clamp(PosY + IncY, PathParam_TZ1_TZ2.P3[1] - 0.15f, PathParam_TZ1_TZ2.P3[1] + 0.10f);
        break;
        
        case TZ3:
        GoalX = clamp(PosX + IncX, PathParam_TZ2_TZ3.P3[0] - 0.20f, PathParam_TZ2_TZ3.P3[0] + 0.20f);
        GoalY = clamp(PosY + IncY, PathParam_TZ2_TZ3.P3[1] - 0.15f, PathParam_TZ2_TZ3.P3[1] + 0.10f);
        break;
    }
}

void Move_ExitDash()
{
    Move_UpdateZone();
    switch (Zone)
    {
        case TZ1:
        GoalX = PathParam_SZ_TZ1.P3[0];
        GoalY = PathParam_SZ_TZ1.P3[1];
        break;
        
        case TZ2:
        GoalX = PathParam_TZ1_TZ2.P3[0];
        GoalY = PathParam_TZ1_TZ2.P3[1];
        break;
        
        case TZ3:
        GoalX = PathParam_TZ2_TZ3.P3[0];
        GoalY = PathParam_TZ2_TZ3.P3[1];
        break;
    }
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

void Move_PID_SetTunings(PathParam_t* PathParamStruct)
{
    PID_SetTunings(&PIDX, PathParamStruct->Kp[0], PathParamStruct->Ki[0], PathParamStruct->Kd[0], PID_P_ON_E);
    PID_SetTunings(&PIDY, PathParamStruct->Kp[1], PathParamStruct->Ki[1], PathParamStruct->Kd[1], PID_P_ON_E);
    PID_SetTunings(&PIDZ, PathParamStruct->Kp[2], PathParamStruct->Ki[2], PathParamStruct->Kd[2], PID_P_ON_E);
}

void Move_PID_SetLimits(PathParam_t* PathParamStruct)
{
    PID_SetOutputLimits(&PIDX, -PathParamStruct->Limit[0], PathParamStruct->Limit[0]);
    PID_SetOutputLimits(&PIDY, -PathParamStruct->Limit[1], PathParamStruct->Limit[1]);
    PID_SetOutputLimits(&PIDZ, -PathParamStruct->Limit[2], PathParamStruct->Limit[2]);
}

float CubicBezier(float P0, float P1, float P2, float P3, uint32_t T, uint32_t Duration)
{
    float Fraction = 1.0f * clamp(T, 0, Duration) / Duration;
    return pow(1 - Fraction, 3) * P0 + 
           3 * pow(1 - Fraction, 2) * Fraction * P1 + 
           3 * (1 - Fraction) * pow(Fraction, 2) * P2 + 
           pow(Fraction, 3) * P3;
}

float CubicBezierDt(float P0, float P1, float P2, float P3, uint32_t T, uint32_t Duration)
{
    float Fraction = 1.0f * clamp(T, 0, Duration) / Duration;
    return (3 * pow(1 - Fraction, 2) * (P1 - P0) + 
            6 * (1 - Fraction) * Fraction * (P2 - P1) + 
            3 * pow(Fraction, 2) * (P3 - P2)) / (Duration / 1000.0f);
}
