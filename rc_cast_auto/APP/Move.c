#include "Move.h"

//                                        X     Y     Z
PathParam_t PathParam_SZ_TZ1 =    {.P0 = {0.55, 7.54, 0.00}, 
                                   .P1 = {0.80, 4.00, 0.00}, 
                                   .P2 = {0.80, 3.32, 0.00}, 
                                   .P3 = {2.80, 3.37, 0.00}, 
                                   .Kp = {2.25, 2.50, 5.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {4.25, 4.25, 90}, 
                                   .Duration = 2200};

PathParam_t PathParam_TZ1_TZ2 =   {.P0 = {2.80, 3.37, 0.00}, 
                                   .P1 = {1.10, 2.87, 1.00}, 
                                   .P2 = {1.10, 1.47, 2.00}, 
                                   .P3 = {2.80, 1.37, 3.00}, 
                                   .Kp = {2.50, 2.50, 5.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {4.25, 4.25, 90}, 
                                   .Duration = 1750};

PathParam_t PathParam_TZ2_TZ3 =   {.P0 = {2.80, 1.37, 3.00}, 
                                   .P1 = {4.80, 1.37, 5.00}, 
                                   .P2 = {6.99, 1.37, 9.00}, 
                                   .P3 = {7.00, 1.37, 13.00}, 
                                   .Kp = {2.25, 2.50, 5.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {4.25, 4.25, 90}, 
                                   .Duration = 2000};

PathParam_t PathParam_SZ_TZ2 =    {.P0 = {0.55, 7.54, 0.00}, 
                                   .P1 = {1.25, 2.00, 1.00}, 
                                   .P2 = {1.25, 1.57, 2.00}, 
                                   .P3 = {2.80, 1.37, 3.00}, 
                                   .Kp = {2.50, 2.50, 5.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}, 
                                   .Limit = {4.25, 4.25, 90}, 
                                   .Duration = 3000};

PathParam_t PathParam_LockPoint = {.Kp = {4.00, 4.00, 5.00}, 
                                   .Ki = {0.00, 0.00, 0.00}, 
                                   .Kd = {0.00, 0.00, 0.00}};

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
    Elmo_Reinit(1);
    Elmo_Reinit(2);
    Elmo_Reinit(3);
    delay_ms(50);
}

bool Move_UpdateZone()
{
    if (PosX >= 2.5f && PosX <= 4.0f && PosY >= 2.75f && PosY <= 3.5f)
    {
        if (Zone != TZ1)
        {
            Zone = TZ1;
            return true;
        }
    }
    else if (PosX >= 2.5f && PosX <= 4.0f && PosY >= 0.75f && PosY <= 1.5f)
    {
        if (Zone != TZ2)
        {
            Zone = TZ2;
            return true;
        }
    }
    else if (PosX >= 6.5f && PosX <= 8.0f && PosY >= 0.75f && PosY <= 1.5f)
    {
        if (Zone != TZ3)
        {
            Zone = TZ3;
            return true;
        }
    }
    else
    {
        if (Zone != CORRIDOR)
        {
            Zone = CORRIDOR;
            return true;
        }
    }
    return false;
}

/*
bool Move_Relocate()
{
    Move_UpdateZone();
    if ((Zone == TZ1 || Zone == TZ2 || Zone == TZ3) &&  //只在TZ1，TZ2，TZ3内进行重定位
        (fabs(PosZ) < 15) &&                            //偏角在15度以内
        (fabs(VelX) < 0.20) &&                          //X轴速度小于20cm/s
        (fabs(VelY) < 0.05) &&                          //Y轴速度小于5cm/s
        (fabs(VelZ) < 5) &&                             //Z轴速度小于5度/s
        (fabs(PosZ - GY532RealZ) < 15))                 //GY53算出的角度和陀螺仪反馈的角度差值小于15度
    {
        if (Zone == TZ1 && fabs(DT502RealX - PosX) <= 0.2 && fabs(3.37 + 0.60 - GY532RealY - PosY) <= 0.2)
        {
            GyroEncoder_Off();
            PosX = DT502RealX;
            PosY = 3.37 + 0.60 - GY532RealY;
            GyroEncoder_SetPos();
            GyroEncoder_On();
            return true;
        }
        else if (Zone == TZ2 && fabs(DT502RealX - PosX) <= 0.2 && fabs(1.37 + 0.60 - GY532RealY - PosY) <= 0.2)
        {
            GyroEncoder_Off();
            PosX = DT502RealX;
            PosY = 1.37 + 0.60 - GY532RealY;
            GyroEncoder_SetPos();
            GyroEncoder_On();
            return true;
        }
        else if (Zone == TZ3 && fabs(DT502RealX - PosX) <= 0.2 && fabs(1.37 + 0.60 - GY532RealY - PosY) <= 0.2)
        {
            GyroEncoder_Off();
            PosX = DT502RealX;
            PosY = 1.37 + 0.60 - GY532RealY;
            GyroEncoder_SetPos();
            GyroEncoder_On();
            return true;
        }
    }
    return false;
}
*/

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

/*
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
*/
