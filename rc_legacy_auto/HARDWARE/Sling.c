#include "Sling.h"

//ArmParam_t ArmParam_TZ1 = {.StartPoint = ARM_LOADING_POS, .EndPoint = 3000, .Speed = 58000}; //2900 48000 //46000
ArmParam_t ArmParam_TZ1 = {.StartPoint = ARM_LOADING_POS, .EndPoint = 2980, .Speed = 55000}; //2900 48000 //46000
ArmParam_t ArmParam_TZ2 = {.StartPoint = ARM_LOADING_POS, .EndPoint = 3050, .Speed = 58500}; //2900 53000 //51000 //60000
ArmParam_t ArmParam_TZ3 = {.StartPoint = ARM_LOADING_POS, .EndPoint = 2875, .Speed = 60500}; //62500 //59600

int32_t SliderPos = 0;
int32_t RotorPos = 0;

int32_t RotorRegister = 0;

PID_t PIDArm;
float PIDOutArm;
float GoalArm = ARM_LOADING_POS;

float ArmPos = 0;
float ArmEndPoint = 0;
uint8_t ArmJob = IDLE;

void Sling_Init()
{
    PIDArm = PID(&ArmPos, &PIDOutArm, &GoalArm, 80, 0, 0, PID_P_ON_E, PID_REVERSE);
    PID_SetOutputLimits(&PIDArm, -8000, 8000);
    PID_SetSampleTime(&PIDArm, 5);
    Elmo_Reinit(4);
    Elmo_Reinit(5);
    Elmo_Reinit(6);
    //delay_xms(50);
    Elmo_Set_POS(4, 0);
    Elmo_Set_POS(5, 0);
    //delay_xms(50);
    Elmo_SetAcc(4, 50000000, 50000000);
    //delay_xms(50);
}

void Sling_Release()
{
    Elmo_Close(4);
    Elmo_Close(5);
    Elmo_Close(6);
}

void Sling_SliderShift(int32_t Position, uint32_t Speed, float Precision, uint32_t Timeout)
{
    Elmo_PPM(4, Speed, Position, POS_ABS);
	uint32_t TS = millis();
	do
	{
		Sling_QueryPos();
		delay_ms(10);
		if (1.0f * abs(SliderPos - Position) / 100000 <= Precision) break;
	}
    while (millis() - TS < Timeout);
}

void Sling_RotorSweep(float Propotion, uint32_t Speed, float Precision, uint32_t Timeout)
{
	RotorRegister += REDUCTION_RATIO * 2000 * Propotion;
	Elmo_PPM(5, Speed, RotorRegister, POS_ABS);
	uint32_t TS = millis();
	do
	{
		Sling_QueryPos();
		delay_ms(10);
		if (1.0f * abs(RotorPos - RotorRegister) / (REDUCTION_RATIO * 2000) <= Precision) break;
	}
    while (millis() - TS < Timeout);
}

void Sling_ArmAdjust()
{
    while (ArmJob == FIRE) delay_ms(1); //防止抛射过程被打断
    if (ArmJob == ADJUST) return; //防止悬臂回归线程被创建多个
    ArmJob = ADJUST;
    PID_SetMode(&PIDArm, PID_AUTOMATIC);
    do
    {
        PID_Compute(&PIDArm);
        Elmo_PVM(6, (int)PIDOutArm);
        delay_ms(5);
    }
    while ((int)PIDOutArm != 0);
    Printf(ESP8266, "Arm=%d\r\n", (int)ArmPos);
    Elmo_Stop(6);
    PID_SetMode(&PIDArm, PID_MANUAL);
    GoalArm = ARM_LOADING_POS;
    ArmJob = IDLE;
}

void Sling_Fire(ArmParam_t *ArmParamStruct, bool Block)
{
    while (ArmJob != IDLE) delay_ms(1); //等待悬臂空闲
    //GoalArm = ArmParamStruct->StartPoint;
    //Sling_ArmAdjust();
    ArmJob = FIRE;
    ArmEndPoint = ArmParamStruct->EndPoint;
    Elmo_PVM(6, - ArmParamStruct->Speed);
    if (Block) while (ArmJob != IDLE) delay_ms(1);
}

void Sling_QueryPos()
{
    Elmo_Read_POS(4);
    Elmo_Read_POS(5);
}
