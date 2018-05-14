#ifndef __SLING_H
#define __SLING_H

#include "Includes.h"

#define ARM_LOADING_POS   1925

#define SLIDER_TZ1_POS    76000 //60000 //73000
#define SLIDER_TZ2_POS    93000 //91000 //88000
#define SLIDER_TZ3_POS    77000 //75000 //81000
#define SLIDER_LOAD_POS   40000 //45000

enum ARM_JOB
{
    IDLE, 
    ADJUST, 
    FIRE
};

typedef struct 
{
    float StartPoint;
    float EndPoint;
    int32_t Speed;
}
ArmParam_t;

extern ArmParam_t ArmParam_TZ1;
extern ArmParam_t ArmParam_TZ2;
extern ArmParam_t ArmParam_TZ3;

extern int32_t SliderPos;
extern int32_t SliderGoal;

extern int32_t RotorPos;
extern int32_t RotorGoal;

extern float ArmPos;
extern float ArmEndPoint;
extern uint8_t ArmJob;

void Sling_Init(void);
void Sling_Release(void);
void Sling_SliderShift(int32_t Position, uint32_t Speed, float Precision, uint32_t Timeout);
void Sling_RotorSweep(float Propotion, uint32_t Speed, float Precision, uint32_t Timeout);
void Sling_ArmAdjust(void);
void Sling_Fire(ArmParam_t *ArmParamStruct, bool Block);
void Sling_QueryPos(void);

#endif
