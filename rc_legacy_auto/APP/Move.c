#include "Move.h"

uint32_t UniformPlusP(float GoalX, float GoalY, float GoalZ, uint32_t BlockTime)
{
    uint32_t StartTS = millis();
    float DistX, DistY, DistZ, DistRes;
    do
    {
        DistX = GoalX - PosX;
        DistY = GoalY - PosY;
        DistZ = DeltaAng(GoalZ - AngZ);
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
