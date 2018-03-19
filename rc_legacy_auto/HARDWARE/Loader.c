#include "Loader.h"

int32_t SliderPos = 0;
int32_t TongPos = 0;

void Loader_Init()
{
    Elmo_Reinit(4);
    Elmo_Reinit(5);
    Elmo_Set_POS(4, 0);
    Elmo_Set_POS(5, 0);
    SliderPos = 0;
    TongPos = 0;
}

void Loader_Release()
{
    Elmo_Close(4);
    Elmo_Close(5);
}

void Loader_Shift(uint8_t PosID)
{
    switch (PosID)
    {
        case 1:
        SliderPos = 0;
        break;
        
        case 2:
        SliderPos = -27000;
        break;
    }
    Elmo_PPM(4, 100000, SliderPos, POS_ABS);
}

void Loader_Sweep()
{
    TongPos -= 38100;
    Elmo_PPM(5, 30000, TongPos, POS_ABS);
}

void Loader_QueryPos()
{
    Elmo_Read_POS(4);
    Elmo_Read_POS(5);
    delay_ms(100);
}
