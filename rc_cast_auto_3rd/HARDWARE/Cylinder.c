#include "Cylinder.h"

void Cylinder_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    // 1 气缸, 2 真空泵
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    for (uint8_t i = 1; i <= 2; i++) Cylinder_Off(i); 
}

void Cylinder_On(uint8_t ID)
{
    switch(ID)
    {
        case 1: PEout(8) = 0; break;
        case 2: PEout(12) = 0; break;
    }
}

void Cylinder_Off(uint8_t ID)
{
    switch(ID)
    {
        case 1: PEout(8) = 1; break;
        case 2: PEout(12) = 1; break;
    }
}

void Pin()
{
    Cylinder_On(1);
    Cylinder_On(2);
}

void Unpin()
{
    Cylinder_Off(1);
    Cylinder_Off(2);
}
