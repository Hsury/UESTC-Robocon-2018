#include "Cylinder.h"

void Cylinder_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    Unpin();
}

void Cylinder_On(uint8_t ID)
{
    switch(ID)
    {
        case 1: PBout(1) = 0; break;
        case 2: PEout(8) = 0; break;
        case 3: PEout(10) = 0; break;
        case 4: PEout(14) = 0; break;
        case 5: PEout(15) = 0; break;
    }
}

void Cylinder_Off(uint8_t ID)
{
    switch(ID)
    {
        case 1: PBout(1) = 1; break;
        case 2: PEout(8) = 1; break;
        case 3: PEout(10) = 1; break;
        case 4: PEout(14) = 1; break;
        case 5: PEout(15) = 1; break;
    }
}

void Pin()
{
    Cylinder_On(5); // 左气缸
    Cylinder_On(3); // 右气缸
    
    Cylinder_Off(4); // 左排气
    Cylinder_Off(2); // 右排气
    
    Cylinder_Off(1); // 气缸
}

void Unpin()
{
    Cylinder_Off(5); // 左气缸
    Cylinder_Off(3); // 右气缸
    
    Cylinder_On(4); // 左排气
    Cylinder_On(2); // 右排气
    
    Cylinder_On(1); // 气缸
}
