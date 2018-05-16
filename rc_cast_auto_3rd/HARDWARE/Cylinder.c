#include "Cylinder.h"

void Cylinder_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    for (uint8_t i = 1; i <= 8; i++) Cylinder_Off(i);
}

void Cylinder_On(uint8_t ID)
{
    switch(ID)
    {
        case 1: PBout(1) = 0; break;
        case 2: PEout(8) = 0; break;
        case 3: PEout(10) = 0; break;
        case 4: PEout(12) = 0; break;
        case 5: PEout(14) = 0; break;
        case 6: PBout(10) = 0; break;
        case 7: PBout(14) = 0; break;
        case 8: PDout(8) = 0; break;
    }
}

void Cylinder_Off(uint8_t ID)
{
    switch(ID)
    {
        case 1: PBout(1) = 1; break;
        case 2: PEout(8) = 1; break;
        case 3: PEout(10) = 1; break;
        case 4: PEout(12) = 1; break;
        case 5: PEout(14) = 1; break;
        case 6: PBout(10) = 1; break;
        case 7: PBout(14) = 1; break;
        case 8: PDout(8) = 1; break;
    }
}

void Pin()
{
    Cylinder_On(5); // 左气缸
    Cylinder_On(3); // 右气缸
    
    Cylinder_Off(7); // 左排气
    Cylinder_Off(1); // 右排气
    
    Cylinder_On(8); // 气缸
}

void Unpin()
{
    Cylinder_Off(5); // 左气缸
    Cylinder_Off(3); // 右气缸
    
    Cylinder_On(7); // 左排气
    Cylinder_On(1); // 右排气
    
    Cylinder_Off(8); // 气缸
}
