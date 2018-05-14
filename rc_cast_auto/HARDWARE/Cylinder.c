#include "Cylinder.h"

void Cylinder_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);
    
    //Cylinder 1 放气电磁阀
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    //Cylinder 2 吸盘气泵A
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    //Cylinder 3 摄像头气缸
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    //Cylinder 4 吸盘气缸
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //Cylinder 5 摄像头左右电磁铁
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //Cylinder 6 吸盘气泵B
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    //Cylinder 7 吸盘气泵C
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    //Cylinder 8 吸盘气泵D
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void Cylinder_On(uint8_t ID)
{
    switch(ID)
    {
        case 1: PCout(5) = 1; break;
        case 2: PEout(8) = 1; break;
        case 3: PEout(12) = 1; break;
        case 4: PBout(10) = 1; break;
        case 5: PBout(1) = 1; break;
        case 6: PEout(10) = 1; break;
        case 7: PEout(14) = 1; break;
        case 8: PEout(15) = 1; break;
    }
}

void Cylinder_Off(uint8_t ID)
{
    switch(ID)
    {
        case 1: PCout(5) = 0; break;
        case 2: PEout(8) = 0; break;
        case 3: PEout(12) = 0; break;
        case 4: PBout(10) = 0; break;
        case 5: PBout(1) = 0; break;
        case 6: PEout(10) = 0; break;
        case 7: PEout(14) = 0; break;
        case 8: PEout(15) = 0; break;
    }
}

void Pin()
{
    Cylinder_On(4);
    delay_ms(50);
    Cylinder_On(2);
    Cylinder_On(6);
    Cylinder_On(7);
    Cylinder_On(8);
    Cylinder_Off(1);
}

void PinFromISR()
{
    Cylinder_On(4);
    delay_xms(50);
    Cylinder_On(2);
    Cylinder_On(6);
    Cylinder_On(7);
    Cylinder_On(8);
    Cylinder_Off(1);
}

void Unpin()
{
    Cylinder_Off(2);
    Cylinder_Off(6);
    Cylinder_Off(7);
    Cylinder_Off(8);
    Cylinder_On(1);
    delay_ms(50);
    Cylinder_Off(4);
}

void UnpinFromISR()
{
    Cylinder_Off(2);
    Cylinder_Off(6);
    Cylinder_Off(7);
    Cylinder_Off(8);
    Cylinder_On(1);
    delay_xms(50);
    Cylinder_Off(4);
}

void DropCamera()
{
    Cylinder_On(3);
    //delay_ms(500);
    //Cylinder_Off(5);
    //Cylinder_Off(3);
}

void TakeCamera()
{
    //Cylinder_On(5);
    //Cylinder_On(3);
    //delay_ms(500);
    Cylinder_Off(3);
}
