#include "Sensor.h"

SemaphoreHandle_t SwitchSemaphore;
SemaphoreHandle_t DimmerSemaphore;

void Sensor_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //使能SYSCFG时钟
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource14); //Sensor 1
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource8);  //Sensor 2
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource15); //Sensor 3
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11); //Sensor 4
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource9);  //Sensor 5
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10); //Sensor 6
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line8 | EXTI_Line9 | EXTI_Line10 | EXTI_Line11 | EXTI_Line14 | EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	SwitchSemaphore = xSemaphoreCreateBinary();
	DimmerSemaphore = xSemaphoreCreateBinary();
}

void EXTI9_5_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line8) != RESET)
    {
        if (!PDin(8))
        {
            //Sensor 2
            //触控开关
            //BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
            //xSemaphoreGiveFromISR(SwitchSemaphore, &pxHigherPriorityTaskWoken);
            //if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
        }
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
    else if (EXTI_GetITStatus(EXTI_Line9) != RESET)
    {
        if (!PDin(9))
        {
            //Sensor 5
        }
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}

void EXTI15_10_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line10) != RESET)
    {
        if (!PDin(10))
        {
            //Sensor 6
        }
        EXTI_ClearITPendingBit(EXTI_Line10);
    }
    else if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        if (PDin(11))
        {
            //Sensor 4
            //光电门
            BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(DimmerSemaphore, &pxHigherPriorityTaskWoken);
            if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
        }
        EXTI_ClearITPendingBit(EXTI_Line11);
    }
    else if (EXTI_GetITStatus(EXTI_Line14) != RESET)
    {
        if (!PBin(14))
        {
            //Sensor 1
            //Y轴色标传感器
            if (PosX >= 1.50f && PosX <= 4.50f && PosY >= 3.17f && PosY <= 3.47f)
            {
                printf("TZ1 Y-Axis relocated at %.3f, VelY = %.3f\r\n", PosY, RealVelY);
                GyroEncoder_Off();
                PosY = 3.32 + PhotoSensorYOffset + (RealVelY < 0 ? 0 : -0.02);
                GyroEncoder_SetPos();
                GyroEncoder_On();
            }
            else if (PosX >= 1.50f && PosX <= 4.50f && PosY >= 1.17f && PosY <= 1.47f)
            {
                printf("TZ2 Y-Axis relocated at %.3f, VelY = %.3f\r\n", PosY, RealVelY);
                GyroEncoder_Off();
                PosY = 1.32 + PhotoSensorYOffset + (RealVelY < 0 ? 0 : -0.02);
                GyroEncoder_SetPos();
                GyroEncoder_On();
            }
            else if (PosX >= 5.00f && PosX <= 8.00f && PosY >= 1.17f && PosY <= 1.47f)
            {
                printf("TZ3 Y-Axis relocated at %.3f, VelY = %.3f\r\n", PosY, RealVelY);
                GyroEncoder_Off();
                PosY = 1.32 + PhotoSensorYOffset + (RealVelY < 0 ? 0 : -0.02);
                GyroEncoder_SetPos();
                GyroEncoder_On();
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line14);
    }
    else if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
        if (!PBin(15))
        {
            //Sensor 3
            //X轴色标传感器
            if (PosX >= 2.45f && PosX <= 2.75f && PosY >= 2.75f && PosY <= 3.50f)
            {
                printf("TZ1 X-Axis relocated at %.3f, VelX = %.3f\r\n", PosX, RealVelX);
                GyroEncoder_Off();
                PosX = 2.60 + PhotoSensorXOffset + (RealVelX > 0 ? 0 : 0.01);
                GyroEncoder_SetPos();
                GyroEncoder_On();
            }
            else if (PosX >= 2.45f && PosX <= 2.75f && PosY >= 0.75f && PosY <= 1.50f)
            {
                printf("TZ2 X-Axis relocated at %.3f, VelX = %.3f\r\n", PosX, RealVelX);
                GyroEncoder_Off();
                PosX = 2.60 + PhotoSensorXOffset + (RealVelX > 0 ? 0 : 0.01);
                GyroEncoder_SetPos();
                GyroEncoder_On();
            }
            else if (PosX >= 4.45f && PosX <= 4.75f && PosY >= 0.75f && PosY <= 1.50f)
            {
                printf("TZ3 X-Axis relocated at %.3f, VelX = %.3f\r\n", PosX, RealVelX);
                GyroEncoder_Off();
                PosX = 4.60 + PhotoSensorXOffset + (RealVelX > 0 ? 0 : 0.01);
                GyroEncoder_SetPos();
                GyroEncoder_On();
            }
        }
        EXTI_ClearITPendingBit(EXTI_Line15);
    }
}
