#include "Sensor.h"

void Sensor_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //使能SYSCFG时钟
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1); //Key 1
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2); //Key 2
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3); //Key 3
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5); //Key 4
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI1_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        if (!PEin(1)) //Key 1
        {
            //Retry = true;
            //Cradle_RestartNotify(TZ3);
            BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
            //xTaskNotifyFromISR(FlowTask_Handler, LAUNCH, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            xTaskNotifyFromISR(FlowTask_Handler, MOVE_ON, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void EXTI2_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        if (!PEin(2)) //Key 2
        {
            BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(FlowTask_Handler, GET_READY, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
        }
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

void EXTI3_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        if (!PEin(3)) //Key 3
        {
            BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(FlowTask_Handler, LAUNCH, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
        }
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

void EXTI9_5_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        if (!PEin(5)) //Key 4
        {
            /*
            Retry = true;
            Cradle_RestartNotify(TZ2);
            BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(FlowTask_Handler, LAUNCH, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
            */
        }
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}
