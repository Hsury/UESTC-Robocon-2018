#include "CH340.h"

RingBuf_t CH340_RingBuf;

void CH340_Init(uint32_t Baudrate)
{
    // PC10 = USART3_Tx; PC11 = USART3_Rx
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = Baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    
    USART_Cmd(USART3, ENABLE);
    
    USART_ClearFlag(USART3, USART_FLAG_TC);
    
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    CH340_RingBuf = RingBuf_Create(8, 1);
}

void USART3_IRQHandler(void)
{
    uint8_t tmp;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收中断
    {
        tmp = USART_ReceiveData(USART3); //(USART3->DR);
        RingBuf_Overwrite(&CH340_RingBuf, &tmp);
        Checkout(&CH340_RingBuf);
    }
    else
    {
        if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) == SET)
        {
            USART_ClearFlag(USART3, USART_FLAG_ORE);
            USART_ReceiveData(USART3);
        }
        USART_ClearFlag(USART3, USART_IT_RXNE);
    }
}
