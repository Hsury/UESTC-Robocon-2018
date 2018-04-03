#include "GY53.h"

bool GY53A_Capturing = false;
bool GY53B_Capturing = false;

void GY53_PWM_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM3_TimeBaseStructure;
    TIM_TimeBaseInitTypeDef TIM4_TimeBaseStructure;
    TIM_ICInitTypeDef TIM3_ICInitStructure;
    TIM_ICInitTypeDef TIM4_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);              //TIM3，TIM4时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);            //使能PORTC，PORTD时钟
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;                                               //GPIO C6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                            //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                                      //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                          //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                                          //下拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);                                                  //初始化PC6
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);                                 //PC6复用位定时器3
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;                                              //GPIO D12
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                            //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                                      //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                          //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;                                          //下拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);                                                  //初始化PD12
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);                                //PD12复用位定时器4
    
    TIM3_TimeBaseStructure.TIM_Prescaler = 84 - 1;                                          //定时器分频
    TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;                            //向上计数模式
    TIM3_TimeBaseStructure.TIM_Period = 0xFFFF;                                             //自动重装载值，通用计数器只有16位
    TIM3_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM3_TimeBaseStructure);
    
    TIM4_TimeBaseStructure.TIM_Prescaler = 84 - 1;                                          //定时器分频
    TIM4_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;                            //向上计数模式
    TIM4_TimeBaseStructure.TIM_Period = 0xFFFF;                                             //自动重装载值，通用计数器只有16位
    TIM4_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM4_TimeBaseStructure);
    
    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1;                                       //CC1S=01，选择输入端IC1映射到TI1上
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;                            //上升沿捕获
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;                        //映射到TI1上
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;                                  //配置输入分频,不分频
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;                                               //IC1F=0000配置输入滤波器，不滤波
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);                                                //初始化TIM3输入捕获参数
    
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;                                       //CC1S=01，选择输入端IC1映射到TI1上
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;                            //上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;                        //映射到TI1上
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;                                  //配置输入分频,不分频
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;                                               //IC1F=0000配置输入滤波器，不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);                                                //初始化TIM4输入捕获参数
    
    TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_CC1, ENABLE);                                 //允许更新中断，允许CC1IE捕获中断
    TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC1, ENABLE);                                 //允许更新中断，允许CC1IE捕获中断
    
    TIM_Cmd(TIM3, ENABLE);                                                                  //使能定时器3
    TIM_Cmd(TIM4, ENABLE);                                                                  //使能定时器4
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;                               //抢占优先级5
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                                      //子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                                         //根据指定的参数初始化NVIC寄存器
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;                               //抢占优先级5
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                                      //子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                                         //根据指定的参数初始化NVIC寄存器
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        if (GY53A_Capturing)
        {
            uint16_t Tmp = TIM_GetCapture1(TIM3) / 10;
            if (Tmp >= 30 && Tmp <= 1200) GY53A = Tmp / 1000.0f + GY53_A_STATIC_DIST;
            else GY53A = 0;
            CanTxMsg TxMessage;
            TxMessage.StdId = GY53_A_CAN_ID;
            TxMessage.IDE = CAN_ID_STD;
            TxMessage.RTR = CAN_RTR_DATA;
            TxMessage.DLC = 4;
            memcpy(&TxMessage.Data[0], &GY53A, 4);
            CAN2_Send(&TxMessage);
            TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising);         //CC1P=0，设置为上升沿捕获
            Online |= (1 << 6);
        }
        else
        {
            TIM_Cmd(TIM3, DISABLE);                                     //关闭定时器3
            TIM_SetCounter(TIM3, 0);
            TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling);        //CC1P=1，设置为下降沿捕获
            TIM_Cmd(TIM3, ENABLE);                                      //使能定时器3
        }
        GY53A_Capturing = !GY53A_Capturing;
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1 | TIM_IT_Update);            //清除中断标志位
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
    {
        if (GY53B_Capturing)
        {
            uint16_t Tmp = TIM_GetCapture1(TIM4) / 10;
            if (Tmp >= 30 && Tmp <= 1200) GY53B = Tmp / 1000.0f + GY53_B_STATIC_DIST;
            else GY53B = 0;
            CanTxMsg TxMessage;
            TxMessage.StdId = GY53_B_CAN_ID;
            TxMessage.IDE = CAN_ID_STD;
            TxMessage.RTR = CAN_RTR_DATA;
            TxMessage.DLC = 4;
            memcpy(&TxMessage.Data[0], &GY53B, 4);
            CAN2_Send(&TxMessage);
            TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising);         //CC1P=0，设置为上升沿捕获
            Online |= (1 << 7);
        }
        else
        {
            TIM_Cmd(TIM4, DISABLE);                                     //关闭定时器4
            TIM_SetCounter(TIM4, 0);
            TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);        //CC1P=1，设置为下降沿捕获
            TIM_Cmd(TIM4, ENABLE);                                      //使能定时器4
        }
        GY53B_Capturing = !GY53B_Capturing;
    }
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1 | TIM_IT_Update);            //清除中断标志位
}
