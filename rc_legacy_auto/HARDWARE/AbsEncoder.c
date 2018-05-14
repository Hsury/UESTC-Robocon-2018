#include "AbsEncoder.h"

void AbsEncoder_SSI_Init()
{
    //CLK: PC2; CS: PC13
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    TIM_TimeBaseInitStructure.TIM_Period = 50 - 1; //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1; //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure); //初始化TIM2
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //允许定时器2更新中断
    TIM_Cmd(TIM2, ENABLE); //使能定时器2
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //定时器2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; //抢占优先级6
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) //溢出中断
    {
        PCout(13) = 1;
        delay_us(1);
        PCout(13) = 0;
        delay_us(1);
        uint16_t Data = 0x0000;
        for (uint8_t i = 0; i < 12; i++)
        {
            PCout(2) = 0;
            delay_us(1);
            PCout(2) = 1;
            delay_us(1);
            Data <<= 1;
            Data |= PCin(0);
        }
        ArmPos = Data;
        if (ArmJob == FIRE && fabs(ArmPos - ArmEndPoint) < 100 && ArmPos >= ArmEndPoint)
        {
            //printf("ArmPos = %.1f\r\n", ArmPos);
            Elmo_Stop(6);
            ArmJob = IDLE;
        }
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //清除中断标志位
}

void AbsEncoder_CAN_Query(uint8_t NodeID)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = NodeID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 4;
    TxMessage.Data[0] = 0x04; //长度LEN
    TxMessage.Data[1] = NodeID; //设备ID
    TxMessage.Data[2] = 0x01; //指令FUNC
    TxMessage.Data[3] = 0x00; //数据DATA
    CAN2_Send(&TxMessage);
}

void AbsEncoder_CAN_SetID(uint8_t NodeID, uint8_t NewNodeID)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = NodeID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 4;
    TxMessage.Data[0] = 0x04; //长度LEN
    TxMessage.Data[1] = NodeID; //设备ID
    TxMessage.Data[2] = 0x02; //指令FUNC
    TxMessage.Data[3] = NewNodeID; //数据DATA
    CAN2_Send(&TxMessage);
}

void AbsEncoder_CAN_SetMode(uint8_t NodeID, bool AutoSend)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = NodeID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 4;
    TxMessage.Data[0] = 0x04; //长度LEN
    TxMessage.Data[1] = NodeID; //设备ID
    TxMessage.Data[2] = 0x04; //指令FUNC
    TxMessage.Data[3] = AutoSend ? 0xAA : 0x00; //数据DATA
    CAN2_Send(&TxMessage);
}

void AbsEncoder_CAN_SetInterval(uint8_t NodeID, uint16_t Interval)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = NodeID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 5;
    TxMessage.Data[0] = 0x05; //长度LEN
    TxMessage.Data[1] = NodeID; //设备ID
    TxMessage.Data[2] = 0x05; //指令FUNC
    TxMessage.Data[3] = Interval & (0xFF); //数据DATA
    TxMessage.Data[4] = Interval >> 8; //数据DATA
    CAN2_Send(&TxMessage);
}

void AbsEncoder_CAN_SetZero(uint8_t NodeID)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = NodeID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 4;
    TxMessage.Data[0] = 0x04; //长度LEN
    TxMessage.Data[1] = NodeID; //设备ID
    TxMessage.Data[2] = 0x06; //指令FUNC
    TxMessage.Data[3] = 0x00; //数据DATA
    CAN2_Send(&TxMessage);
}
