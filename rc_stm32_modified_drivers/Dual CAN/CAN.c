#include "CAN.h"

GPIO_InitTypeDef       GPIO_InitStructure; 
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
NVIC_InitTypeDef       NVIC_InitStructure;

CanTxMsg CAN1TxMessage;
CanRxMsg CAN1RxMessage;
CanTxMsg CAN2TxMessage;
CanRxMsg CAN2RxMessage;

void Dual_CAN_Init()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);                   // GPIO时钟初始化
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);                     // CAN外设时钟初始化

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                                                  // 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                                 // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                                             // 翻转速度100MHz
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                                                  // 上拉
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;                                     // CAN1使用PA11与PA12
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                                         // 初始化CAN1的GPIO
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;                                     // CAN2使用PB12与PB13
    GPIO_Init(GPIOB, &GPIO_InitStructure);                                                         // 初始化CAN2的GPIO

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);                                       // PA11复用为CAN1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);                                       // PA12复用为CAN1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);                                       // PB12复用为CAN2
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);                                       // PB13复用为CAN2

    CAN_InitStructure.CAN_TTCM      = DISABLE;                                                     // 非时间触发通信模式
    CAN_InitStructure.CAN_ABOM      = DISABLE;                                                     // 软件自动离线管理
    CAN_InitStructure.CAN_AWUM      = DISABLE;                                                     // 睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART      = DISABLE;                                                     // 禁止报文自动传送
    CAN_InitStructure.CAN_RFLM      = DISABLE;                                                     // 报文不锁定,新的覆盖旧的
    CAN_InitStructure.CAN_TXFP      = ENABLE;                                                      // 优先级由报文标识符决定
    CAN_InitStructure.CAN_Mode      = CAN_Mode_Normal;                                             // 模式设置
    CAN_InitStructure.CAN_SJW       = CAN_SJW_1tq;                                                 // 重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1       = CAN_BS1_9tq;                                                 // Tbs1范围CAN_BS1_1tq ~ CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2       = CAN_BS2_4tq;                                                 // Tbs2范围CAN_BS2_1tq ~ CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = 3;                                                           // 分频系数(Fdiv)为brp+1
    CAN_Init(CAN1, &CAN_InitStructure);                                                            // 初始化CAN1
    CAN_Init(CAN2, &CAN_InitStructure);                                                            // 初始化CAN2

    CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;                      // 掩码过滤器
    CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;                      // 32位
    CAN_FilterInitStructure.CAN_FilterIdHigh         = 0x0000;                                     // 32位ID
    CAN_FilterInitStructure.CAN_FilterIdLow          = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0x0000;                                     // 32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0x0000;
    CAN_FilterInitStructure.CAN_FilterNumber         = 0;                                          // 过滤器0
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;                           // 过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;                                     // 激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);                                                      // 滤波器初始化
    CAN_FilterInitStructure.CAN_FilterNumber         = 14;                                         // 过滤器14
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;                           // 过滤器14关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;                                     // 激活过滤器14
    CAN_FilterInit(&CAN_FilterInitStructure);                                                      // 滤波器初始化
    
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;                                      // 主优先级为9
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;                                      // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);                                                       // FIFO0消息挂号中断允许
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;                                     // 主优先级为10
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;                                      // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);                                                       // FIFO0消息挂号中断允许
    NVIC_Init(&NVIC_InitStructure);
}

uint8_t CAN1_Send()
{
    uint8_t mailbox;
    uint16_t i = 0;
    mailbox = CAN_Transmit(CAN1, &CAN1TxMessage);
    while ((CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Failed) && (i < 0xFFF)) i++;
    if (i >= 0xFFF) return 1;
    return 0;
}

uint8_t CAN2_Send()
{
    uint8_t mailbox;
    uint16_t i = 0;
    mailbox = CAN_Transmit(CAN2, &CAN2TxMessage);
    while ((CAN_TransmitStatus(CAN2, mailbox) == CAN_TxStatus_Failed) && (i < 0xFFF)) i++;
    if (i >= 0xFFF) return 1;
    return 0;
}

void CAN1_RX0_IRQHandler()
{
    CAN_Receive(CAN1, CAN_FIFO0, &CAN1RxMessage);
    // 在这里添加CAN1中断服务函数
}

void CAN2_RX0_IRQHandler()
{
    CAN_Receive(CAN2, CAN_FIFO0, &CAN2RxMessage);
    // 在这里添加CAN2中断服务函数
}
