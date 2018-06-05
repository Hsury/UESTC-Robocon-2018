#include "CAN.h"

void Dual_CAN_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure; 
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOB, ENABLE);                   // GPIO时钟初始化
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);                     // CAN外设时钟初始化

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                                                  // 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                                 // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                                             // 翻转速度100MHz
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                                                  // 上拉
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;                                       // CAN1使用PD0与PD1
    GPIO_Init(GPIOD, &GPIO_InitStructure);                                                         // 初始化CAN1的GPIO
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;                                     // CAN2使用PB12与PB13
    GPIO_Init(GPIOB, &GPIO_InitStructure);                                                         // 初始化CAN2的GPIO

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);                                        // PD0复用为CAN1
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);                                        // PD1复用为CAN1
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
    
    CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;                      // 掩码过滤器
    CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;                      // 32位
    CAN_FilterInitStructure.CAN_FilterIdHigh         = 0x0000;                                     // 32位ID
    CAN_FilterInitStructure.CAN_FilterIdLow          = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0xE000;                                     // 32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0x0000;
    CAN_FilterInitStructure.CAN_FilterNumber         = 14;                                         // 过滤器14
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;                           // 过滤器14关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;                                     // 激活过滤器14
    CAN_FilterInit(&CAN_FilterInitStructure);                                                      // 滤波器初始化
    
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;                                      // 主优先级为8
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;                                      // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);                                                       // FIFO0消息挂号中断允许
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;                                      // 主优先级为8
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;                                      // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);                                                       // FIFO0消息挂号中断允许
    NVIC_Init(&NVIC_InitStructure);
}

uint8_t CAN1_Send(CanTxMsg* TxMessage)
{
    uint8_t mailbox;
    uint16_t i = 0;
    mailbox = CAN_Transmit(CAN1, TxMessage);
    while ((CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Failed) && (i < 0xFFF)) i++;
    if (i >= 0xFFF) return 1;
    return 0;
}

uint8_t CAN2_Send(CanTxMsg* TxMessage)
{
    uint8_t mailbox;
    uint16_t i = 0;
    mailbox = CAN_Transmit(CAN2, TxMessage);
    while ((CAN_TransmitStatus(CAN2, mailbox) == CAN_TxStatus_Failed) && (i < 0xFFF)) i++;
    if (i >= 0xFFF) return 1;
    return 0;
}

void CAN1_RX0_IRQHandler()
{
    CanRxMsg RxMessage;
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    switch (RxMessage.StdId)
    {
        case ELMO1_CAN_ID:
        Online |= (1 << 0);
        break;
        
        case ELMO2_CAN_ID:
        Online |= (1 << 1);
        break;
        
        case ELMO3_CAN_ID:
        Online |= (1 << 2);
        break;
    }
    // 在这里添加CAN1中断服务函数
}

void CAN2_RX0_IRQHandler()
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    uint32_t TS = millis();
    uint32_t DeltaTS;
    CanRxMsg RxMessage;
    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
    switch (RxMessage.StdId)
    {
        case GE_POS_CAN_ID:
        Online |= (1 << 3);
        if (GESwitch)
        {
            int32_t STmp;
            float PosXTmp, PosYTmp, RealVelXTmp, RealVelYTmp;
            DeltaTS = TS - EncoderTS;
            memcpy(&STmp, &RxMessage.Data[0], 4);
            PosXTmp = Encoder2RealX(STmp);
            RealVelXTmp = (PosXTmp - PosX) / DeltaTS * 1000;
            if (fabs(PosXTmp) > 8 || fabs(PosXTmp - PosX) > 1)
            {
                GyroEncoder_SetFlag(false);
                break;
            }
            RealAccX = (RealVelXTmp - RealVelX) / DeltaTS * 1000;
            RealVelX = RealVelXTmp;
            PosX = PosXTmp;
            memcpy(&STmp, &RxMessage.Data[4], 4);
            PosYTmp = Encoder2RealY(STmp);
            RealVelYTmp = (PosYTmp - PosY) / DeltaTS * 1000;
            if (fabs(PosYTmp) > 8 || fabs(PosYTmp - PosY) > 1)
            {
                GyroEncoder_SetFlag(false);
                break;
            }
            RealAccY = (RealVelYTmp - RealVelY) / DeltaTS * 1000;
            RealVelY = RealVelYTmp;
            PosY = PosYTmp;
            EncoderTS = TS;
        }
        break;
        
        case GE_ANG_CAN_ID:
        Online |= (1 << 3);
        if (GESwitch)
        {
            float FTmp;
            float PosZTmp, RealVelZTmp;
            DeltaTS = TS - GyroTS;
            memcpy(&FTmp, &RxMessage.Data[0], 4);
            PosZTmp = Gyro2RealZ(FTmp);
            RealVelZTmp = (DeltaAng(PosZTmp - PosZ)) / DeltaTS * 1000;
            if (FTmp < -180 || FTmp > 180 || fabs(PosZTmp - PosZ) > 90)
            {
                GyroEncoder_SetFlag(false);
                break;
            }
            RealAccZ = (RealVelZTmp - RealVelZ) / DeltaTS * 1000;
            RealVelZ = RealVelZTmp;
            PosZ = PosZTmp;
            GyroTS = TS;
        }
        break;
        
        case DT35_X_CAN_ID:
        Online |= (1 << 4);
        int32_t DT35XTmp;
        memcpy(&DT35XTmp, &RxMessage.Data[0], 4);
        DT35X = DT35XTmp / 1000.0f + DT35_X_STATIC_DIST;
        break;
        
        case DT35_Y_CAN_ID:
        Online |= (1 << 5);
        int32_t DT35YTmp;
        memcpy(&DT35YTmp, &RxMessage.Data[0], 4);
        DT35Y = DT35YTmp / 1000.0f + DT35_Y_STATIC_DIST;
        break;
        
        case KEYBOARD_CAN_ID:
        switch (RxMessage.Data[0])
        {
            case 0x08: // 初始化
            xTaskNotifyFromISR(FlowTask_Handler, GET_READY, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            break;
            
            case 0x06: // 开启吸盘
            GoalX = PosX;
            GoalY = PosY;
            GoalZ = PosZ;
            Pin();
            Cradle_AdjustNotify();
            //Move_Init();
            //xTaskNotifyFromISR(MoveTask_Handler, LOCKPOINT, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            break;
            
            case 0x04: // 复位
            NVIC_SystemReset();
            break;
            
            case 0x02: // 发车
            Cradle_RestartNotify(TZ1);
            xTaskNotifyFromISR(FlowTask_Handler, LAUNCH, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            break;
            
            case 0x01: // 重试
            xTaskNotifyFromISR(FlowTask_Handler, MOVE_ON, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
            break;
            
            case 0x07: // 到TZ1
            Cradle_ArriveNotify(TZ1);
            break;
            
            case 0x05: // 到TZ2
            Cradle_ArriveNotify(TZ2);
            break;
            
            case 0x03: // 到TZ3
            Cradle_ArriveNotify(TZ3);
            break;
        }
        break;
        
        case CRADLE_CAN_ID:
        Move_UpdateZone();
        if (RxMessage.Data[0] == 0xBB && RxMessage.Data[1] == 0x02 && Zone == TZ1)
        {
            xTaskNotifyFromISR(FlowTask_Handler, MOVE_ON, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
        }
        else if (RxMessage.Data[0] == 0xBB && RxMessage.Data[1] == 0x03 && Zone == TZ2)
        {
            xTaskNotifyFromISR(FlowTask_Handler, MOVE_ON, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
        }
        else if (RxMessage.Data[0] == 0xEE)
        {
            xTaskNotifyFromISR(WirelessTask_Handler, RxMessage.Data[1], eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
        }
        break;
        
        case START_DASH_CAN_ID:;
        int32_t IncX, IncY;
        memcpy(&IncX, &RxMessage.Data[0], 4);
        memcpy(&IncY, &RxMessage.Data[4], 4);
        Move_EnterDash(IncX / 1000.0f, IncY / 1000.0f);
        xTaskNotifyFromISR(MoveTask_Handler, DASH, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
        break;
        
        case EXIT_DASH_CAN_ID:
        Move_ExitDash();
        xTaskNotifyFromISR(MoveTask_Handler, LOCKPOINT, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
        break;
        
        case PROBE_ONLINE_CAN_ID:
        Probe_ReportOnline();
        break;
    }
    if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
    // 在这里添加CAN2中断服务函数
}

/*
// 陀螺仪与码盘数据异常保护
if (!GEError && fabs(PosXTmp) > 8)
{
    GEError = true;
    GEErrorTS = millis();
    vTaskSuspend(MoveTask);
    Omni_Elmo_Stop();
    Buzzer_On();
    break;
}
else if (GEError && fabs(PosXTmp) <= 8)
{
    GEError = false;
    GEErrorDuration += millis() - GEErrorTS;
    vTaskResume(MoveTask);
    Buzzer_Off();
}
*/
