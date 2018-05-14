#include "Protocol.h"

uint8_t Checkout(RingBuf_t* Q)
{
    BaseType_t pxHigherPriorityTaskWoken;
    uint8_t HeadByte, SeperatorByte, TailByte;
    RingBuf_Peek(Q, 0, &HeadByte);
    RingBuf_Peek(Q, 2, &SeperatorByte);
    RingBuf_Peek(Q, 7, &TailByte);
    if (HeadByte == '{' && SeperatorByte == '|' && TailByte == '}')
    {
        uint8_t Addr;
        union
        {
            uint8_t B[4];
            uint32_t U;
            float F;
        }
        Data;
        RingBuf_Peek(Q, 1, &Addr);
        for (uint8_t i = 0; i < 4; i++) RingBuf_Peek(Q, 3 + i, &Data.B[i]);
        PackUp(0xFF, Data.B); // 回声
        switch (Addr)
        {
            case 0x00: // 全局级别
            switch (Data.U)
            {
                case 0x00000000: // 报告外设上线状态
                PackUp(0x00, (uint8_t*)(&Online));
                break;
                
                case 0x00000001: // 初始化底盘
                Move_Init();
                break;
                
                case 0x00000002: // 初始化抛射机构
                Sling_Init();
                break;
                
                case 0x00000003: // 开始执行底盘运动任务
                //vTaskNotifyGiveFromISR(MoveTask_Handler, &pxHigherPriorityTaskWoken);
                break;
                
                case 0x00000004: // 底盘锁当前点
                xTaskNotifyFromISR(MoveTask_Handler, LOCKPOINT, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
                break;
                
                case 0xFFFFFFFF: // 复位主控
                NVIC_SystemReset();
                break;
            }
            break;
            
            case 0x10: // 底盘总控级别
            switch (Data.U)
            {
                case 0x00000000: // 释放底盘所有电机
                Omni_Elmo_Close();
                break;
                
                case 0x00000001: // 抱死底盘所有电机
                Omni_Elmo_Stop();
                break;
                
                case 0x00000002: // 底盘运动生效
                Omni_Elmo_PVM();
                break;
            }
            break;
            
            case 0x11: // 预设置底盘X轴运动速度
            VelX = Data.F;
            break;
            
            case 0x12: // 预设置底盘Y轴运动速度
            VelY = Data.F;
            break;
            
            case 0x13: // 预设置底盘Z轴运动速度
            VelZ = Data.F;
            break;
            
            case 0x20: // 陀螺仪与码盘总控级别
            switch (Data.U)
            {
                case 0x00000000: // 陀螺仪与码盘初始化
                GyroEncoder_Reset();
                PackUp(0xFF, Data.B); // 初始化完成提示
                break;
                
                case 0x00000001: // 陀螺仪与码盘清零
                GyroEncoder_Clear();
                PackUp(0xFF, Data.B); // 初始化完成提示
                break;
            }
            break;
            
            case 0x21: // 设置码盘X轴坐标
            PosX = Data.F;
            GyroEncoder_SetPos();
            break;
            
            case 0x22: // 设置码盘Y轴坐标
            PosY = Data.F;
            GyroEncoder_SetPos();
            break;
            
            case 0x23: // 设置陀螺仪Z轴坐标
            PosZ = Data.F;
            GyroEncoder_SetAng();
            break;
            
            case 0x30: // 抛射机构总控级别
            switch (Data.U)
            {
                case 0x00000001: // 投TZ1
                xSemaphoreGiveFromISR(JustThrowSemaphore, &pxHigherPriorityTaskWoken);
                xTaskNotifyFromISR(FireTask_Handler, TZ1, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
                break;
                
                case 0x00000002: // 投TZ2
                xSemaphoreGiveFromISR(JustThrowSemaphore, &pxHigherPriorityTaskWoken);
                xTaskNotifyFromISR(FireTask_Handler, TZ2, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
                break;
                
                case 0x00000003: // 投TZ3
                xSemaphoreGiveFromISR(JustThrowSemaphore, &pxHigherPriorityTaskWoken);
                xTaskNotifyFromISR(FireTask_Handler, TZ3, eSetValueWithOverwrite, &pxHigherPriorityTaskWoken);
                break;
            }
            break;
        }
        if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
        return 1;
    }
    else return 0;
}

void PackUp(uint8_t Addr, uint8_t* Data)
{
    uint8_t Buf[8];
    Buf[0] = '{';
    Buf[1] = Addr;
    Buf[2] = '|';
    Buf[3] = Data[0];
    Buf[4] = Data[1];
    Buf[5] = Data[2];
    Buf[6] = Data[3];
    Buf[7] = '}';
    for (uint8_t i = 0; i < 8; i++)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *(Buf + i));
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }
    if (isVCPAvailable) VCP_DataTx(Buf, 8);
    //VCP_DataTx("\r\n", 2);
}
