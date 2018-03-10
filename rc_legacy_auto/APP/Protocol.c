#include "Protocol.h"

extern TaskHandle_t MoveTask_Handler;

uint8_t Checkout(RingBuf_t* Q)
{
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
        PackUp(0xFF, Data.B); // ����
        switch (Addr)
        {
            case 0x00: // ȫ�ּ���
            switch (Data.U)
            {
                case 0x00000000:; // ������������״̬
                PackUp(0x00, (uint8_t*)(&Online));
                break;
                
                case 0x00000001: // ��ʼ����������
                Online = 0x0000;
                Elmo_Init(CAN1, 9, 0);
                GyroEncoder_Reset();
                VelX = 0;
                VelY = 0;
                VelZ = 0;
                PosX = 0;
                PosY = 0;
                AngZ = 0;
                break;
                
                case 0x00000002:; // ��ʼִ�е����˶�����
                BaseType_t pxHigherPriorityTaskWoken;
                vTaskNotifyGiveFromISR(MoveTask_Handler, &pxHigherPriorityTaskWoken);
                if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
                break;
                
                case 0xFFFFFFFF: // ��λ����
                Elmo_Close(0);
                delay_xms(100);
                NVIC_SystemReset();
                break;
            }
            break;
            
            case 0x10: // �����ܿؼ���
            switch (Data.U)
            {
                case 0x00000000: // �ͷŵ������е��
                Omni_Elmo_Close();
                VelX = 0;
                VelY = 0;
                VelZ = 0;
                break;
                
                case 0x00000001: // �����������е��
                Omni_Elmo_Stop();
                VelX = 0;
                VelY = 0;
                VelZ = 0;
                break;
                
                case 0x00000002: // �����˶���Ч
                Omni_Elmo_PVM();
                break;
            }
            break;
            
            case 0x11: // Ԥ���õ���X���˶��ٶ�
            VelX = Data.F;
            break;
            
            case 0x12: // Ԥ���õ���Y���˶��ٶ�
            VelY = Data.F;
            break;
            
            case 0x13: // Ԥ���õ���Z���˶��ٶ�
            VelZ = Data.F;
            break;
            
            case 0x20: // �������������ܿؼ���
            switch (Data.U)
            {
                case 0x00000000: // �����������̳�ʼ��
                GyroEncoder_Reset();
                PosX = 0;
                PosY = 0;
                AngZ = 0;
                PackUp(0xFF, Data.B); // ��ʼ�������ʾ
                break;
                
                case 0x00000001: // ����������������
                GyroEncoder_Clear();
                PosX = 0;
                PosY = 0;
                AngZ = 0;
                PackUp(0xFF, Data.B); // ��ʼ�������ʾ
                break;
            }
            break;
            
            case 0x21: // ��������X������
            PosX = Data.F;
            GyroEncoder_SetPos();
            break;
            
            case 0x22: // ��������Y������
            PosY = Data.F;
            GyroEncoder_SetPos();
            break;
            
            case 0x23: // ����������Z������
            AngZ = Data.F;
            GyroEncoder_SetAng();
            break;
        }
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
        USART_SendData(USART1, *(Buf + i));
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
    if (isVCPAvailable) VCP_DataTx(Buf, 8);
    //VCP_DataTx("\r\n", 2);
}
