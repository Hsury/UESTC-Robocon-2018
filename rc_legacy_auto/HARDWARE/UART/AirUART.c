/*

  /\\\        /\\\  /\\\\\\\\\\\\\\\     /\\\\\\\\\\\    /\\\\\\\\\\\\\\\        /\\\\\\\\\         
  \/\\\       \/\\\ \/\\\///////////    /\\\/////////\\\ \///////\\\/////      /\\\////////         
   \/\\\       \/\\\ \/\\\              \//\\\      \///        \/\\\         /\\\/                 
    \/\\\       \/\\\ \/\\\\\\\\\\\       \////\\\               \/\\\        /\\\                  
     \/\\\       \/\\\ \/\\\///////           \////\\\            \/\\\       \/\\\                 
      \/\\\       \/\\\ \/\\\                     \////\\\         \/\\\       \//\\\               
       \//\\\      /\\\  \/\\\              /\\\      \//\\\        \/\\\        \///\\\            
         \///\\\\\\\\\/   \/\\\\\\\\\\\\\\\ \///\\\\\\\\\\\/         \/\\\          \////\\\\\\\\\  
            \/////////     \///////////////    \///////////           \///              \/////////  

                      ===== UESTC Air UART Driver For ABU Robocon 2018 =====
                              Copyright (c) 2018 HsuRY <i@hsury.com>

                                        VERSION 2018/03/01

*/

#include "AirUART.h"

uint8_t SM = 0;
uint8_t Response = UNKNOWN;

uint8_t DataBuf[128] = {0};
uint8_t DataPtr = 0;
uint8_t DataValid = 0;

uint16_t CRCBuf = 0x0000;

// 注意：当前协议下，不允许数据中出现'{', '|', '}'三种字符

/**
  * @brief  初始化USART2外设相关
  * @param  Baudrate: 波特率
  * @retval 无
  */
void AirUART_Init(uint32_t Baudrate)
{
    // PD5 = USART2_Tx; PD6 = USART2_Rx
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = Baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    
    USART_Cmd(USART2, ENABLE);
    
    USART_ClearFlag(USART2, USART_FLAG_TC);
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    AirUART_Send_Raw(0x00);
}

/**
  * @brief  向外发送数据，不等待响应
  * @param  Data: 要发送的数据指针
  * @param  Len: 要发送的数据的长度
  * @retval 无
  */
void AirUART_Send(uint8_t *Data, uint8_t Len)
{
    if (Len > 0 && Len <= 128)
    {
        Response = UNKNOWN;
        AirUART_Send_Raw('{');
        for (uint8_t i = 0; i < Len; i++)
        {
            AirUART_Send_Raw(Data[i]);
        }
        AirUART_Send_Raw('|');
        uint16_t CRCTemp = AirUART_Calc_CRC(Data, Len);
        AirUART_Send_Raw((CRCTemp >> 8) & 0xFF);
        AirUART_Send_Raw(CRCTemp & 0xFF);
        AirUART_Send_Raw('}');
    }
}

/**
  * @brief  向外发送数据，等待另一方作出确认响应，或是达到了重传最大尝试次数
  * @param  Data: 要发送的数据指针
  * @param  Len: 要发送的数据的长度
  * @param  RetryTimes: 重传最大尝试次数
  * @retval 0: 传输失败
            1: 传输成功
  */
uint8_t AirUART_Send_And_Wait(uint8_t *Data, uint8_t Len, uint8_t RetryTimes)
{
    uint32_t TS = 0;
    uint8_t RE = 0;
    AirUART_Send(Data, Len);
    while (RE <= RetryTimes)
    {
        TS = xTaskGetTickCount() * portTICK_PERIOD_MS;
        while (xTaskGetTickCount() * portTICK_PERIOD_MS < TS + 250)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            if (AirUART_Get_Response() == SUCC) return 1;
            else if (AirUART_Get_Response() == FAIL) break;
        }
        AirUART_Send(Data, Len);
        RE++;
    }
    return 0;
}

/**
  * @brief  获取来自对方的响应
  * @param  无
  * @retval UNKNOWN: 未得到响应
            SUCC: 对方响应传输成功
            FAIL: 对方响应传输失败
  */
uint8_t AirUART_Get_Response()
{
    return Response;
}

/**
  * @brief  获取接收缓冲区中的数据长度
  * @param  无
  * @retval 接收缓冲区中的数据长度
  */
uint8_t AirUART_Available()
{
    if (DataValid) return DataPtr;
    else return 0;
}

/**
  * @brief  从接收缓冲区中取出全部数据
  * @param  Data: 用来转移数据的目标指针
  * @retval 取出的数据长度
  */
uint8_t AirUART_Receive(uint8_t *Data)
{
    uint8_t Len = AirUART_Available();
    if (Len)
    {
        memcpy(Data, DataBuf, Len);
        DataPtr = 0;
    }
    return Len;
}

void AirUART_Receive_Callback(uint8_t *Data, uint8_t Len)
{
    // 在这里填写从空中串口接收到有效数据后自动执行的代码
    // 尽可能在这里释放信号量，然后在任务中作出处理
}

void USART2_IRQHandler()
{
    uint8_t Temp;
    uint8_t Skip = 0;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        Temp = USART_ReceiveData(USART2);
        switch (Temp)
        {
            case '{': DataPtr = 0; DataValid = 0; CRCBuf = 0x0000; SM = HEAD; Skip = 1; break;
            case '|': if (SM == HEAD) {if (DataPtr > 0) {SM = SEPERATOR; Skip = 1;} else SM = RST;} else if (SM == RST) SM = ACK; break;
            case '}': if (SM == SEPERATOR) SM = DATA_TAIL; if (SM == RST) SM = RST_TAIL; if (SM == ACK) SM = ACK_TAIL; break;
        }
        if (!Skip)
        {
            switch (SM)
            {
                case HEAD: if (DataPtr < 128) DataBuf[DataPtr++] = Temp; break;
                case SEPERATOR: CRCBuf = (CRCBuf << 8) | Temp; break;
                case DATA_TAIL: if (AirUART_Calc_CRC(DataBuf, DataPtr) == CRCBuf) {DataValid = 1; AirUART_Send_ACK(); AirUART_Receive_Callback(DataBuf, DataPtr);} else AirUART_Send_RST(); SM = FIN; break;
                case RST_TAIL: Response = FAIL; SM = FIN; break;
                case ACK_TAIL: Response = SUCC; SM = FIN; break;
            }
        }
    }
    else
    {
        if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) == SET)
        {
            USART_ClearFlag(USART2, USART_FLAG_ORE);
            USART_ReceiveData(USART2);
        }
        USART_ClearFlag(USART2, USART_IT_RXNE);
    }
}

void AirUART_Send_Raw(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void AirUART_Send_RST()
{
    AirUART_Send_Raw('{');
    AirUART_Send_Raw('|');
    AirUART_Send_Raw('}');
}

void AirUART_Send_ACK()
{
    AirUART_Send_Raw('{');
    AirUART_Send_Raw('|');
    AirUART_Send_Raw('|');
    AirUART_Send_Raw('}');
}

const uint8_t CRC_High_Byte_Table[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

const uint8_t CRC_Low_Byte_Table[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

uint16_t AirUART_Calc_CRC(uint8_t *Data, uint16_t Len)
{
    uint8_t High_Byte = 0xFF;
	uint8_t Low_Byte = 0xFF;
    uint32_t Index;
    while (Len--)
    {
        Index = High_Byte ^ *(Data++);
        High_Byte = Low_Byte ^ CRC_High_Byte_Table[Index];
        Low_Byte = CRC_Low_Byte_Table[Index];
    }
    return ((High_Byte << 8) | Low_Byte);
}
