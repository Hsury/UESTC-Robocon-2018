#include "ESP8266.h"

SemaphoreHandle_t ATOKSemaphore;
RingBuf_t ESP8266_RingBuf;

void ESP8266_Init(uint32_t Baudrate)
{
    // PA0 = UART4_Tx; PA1 = UART4_Rx
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = Baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
    
    USART_Cmd(UART4, ENABLE);
    
    USART_ClearFlag(UART4, USART_FLAG_TC);
    
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    ATOKSemaphore = xSemaphoreCreateBinary();
    ESP8266_RingBuf = RingBuf_Create(8, 1);
}

void UART4_IRQHandler(void)
{
    uint8_t tmp;
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //接收中断
    {
        tmp = USART_ReceiveData(UART4); //(UART4->DR);
        if (tmp == 0xAB) Cradle_RetryNotify(Zone);
        RingBuf_Overwrite(&ESP8266_RingBuf, &tmp);
        Checkout(&ESP8266_RingBuf);
        char AT_Response[4];
        for (uint8_t i = 0; i < 4; i++) RingBuf_Peek(&ESP8266_RingBuf, 4 + i, (uint8_t*)(&AT_Response[i]));
        if (!strncmp(AT_Response, "OK\r\n", 4))
        {
            BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(ATOKSemaphore, &pxHigherPriorityTaskWoken);
            if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
        }
    }
    else
    {
        if (USART_GetFlagStatus(UART4, USART_FLAG_ORE) == SET)
        {
            USART_ClearFlag(UART4, USART_FLAG_ORE);
            USART_ReceiveData(UART4);
        }
        USART_ClearFlag(UART4, USART_IT_RXNE);
    }
}

void ESP8266_ExitTransLink()
{
    delay_ms(250);
    Printf(ESP8266, "+++");
    delay_ms(1250);
}

bool ESP8266_UARTConfig(uint32_t Baudrate)
{
    //8 bit数据位，1 bit停止位，无校验位，不使能流控
    xSemaphoreTake(ATOKSemaphore, 0);
    Printf(ESP8266, "AT+UART_DEF=%u,8,1,0,0\r\n", Baudrate);
    delay_ms(100);
    if (xSemaphoreTake(ATOKSemaphore, pdMS_TO_TICKS(15000)) == pdTRUE)
    {
        USART_Cmd(UART4, DISABLE);
        USART_InitTypeDef USART_InitStructure;
        USART_InitStructure.USART_BaudRate = Baudrate;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(UART4, &USART_InitStructure);
        USART_Cmd(UART4, ENABLE);
        delay_ms(100);
        return true;
    }
    else return false;
}

bool ESP8266_WiFiModeConfig(uint8_t Mode)
{
    xSemaphoreTake(ATOKSemaphore, 0);
    Printf(ESP8266, "AT+CWMODE_DEF=%u\r\n", Mode);
    delay_ms(100);
    if (xSemaphoreTake(ATOKSemaphore, pdMS_TO_TICKS(15000)) == pdTRUE) return true;
    else return false;
}

bool ESP8266_StationConfig(char *SSID, char *Password)
{
    xSemaphoreTake(ATOKSemaphore, 0);
    Printf(ESP8266, "AT+CWJAP_DEF=\"%s\",\"%s\"\r\n", SSID, Password);
    delay_ms(100);
    if (xSemaphoreTake(ATOKSemaphore, pdMS_TO_TICKS(15000)) == pdTRUE) return true;
    else return false;
}

bool ESP8266_StationDHCPConfig(bool isEnable)
{
    xSemaphoreTake(ATOKSemaphore, 0);
    Printf(ESP8266, "AT+CWDHCP_DEF=1,%u\r\n", isEnable);
    delay_ms(100);
    if (xSemaphoreTake(ATOKSemaphore, pdMS_TO_TICKS(15000)) == pdTRUE) return true;
    else return false;
}

bool ESP8266_StationIPConfig(char *IPAddress, char *Gateway, char *Netmask)
{
    xSemaphoreTake(ATOKSemaphore, 0);
    Printf(ESP8266, "AT+CIPSTA_DEF=\"%s\",\"%s\",\"%s\"\r\n", IPAddress, Gateway, Netmask);
    delay_ms(100);
    if (xSemaphoreTake(ATOKSemaphore, pdMS_TO_TICKS(15000)) == pdTRUE) return true;
    else return false;
}

bool ESP8266_SoftAPConfig(char *SSID, char *Password, uint8_t Channel, uint8_t Encryption, uint8_t MaxConn, bool HideSSID)
{
    xSemaphoreTake(ATOKSemaphore, 0);
    Printf(ESP8266, "AT+CWSAP_DEF=\"%s\",\"%s\",%u,%u,%u,%u\r\n", SSID, Password, Channel, Encryption, MaxConn, HideSSID);
    delay_ms(100);
    if (xSemaphoreTake(ATOKSemaphore, pdMS_TO_TICKS(15000)) == pdTRUE) return true;
    else return false;
}

bool ESP8266_TransLinkConfig(char *IPAddress, uint16_t RemotePort, uint8_t Protocol, uint16_t LocalPort)
{
    xSemaphoreTake(ATOKSemaphore, 0);
    Printf(ESP8266, "AT+SAVETRANSLINK=1,\"%s\",%u,\"%s\",%u\r\n", IPAddress, RemotePort, Protocol == UDP ? "UDP" : "TCP", LocalPort);
    delay_ms(100);
    if (xSemaphoreTake(ATOKSemaphore, pdMS_TO_TICKS(15000)) == pdTRUE) return true;
    else return false;
}

bool ESP8266_Reset()
{
    xSemaphoreTake(ATOKSemaphore, 0);
    Printf(ESP8266, "AT+RST\r\n");
    delay_ms(100);
    if (xSemaphoreTake(ATOKSemaphore, pdMS_TO_TICKS(15000)) == pdTRUE) return true;
    else return false;
}
