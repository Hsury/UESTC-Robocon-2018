#include "sys.h"
#include "usart.h"
#include "Includes.h"

RingBuf_t UART_RingBuf;

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h" //FreeRTOSʹ��
#endif

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)

//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;
};

FILE __stdout;

//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x;
}

//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0); //ѭ������,ֱ���������
    USART1->DR = (uint8_t)ch;
    return ch;
}

#endif

#if EN_USART1_RX //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���

//��ʼ��IO ����1
//bound:������
void uart_init(uint32_t bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //ʹ��USART1ʱ��
 
    //����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9����ΪUSART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10����ΪUSART1
    
    //USART1�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9��PA10
    
    //USART1 ��ʼ������
    USART_InitStructure.USART_BaudRate = bound; //����������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No; //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    
    USART_Cmd(USART1, ENABLE); //ʹ�ܴ���1
    
    USART_ClearFlag(USART1, USART_FLAG_TC);
    
    #if EN_USART1_RX
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //��������ж�
    //USART1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //����1�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7; //��ռ���ȼ�7
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���
    #endif
    
    UART_RingBuf = RingBuf_Create(8, 1);
}

//����1�жϷ������
void USART1_IRQHandler(void)
{
    uint8_t tmp;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�����ж�
    {
        tmp = USART_ReceiveData(USART1); //(USART1->DR);
        RingBuf_Overwrite(&UART_RingBuf, &tmp);
        Checkout(&UART_RingBuf);
    }
    else
    {
        if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
        {
            USART_ClearFlag(USART1, USART_FLAG_ORE);
            USART_ReceiveData(USART1);
        }
        USART_ClearFlag(USART1, USART_IT_RXNE);
    }
}

#endif
