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

                 ===== UESTC RingBuffer Data Structure For ABU Robocon 2018 =====
                              Copyright (c) 2018 HsuRY <i@hsury.com>

                                        VERSION 2018/03/06

*/

#include "RingBuf.h"

// ע�⣺��ʵ����Ϊ�����ֻ��λ�����Ϊ�ջ���Ϊ��������ʧһ��Ԫ�صĴ洢�ռ䡣�����λ��С����1Byte�������ڴ�Խ����ɿ���ʹ�ñ�־λ������

/**
  * @brief  ����һ�����λ�������FIFO��
  * @param  QSize: �ɴ��Ԫ�ص�����
            ESize: ����Ԫ��ռ�õ��ֽ���
  * @retval һ��RingBuf_t�ṹ��
  */
RingBuf_t RingBuf_Create(uint16_t QSize, uint16_t ESize)
{
    //QSize++; // �����Ի��ʵ�ʿ��ô�С��ͬ�ڴ������QSize�Ļ��λ�����
    uint8_t Data[QSize][ESize];
    memset(Data, 0, QSize * ESize);
    RingBuf_t RingBuf;
    RingBuf.Front = 0;
    RingBuf.Rear = 0;
    RingBuf.QSize = QSize;
    RingBuf.ESize = ESize;
    RingBuf.Data = &Data[0][0];
    return RingBuf;
}

/**
  * @brief  ��������λ��һ�����λ�����
  * @param  Q: RingBuf_t�ṹ��
  * @retval ��
  */
void RingBuf_Wipe(RingBuf_t* Q)
{
    memset(Q, 0, Q->QSize * Q->ESize);
    Q->Front = 0;
    Q->Rear = 0;
}

/**
  * @brief  ���ػ��λ�������ռ����
  * @param  Q: RingBuf_t�ṹ��
  * @retval ���������Ѵ�ŵ�Ԫ������
  */
uint16_t RingBuf_Usage(RingBuf_t* Q)
{
    return ((Q->Rear - Q->Front + Q->QSize) % Q->QSize);
}

/**
  * @brief  ���ػ��λ������Ŀ�����
  * @param  Q: RingBuf_t�ṹ��
  * @retval �������л��ɴ����Ԫ������
  */
uint16_t RingBuf_Available(RingBuf_t* Q)
{
    return (Q->QSize - 1 - RingBuf_Usage(Q));
}

/**
  * @brief  ���ػ��λ������Ƿ�Ϊ��
  * @param  Q: RingBuf_t�ṹ��
  * @retval 0: ��Ϊ��
            1: Ϊ��
  */
uint8_t RingBuf_isEmpty(RingBuf_t* Q)
{
    if (RingBuf_Usage(Q) == 0) return 1;
    else return 0;
}

/**
  * @brief  ���ػ��λ������Ƿ�Ϊ��
  * @param  Q: RingBuf_t�ṹ��
  * @retval 0: ��Ϊ��
            1: Ϊ��
  */
uint8_t RingBuf_isFull(RingBuf_t* Q)
{
    if (RingBuf_Usage(Q) == Q->QSize - 1) return 1;
    else return 0;
}

/**
  * @brief  ���������δ�����������з���Ԫ��
  * @param  Q: RingBuf_t�ṹ��
            Data: Ҫ������ֽ��������ʼ��ַ
  * @retval 0: ����������������ʧ��
            1: ���óɹ�
  */
uint8_t RingBuf_Put(RingBuf_t* Q, uint8_t* Data)
{
    if (!RingBuf_isFull(Q))
    {
        memcpy(&Q->Data[Q->Rear], Data, Q->ESize);
        Q->Rear = (Q->Rear + 1) % Q->QSize;
        return 1;
    }
    else return 0;
}

/**
  * @brief  ǿ�����������з���Ԫ�أ��������򸲸�������Ǹ�Ԫ��
  * @param  Q: RingBuf_t�ṹ��
            Data: Ҫ������ֽ��������ʼ��ַ
  * @retval ��
  */
void RingBuf_Overwrite(RingBuf_t* Q, uint8_t* Data)
{
    if (RingBuf_isFull(Q)) Q->Front = (Q->Front + 1) % Q->QSize;
    RingBuf_Put(Q, Data);
}

/**
  * @brief  ���Դӻ�����ȡ��Ԫ��
  * @param  Q: RingBuf_t�ṹ��
            Data: ���ڴ��ȡ��Ԫ�ص��ֽ��������ʼ��ַ
  * @retval 0: ������Ϊ�գ�ȡ��ʧ��
            1: ȡ�سɹ�
  */
uint8_t RingBuf_Get(RingBuf_t* Q, uint8_t* Data)
{
    if (!RingBuf_isEmpty(Q))
    {
        memcpy(Data, &Q->Data[Q->Front], Q->ESize);
        Q->Front = (Q->Front + 1) % Q->QSize;
        return 1;
    }
    else return 0;
}

/**
  * @brief  ����������ָ��λ�õ�Ԫ��
  * @param  Q: RingBuf_t�ṹ��
            Index: Ҫ��Ԫ�ص������ţ�0Ϊ��ɣ�(QSize - 1)Ϊ����
            Data: ���ڴ��ȡ��Ԫ�ص��ֽ��������ʼ��ַ
  * @retval ��
  */
void RingBuf_Peek(RingBuf_t* Q, uint16_t Index, uint8_t* Data)
{
    memcpy(Data, &Q->Data[(Q->Rear + Index) % Q->QSize], Q->ESize);
}

// ���������ģʽ���ò�ͬ������
void _ttywrch(int ch)
{
    ch = ch;
}
