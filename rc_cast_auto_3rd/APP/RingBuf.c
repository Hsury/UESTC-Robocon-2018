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

// 注意：本实现中为了区分环形缓冲区为空还是为满，将损失一个元素的存储空间。如果单位大小超过1Byte，并且内存吃紧，可考虑使用标志位来区分

/**
  * @brief  创建一个环形缓冲区（FIFO）
  * @param  QSize: 可存放元素的数量
            ESize: 单个元素占用的字节数
  * @retval 一个RingBuf_t结构体
  */
RingBuf_t RingBuf_Create(uint16_t QSize, uint16_t ESize)
{
    //QSize++; // 加上以获得实际可用大小等同于传入参数QSize的环形缓冲区
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
  * @brief  擦除（复位）一个环形缓冲区
  * @param  Q: RingBuf_t结构体
  * @retval 无
  */
void RingBuf_Wipe(RingBuf_t* Q)
{
    memset(Q, 0, Q->QSize * Q->ESize);
    Q->Front = 0;
    Q->Rear = 0;
}

/**
  * @brief  返回环形缓冲区的占用量
  * @param  Q: RingBuf_t结构体
  * @retval 缓冲区中已存放的元素数量
  */
uint16_t RingBuf_Usage(RingBuf_t* Q)
{
    return ((Q->Rear - Q->Front + Q->QSize) % Q->QSize);
}

/**
  * @brief  返回环形缓冲区的空余量
  * @param  Q: RingBuf_t结构体
  * @retval 缓冲区中还可存入的元素数量
  */
uint16_t RingBuf_Available(RingBuf_t* Q)
{
    return (Q->QSize - 1 - RingBuf_Usage(Q));
}

/**
  * @brief  返回环形缓冲区是否为空
  * @param  Q: RingBuf_t结构体
  * @retval 0: 不为空
            1: 为空
  */
uint8_t RingBuf_isEmpty(RingBuf_t* Q)
{
    if (RingBuf_Usage(Q) == 0) return 1;
    else return 0;
}

/**
  * @brief  返回环形缓冲区是否为满
  * @param  Q: RingBuf_t结构体
  * @retval 0: 不为满
            1: 为满
  */
uint8_t RingBuf_isFull(RingBuf_t* Q)
{
    if (RingBuf_Usage(Q) == Q->QSize - 1) return 1;
    else return 0;
}

/**
  * @brief  如果缓冲区未满，则往其中放入元素
  * @param  Q: RingBuf_t结构体
            Data: 要存入的字节数组的起始地址
  * @retval 0: 缓冲区已满，放置失败
            1: 放置成功
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
  * @brief  强制往缓冲区中放入元素，若已满则覆盖最早的那个元素
  * @param  Q: RingBuf_t结构体
            Data: 要存入的字节数组的起始地址
  * @retval 无
  */
void RingBuf_Overwrite(RingBuf_t* Q, uint8_t* Data)
{
    if (RingBuf_isFull(Q)) Q->Front = (Q->Front + 1) % Q->QSize;
    RingBuf_Put(Q, Data);
}

/**
  * @brief  尝试从缓冲区取出元素
  * @param  Q: RingBuf_t结构体
            Data: 用于存放取出元素的字节数组的起始地址
  * @retval 0: 缓冲区为空，取回失败
            1: 取回成功
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
  * @brief  读缓冲区的指定位置的元素
  * @param  Q: RingBuf_t结构体
            Index: 要读元素的索引号，0为最旧，(QSize - 1)为最新
            Data: 用于存放取出元素的字节数组的起始地址
  * @retval 无
  */
void RingBuf_Peek(RingBuf_t* Q, uint16_t Index, uint8_t* Data)
{
    memcpy(Data, &Q->Data[(Q->Rear + Index) % Q->QSize], Q->ESize);
}

// 解决半主机模式设置不同步问题
void _ttywrch(int ch)
{
    ch = ch;
}
