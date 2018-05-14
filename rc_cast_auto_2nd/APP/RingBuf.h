#ifndef __RINGBUF__H
#define __RINGBUF__H

#include "stdint.h"
#include "string.h"

typedef struct
{
    uint16_t Front;
    uint16_t Rear;
    uint16_t QSize;
    uint16_t ESize;
    uint8_t* Data;
}
RingBuf_t;

RingBuf_t RingBuf_Create(uint16_t QSize, uint16_t ESize);
void RingBuf_Wipe(RingBuf_t* Q);
uint16_t RingBuf_Usage(RingBuf_t* Q);
uint16_t RingBuf_Available(RingBuf_t* Q);
uint8_t RingBuf_isEmpty(RingBuf_t* Q);
uint8_t RingBuf_isFull(RingBuf_t* Q);
uint8_t RingBuf_Put(RingBuf_t* Q, uint8_t* Data);
void RingBuf_Overwrite(RingBuf_t* Q, uint8_t* Data);
uint8_t RingBuf_Get(RingBuf_t* Q, uint8_t* Data);
void RingBuf_Peek(RingBuf_t* Q, uint16_t Index, uint8_t* Data);

#endif
