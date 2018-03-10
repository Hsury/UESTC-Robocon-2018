#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "Includes.h"

uint8_t Checkout(RingBuf_t* Q);
void PackUp(uint8_t Addr, uint8_t* Data);

#endif
