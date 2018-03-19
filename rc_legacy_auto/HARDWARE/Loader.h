#ifndef __LOADER_H
#define __LOADER_H

#include "Includes.h"

void Loader_Init(void);
void Loader_Release(void);
void Loader_Shift(uint8_t PosID);
void Loader_Sweep(void);
void Loader_QueryPos(void);

#endif
