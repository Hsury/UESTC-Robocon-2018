#ifndef __CRADLE_H
#define __CRADLE_H

#include "Includes.h"

void Cradle_ArriveNotify(uint8_t TZx);
void Cradle_RestartNotify(uint8_t TZx);
void Cradle_RetryNotify(uint8_t Code);
void Cradle_ReturnNotify(void);

#endif
