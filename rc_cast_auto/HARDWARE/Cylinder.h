#ifndef __CYLINDER_H
#define __CYLINDER_H

#include "Includes.h"

void Cylinder_Init(void);
void Cylinder_On(uint8_t ID);
void Cylinder_Off(uint8_t ID);
void Pin(void);
void PinFromISR(void);
void Unpin(void);
void UnpinFromISR(void);
void DropCamera(void);
void TakeCamera(void);

#endif
