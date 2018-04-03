#ifndef __SENSOR_H
#define __SENSOR_H

#include "Includes.h"

extern SemaphoreHandle_t SwitchSemaphore;
extern SemaphoreHandle_t DimmerSemaphore;

void Sensor_Init(void);

#endif
