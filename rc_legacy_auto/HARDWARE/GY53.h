#ifndef __GY53_H
#define __GY53_H

#include "Includes.h"

void GY53_PWM_Init(void);
bool GY53_Relocate(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);

#endif
