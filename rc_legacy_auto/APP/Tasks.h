#ifndef __TASKS_H
#define __TASKS_H

#include "Includes.h"

extern TaskHandle_t FlashTask_Handler;
extern TaskHandle_t BeepTask_Handler;
extern TaskHandle_t ReportTask_Handler;
extern TaskHandle_t MoveTask_Handler;

void FlashTask(void *pvParameters);
void BeepTask(void *pvParameters);
void ReportTask(void *pvParameters);
void MoveTask(void *pvParameters);

#endif
