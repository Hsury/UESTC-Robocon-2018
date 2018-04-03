#ifndef __TASKS_H
#define __TASKS_H

#include "Includes.h"

enum FLOW_CONTROL_CMD
{
    GET_READY = 1, 
    LAUNCH = 2
};

extern TaskHandle_t FlashTask_Handler;
extern TaskHandle_t BeepTask_Handler;
extern TaskHandle_t ReportTask_Handler;
extern TaskHandle_t MoveTask_Handler;
extern TaskHandle_t WirelessTask_Handler;
extern TaskHandle_t FlowTask_Handler;

void FlashTask(void *pvParameters);
void BeepTask(void *pvParameters);
void ReportTask(void *pvParameters);
void MoveTask(void *pvParameters);
void WirelessTask(void *pvParameters);
void FlowTask(void *pvParameters);

#endif
