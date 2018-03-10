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

                   ===== UESTC Raspberry Pi Executor For ABU Robocon 2018 =====
                              Copyright (c) 2018 HsuRY <i@hsury.com>

                                        VERSION 2018/03/09

*/

#include "Includes.h"

#define ENABLE_ACC_REPORT 0
#define ENABLE_VEL_REPORT 0
#define ENABLE_POS_REPORT 0

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

TaskHandle_t InitTask_Handler;
TaskHandle_t FlashTask_Handler;
TaskHandle_t BeepTask_Handler;
TaskHandle_t ReportTask_Handler;
TaskHandle_t MoveTask_Handler;

void InitTask(void *pvParameters);
void FlashTask(void *pvParameters);
void BeepTask(void *pvParameters);
void ReportTask(void *pvParameters);
void MoveTask(void *pvParameters);

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    delay_init(168);
    uart_init(921600);
    AirUART_Init(115200);
    Buzzer_Init();
    RGBLED_Init();
    Dual_CAN_Init();
    //USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
    xTaskCreate(InitTask, "Init Task", 128, NULL, 1, &InitTask_Handler);
    vTaskStartScheduler();
}

void InitTask(void *pvParameters)
{
    taskENTER_CRITICAL();
    // 任务加到这里
    xTaskCreate(FlashTask, "Flash Task", 128, NULL, 2, &FlashTask_Handler);
    xTaskCreate(BeepTask, "Beep Task", 128, NULL, 3, &BeepTask_Handler);
    xTaskCreate(ReportTask, "Report Task", 256, NULL, 4, &ReportTask_Handler);
    xTaskCreate(MoveTask, "Move Task", 256, NULL, 5, &MoveTask_Handler);
    vTaskDelete(InitTask_Handler);
    taskEXIT_CRITICAL();
}

void FlashTask(void *pvParameters)
{
    while (1)
    {
        RGBShowRed();
        delay_ms(333);
        RGBShowGreen();
        delay_ms(333);
        RGBShowBlue();
        delay_ms(333);
        //printf("Serial Test\r\n");
    }
}

void BeepTask(void *pvParameters)
{
    while (1)
    {
        BuzzerOn();
        delay_ms(250);
        BuzzerOff();
        delay_ms(250);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void ReportTask(void *pvParameters)
{
    while (1)
    {
        delay_ms(5);
        #if ENABLE_ACC_REPORT
        PackUp(0xA1, (uint8_t*)(&RealAccX));
        PackUp(0xA2, (uint8_t*)(&RealAccY));
        PackUp(0xA3, (uint8_t*)(&RealAccZ));
        #endif
        #if ENABLE_VEL_REPORT
        PackUp(0xA4, (uint8_t*)(&RealVelX));
        PackUp(0xA5, (uint8_t*)(&RealVelY));
        PackUp(0xA6, (uint8_t*)(&RealVelZ));
        #endif
        #if ENABLE_POS_REPORT
        PackUp(0xA7, (uint8_t*)(&PosX));
        PackUp(0xA8, (uint8_t*)(&PosY));
        PackUp(0xA9, (uint8_t*)(&AngZ));
        #endif
    }
}

void MoveTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Elmo_Init(CAN1, 9, 0);
        GyroEncoder_Reset();
        delay_ms(250);
        for (uint8_t i = 0; i < 3; i++)
        {
            BuzzerOn();
            delay_ms(250);
            BuzzerOff();
            delay_ms(750);
        }
        delay_ms(250);
        uint32_t Duration;
        Duration = UniformPlusP(0.9, -4.5, 0, 3000);
        xTaskNotifyGive(BeepTask_Handler);
        PackUp(0xB1, (uint8_t*)(&Duration));
        Duration = UniformPlusP(2.3, -4.5, 0, 3000);
        xTaskNotifyGive(BeepTask_Handler);
        PackUp(0xB1, (uint8_t*)(&Duration));
        delay_ms(250);
        Elmo_Close(0);
    }
}
