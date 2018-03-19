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

                     ===== UESTC Legacy Auto Robot For ABU Robocon 2018 =====
                              Copyright (c) 2018 HsuRY <i@hsury.com>

                                        VERSION 2018/03/17

*/

#include "Includes.h"

#define ENABLE_ACC_REPORT 0
#define ENABLE_VEL_REPORT 0
#define ENABLE_POS_REPORT 0

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

TaskHandle_t InitTask_Handler;

void InitTask(void *pvParameters)
{
    taskENTER_CRITICAL();
    // 任务加到这里
    xTaskCreate(FlashTask, "Flash Task", 128, NULL, 2, &FlashTask_Handler);
    xTaskCreate(BeepTask, "Beep Task", 128, NULL, 3, &BeepTask_Handler);
    xTaskCreate(ReportTask, "Report Task", 256, NULL, 4, &ReportTask_Handler);
    xTaskCreate(MoveTask, "Move Task", 256, NULL, 5, &MoveTask_Handler);
    xTaskCreate(LoadTask, "Load Task", 256, NULL, 6, &LoadTask_Handler);
    vTaskDelete(InitTask_Handler);
    taskEXIT_CRITICAL();
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    delay_init(168);
    UART_Init(115200);
    AirUART_Init(115200);
    CH340_Init(921600);
    Dual_CAN_Init();
    //USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
    Buzzer_Init();
    RGBLED_Init();
    Cylinder_Init();
    GY53_PWM_Init();
    Elmo_Init(CAN1, 9, 0);
    Elmo_Close(0);
    printf("System started!\r\n");
    xTaskCreate(InitTask, "Init Task", 128, NULL, 1, &InitTask_Handler);
    vTaskStartScheduler();
}
