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

                      ===== UESTC Cast Auto Robot For ABU Robocon 2018 =====
                              Copyright (c) 2018 HsuRY <i@hsury.com>

                                        VERSION 2018/04/30

*/

#include "Includes.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

TaskHandle_t InitTask_Handler;

void InitTask(void *pvParameters)
{
    taskENTER_CRITICAL();
    xTaskCreate(FlashTask, "Flash Task", 128, NULL, 1, &FlashTask_Handler);
    xTaskCreate(BeepTask, "Beep Task", 128, NULL, 2, &BeepTask_Handler);
    xTaskCreate(ReportTask, "Report Task", 256, NULL, 3, &ReportTask_Handler);
    xTaskCreate(WirelessTask, "Wireless Task", 256, NULL, 4, &WirelessTask_Handler);
    xTaskCreate(MoveTask, "Move Task", 512, NULL, 5, &MoveTask_Handler);
	xTaskCreate(FlowTask, "Flow Task", 256, NULL, 6, &FlowTask_Handler);
    taskEXIT_CRITICAL();
    
    printf("System started!\r\n");
    Printf(AirUART, "Channel: AirUART\r\n");
    //Printf(CH340, "Channel: CH340\r\n");
    Printf(ESP8266, "Channel: ESP8266\r\n");
    
    //ESP8266配置区
    //ESP8266_ExitTransLink();
    //ESP8266_UARTConfig(921600);
    //ESP8266_WiFiModeConfig(SOFTAP_STATION);
    //ESP8266_StationConfig("Robocon-WiFi", "duoguanyuenan8");
    //ESP8266_StationDHCPConfig(true);
    //ESP8266_StationIPConfig("192.168.1.200", "192.168.1.1", "255.255.255.0");
    //ESP8266_SoftAPConfig("HsuRY's STM32F407", "HSURY.COM", 5, WPA2_PSK, 2, false);
    //ESP8266_TransLinkConfig("192.168.1.255", 2018, UDP, 2018);
    //ESP8266_Reset();
    
    vTaskDelete(NULL);
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    delay_init(168);
    UART_Init(115200);
    AirUART_Init(115200);
    //CH340_Init(921600);
    ESP8266_Init(921600);
    Dual_CAN_Init();
    //USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
    Buzzer_Init();
    RGBLED_Init();
    Cylinder_Init();
	Sensor_Init();
    Elmo_Init(CAN1, 9, 0);
    Elmo_Close(0);
    xTaskCreate(InitTask, "Init Task", 128, NULL, 1, &InitTask_Handler);
    vTaskStartScheduler();
}
