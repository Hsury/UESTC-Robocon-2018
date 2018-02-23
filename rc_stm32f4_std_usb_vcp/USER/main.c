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

                            ===== UESTC Robot For ABU Robocon 2018 =====
                               Copyright (c) 2018 HsuRY <i@hsury.com>
*/

#include "usart.h"
#include "delay.h"
#include "led.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usb_dcd_int.h"

uint8_t isVCPAvailable = 0;

//以下代码用于USB高速+DMA模式下告知编译器内存对齐处理
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
    #if defined ( __ICCARM__ ) /*!< IAR Compiler */
        #pragma data_alignment = 4
    #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init(168);
    uart_init(921600);
    printf("\r\n");
    LEDInit();
    LED0 = 0;
    USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
              USB_OTG_HS_CORE_ID,
#else
              USB_OTG_FS_CORE_ID,
#endif  
              &USR_desc,
              &USBD_CDC_cb,
              &USR_cb);
    while (!isVCPAvailable);
    LED0 = 1;
    while (1)
    {
        LED2 = !LED2;
        VCP_DataTx("0123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ 你好，世界！\r\n", 58);
        //delay_ms(100);
    }
} 

//#ifdef USE_FULL_ASSERT
///**
//* @brief  assert_failed
//*         Reports the name of the source file and the source line number
//*         where the assert_param error has occurred.
//* @param  File: pointer to the source file name
//* @param  Line: assert_param error line source number
//* @retval None
//*/
//void assert_failed(uint8_t* file, uint32_t line)
//{
//  /* User can add his own implementation to report the file name and line number,
//  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  
//  /* Infinite loop */
//  while (1)
//  {}
//}
//#endif
