#include "usbd_usr.h"
#include "usbd_ioreq.h"
#include "Includes.h"

USBD_Usr_cb_TypeDef USR_cb =
{
    USBD_USR_Init,
    USBD_USR_DeviceReset,
    USBD_USR_DeviceConfigured,
    USBD_USR_DeviceSuspended,
    USBD_USR_DeviceResumed,
    USBD_USR_DeviceConnected,
    USBD_USR_DeviceDisconnected,
};

/**
* @brief  USBD_USR_Init 
*         Displays the message on LCD for host lib initialization
* @param  None
* @retval None
*/
void USBD_USR_Init(void)
{
    #ifdef USE_USB_OTG_HS
    printf("High Speed USB VCP Device Initialized!\r\n");
    #else
    printf("Full Speed USB VCP Device Initialized!\r\n");
    #endif
}

/**
* @brief  USBD_USR_DeviceReset 
*         Displays the message on LCD on device Reset Event
* @param  speed : device speed
* @retval None
*/
void USBD_USR_DeviceReset(uint8_t speed)
{
    switch (speed)
    {
        case USB_OTG_SPEED_HIGH:
            printf("> Mode: High Speed\r\n");
        break;
        
        case USB_OTG_SPEED_FULL:
            printf("> Mode: Full Speed\r\n" );
        break;
        
        default:
            printf("> Mode: Unknown\r\n" );
        break;
    }
}

/**
* @brief  USBD_USR_DeviceConfigured
*         Displays the message on LCD on device configuration Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceConfigured(void)
{
    printf("> VCP Interface configured.\r\n");
    isUSBAvailable = 1;
    BaseType_t pxHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(BeepTask_Handler, &pxHigherPriorityTaskWoken);
    if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
}

/**
* @brief  USBD_USR_DeviceSuspended 
*         Displays the message on LCD on device suspend Event
* @param  None
* @retval None
*/
void USBD_USR_DeviceSuspended(void)
{
    printf("> USB Device in Suspend Mode.\r\n");
    isUSBAvailable = 0;
    isVCPAvailable = 0;
    BaseType_t pxHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(BeepTask_Handler, &pxHigherPriorityTaskWoken);
    if (pxHigherPriorityTaskWoken != pdFALSE) taskYIELD();
    /* Users can do their application actions here for the USB-Reset */
}

/**
* @brief  USBD_USR_DeviceResumed 
*         Displays the message on LCD on device resume Event
* @param  None
* @retval None
*/
void USBD_USR_DeviceResumed(void)
{
    printf("> USB Device in Idle Mode.\r\n");
    /* Users can do their application actions here for the USB-Reset */
}

/**
* @brief  USBD_USR_DeviceConnected
*         Displays the message on LCD on device connection Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceConnected(void)
{
    printf("> USB Device Connected.\r\n");
}

/**
* @brief  USBD_USR_DeviceDisonnected
*         Displays the message on LCD on device disconnection Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceDisconnected(void)
{
    printf("> USB Device Disconnected.\r\n");
}
