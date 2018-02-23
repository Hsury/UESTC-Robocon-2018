#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#pragma data_alignment = 4
#endif

#include "usbd_cdc_vcp.h"
#include "usb_conf.h"
#include "usart.h"

uint32_t VCP_Tx_Buffer_Usage = 0;

/* These are external variables imported from CDC core to be used for IN 
   transfer management. */
extern uint8_t APP_Rx_Buffer[]; /* Write CDC received data in this buffer.
                                   These data will be sent over USB IN endpoint
                                   in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;  /* Increment this pointer or roll it back to
                                   start address when writing received data
                                   in the buffer APP_Rx_Buffer. */
extern uint32_t APP_Rx_ptr_out;

/* Private function prototypes -----------------------------------------------*/
static uint16_t VCP_Init (void);
static uint16_t VCP_DeInit (void);
static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len);

CDC_IF_Prop_TypeDef VCP_fops = 
{
    VCP_Init,
    VCP_DeInit,
    VCP_Ctrl,
    VCP_DataTx,
    VCP_DataRx
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  VCP_Init
  *         Initializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Init(void)
{
    return USBD_OK;
}

/**
  * @brief  VCP_DeInit
  *         DeInitializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_DeInit(void)
{
    return USBD_OK;
}

/**
  * @brief  VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
    switch (Cmd)
    {
        case SEND_ENCAPSULATED_COMMAND:
            break;
        
        case GET_ENCAPSULATED_RESPONSE:
            break;
        
        case SET_COMM_FEATURE:
            break;
        
        case GET_COMM_FEATURE:
            break;
        
        case CLEAR_COMM_FEATURE:
            break;
        
        case SET_LINE_CODING:
            break;
        
        case GET_LINE_CODING:
            break;
        
        case SET_CONTROL_LINE_STATE:
            break;
        
        case SEND_BREAK:
            break;
        
        default:
            break;
    }
    return USBD_OK;
}

/**
  * @brief  VCP_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in 
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
uint16_t VCP_DataTx (uint8_t* Buf, uint32_t Len)
{
    for (uint32_t i = 0; i < Len; i++)
    {
        while (1)
        {
            VCP_Tx_Buffer_Usage = (APP_Rx_ptr_out > APP_Rx_ptr_in) ?
                                  (APP_RX_DATA_SIZE - APP_Rx_ptr_out) : (APP_Rx_ptr_in - APP_Rx_ptr_out);
            if (VCP_Tx_Buffer_Usage <= VCP_TX_BUFFER_MAX_USAGE) break; //缓冲区到达警戒线
            else
            {
                printf("> VCP Tx Buffer Alert, Usage: %u\r\n", VCP_Tx_Buffer_Usage);
                while (1)
                {
                    VCP_Tx_Buffer_Usage = (APP_Rx_ptr_out > APP_Rx_ptr_in) ?
                                          (APP_RX_DATA_SIZE - APP_Rx_ptr_out) : (APP_Rx_ptr_in - APP_Rx_ptr_out);
                    if (VCP_Tx_Buffer_Usage <= VCP_TX_BUFFER_SAFE_USAGE) break; //等待缓冲区泄压到安全线
                }
            }
        }
        APP_Rx_Buffer[APP_Rx_ptr_in] = *(Buf + i);
        APP_Rx_ptr_in++;
        if (APP_Rx_ptr_in == APP_RX_DATA_SIZE) APP_Rx_ptr_in = 0;
    }
    return USBD_OK;
}

/**
  * @brief  VCP_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
    for (uint32_t i = 0; i < Len; i++)
    {
        USART_SendData(USART1, *(Buf + i));
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
    return USBD_OK;
}
