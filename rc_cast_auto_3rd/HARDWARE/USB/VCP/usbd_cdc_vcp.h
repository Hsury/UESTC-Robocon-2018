#ifndef __USBD_CDC_VCP_H
#define __USBD_CDC_VCP_H

#include "usbd_cdc_core.h"
#include "usbd_conf.h"
#include "Includes.h"

#define VCP_TX_BUFFER_MAX_USAGE  (int)(0.7 * APP_RX_DATA_SIZE)
#define VCP_TX_BUFFER_SAFE_USAGE (int)(0.2 * APP_RX_DATA_SIZE)

extern CDC_IF_Prop_TypeDef VCP_fops;

extern uint8_t isUSBAvailable;
extern uint8_t isVCPAvailable;
extern uint32_t VCP_Tx_Buffer_Usage;

uint16_t VCP_DataTx (uint8_t* Buf, uint32_t Len);
uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len);

#endif
