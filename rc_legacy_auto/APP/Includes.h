#ifndef __INCLUDES_H
#define __INCLUDES_H

// C Level
#include "stdbool.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

// System Level
#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

// Peripheral Level
#include "CAN.h"
#include "AirUART.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "usb_dcd_int.h"

// GPIO Level
#include "Buzzer.h"
#include "RGBLED.h"

// CAN Level
#include "Elmo.h"
#include "GyroEncoder.h"

// Application Level
#include "Algorithm.h"
#include "RingBuf.h"
#include "Protocol.h"
#include "Var.h"
#include "PID.h"
#include "Move.h"

#endif
