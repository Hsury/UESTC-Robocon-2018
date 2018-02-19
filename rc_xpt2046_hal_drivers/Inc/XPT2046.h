#ifndef __XPT2046_H
#define __XPT2046_H

#include "gpio.h"

#define DEBUG 0

#define XPT2046_MIN_X 1300
#define XPT2046_MIN_Y 1300
#define XPT2046_MAX_X 3000
#define XPT2046_MAX_Y 3000

#define XPT2046_MAP_X 350
#define XPT2046_MAP_Y 200

#define XPT2046_TRIM_NUM 1
#define XPT2046_WEIGHT_AVG 0.8
#define XPT2046_WEIGHT_MID 0.2

#define XPT2046_CMD_GET_X 0xD0
#define XPT2046_CMD_GET_Y 0x90

typedef struct
{
	SPI_HandleTypeDef* SPI;
	GPIO_TypeDef* CS_Port;
	uint16_t CS_Pin;
	uint16_t RawX;
	uint16_t RawY;
	uint16_t FilterX;
	uint16_t FilterY;
	uint16_t X;
	uint16_t Y;
}
XPT2046_TypeDef;

void XPT2046_Init(XPT2046_TypeDef* XPT2046, SPI_HandleTypeDef* SPI, GPIO_TypeDef* CS_Port, uint16_t CS_Pin);
uint8_t XPT2046_GetPoint(XPT2046_TypeDef* XPT2046, uint16_t SampleTimes, uint16_t Timeout);
uint16_t XPT2046_GetRawX(XPT2046_TypeDef* XPT2046);
uint16_t XPT2046_GetRawY(XPT2046_TypeDef* XPT2046);

void XPT2046_Send(SPI_HandleTypeDef* SPI, uint8_t Command);
void XPT2046_Recv(SPI_HandleTypeDef* SPI, uint16_t* Data);

void QSort(uint16_t* Array, uint16_t Left, uint16_t Right);

#endif
