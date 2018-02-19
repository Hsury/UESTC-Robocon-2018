#include "XPT2046.h"

void XPT2046_Init(XPT2046_TypeDef* XPT2046, SPI_HandleTypeDef* SPI, GPIO_TypeDef* CS_Port, uint16_t CS_Pin)
{
	XPT2046->SPI = SPI;
	XPT2046->CS_Port = CS_Port;
	XPT2046->CS_Pin = CS_Pin;	
}

uint8_t XPT2046_GetPoint(XPT2046_TypeDef* XPT2046, uint16_t SampleTimes, uint16_t Timeout)
{
	uint16_t Index = 0;
	uint16_t Tick = 0;
	uint16_t XQueue[SampleTimes];
	uint16_t YQueue[SampleTimes];
	uint64_t XSum = 0;
	uint64_t YSum = 0;
	uint32_t ExpireTime = HAL_GetTick() + Timeout;
	#if DEBUG
	uint32_t TimeStamp = HAL_GetTick();
	printf("\r\n");
	printf("XPT2046 Get Point Task Started.\r\n");
	printf("Current Time(ms): %u\r\n", TimeStamp);
	printf("Expire Time(ms): %u\r\n", ExpireTime);
	printf("------- STEP 1: Acquire Data -------\r\n");
	printf("  ID  |   X   |   Y   \r\n");
	printf("------+-------+-------\r\n");
	TimeStamp = HAL_GetTick();
	#endif
	while (HAL_GetTick() < ExpireTime && Index < SampleTimes)
	{
		XPT2046_GetRawX(XPT2046);
		Tick = 0;
		while (Tick < 1E4) Tick++;
		if (XPT2046->RawX < XPT2046_MIN_X || XPT2046->RawX > XPT2046_MAX_X) continue;
		XPT2046_GetRawY(XPT2046);
		Tick = 0;
		while (Tick < 1E4) Tick++;
		if (XPT2046->RawY < XPT2046_MIN_Y || XPT2046->RawY > XPT2046_MAX_Y) continue;
		XQueue[Index] = XPT2046->RawX;
		YQueue[Index] = XPT2046->RawY;
		Index++;
		#if DEBUG
		printf("%-6u|%-7u|%-7u\r\n", Index, XPT2046->RawX, XPT2046->RawY);
		#endif
	}
	#if DEBUG
	printf("Valid Samples: %u\r\n", Index);
	printf("Time Ellapse(ms): %u\r\n", HAL_GetTick() - TimeStamp);
	printf("-------- STEP 2: QSort Data --------\r\n");
	printf("  ID  |   X   |   Y   \r\n");
	printf("------+-------+-------\r\n");
	TimeStamp = HAL_GetTick();
	#endif
	if (Index)
	{
		QSort(XQueue, 0, Index - 1);
		QSort(YQueue, 0, Index - 1);
	}
	#if DEBUG
	for (uint16_t i = 0; i < Index; i++) printf("%-6u|%-7u|%-7u\r\n", i + 1, XQueue[i], YQueue[i]);
	printf("Time Ellapse(ms): %u\r\n", HAL_GetTick() - TimeStamp);
	printf("------- STEP 3: Data Process -------\r\n");
	printf(" Type |   X   |   Y   \r\n");
	printf("------+-------+-------\r\n");
	TimeStamp = HAL_GetTick();
	#endif
	if (Index > 2 * XPT2046_TRIM_NUM)
	{
		for (uint16_t i = XPT2046_TRIM_NUM; i < Index - XPT2046_TRIM_NUM; i++)
		{
			XSum += XQueue[i];
			YSum += YQueue[i];
		}
		XPT2046->FilterX = XPT2046_WEIGHT_AVG * (XSum / (Index - 2 * XPT2046_TRIM_NUM)) + XPT2046_WEIGHT_MID * XQueue[Index / 2];
		XPT2046->FilterY = XPT2046_WEIGHT_AVG * (YSum / (Index - 2 * XPT2046_TRIM_NUM)) + XPT2046_WEIGHT_MID * YQueue[Index / 2];
		XPT2046->X = XPT2046_MAP_X * (XPT2046->FilterX - XPT2046_MIN_X) / (XPT2046_MAX_X - XPT2046_MIN_X);
		XPT2046->Y = XPT2046_MAP_Y * (XPT2046->FilterY - XPT2046_MIN_Y) / (XPT2046_MAX_Y - XPT2046_MIN_Y);		
	}
	else
	{
		XPT2046->FilterX = 0;
		XPT2046->FilterY = 0;
		XPT2046->X = 0;
		XPT2046->Y = 0;
	}
	#if DEBUG
	printf("Max.  |%-7u|%-7u\r\n", XQueue[0], YQueue[0]);
	printf("Min.  |%-7u|%-7u\r\n", XQueue[Index - 1], YQueue[Index - 1]);
	printf("Mid.  |%-7u|%-7u\r\n", XQueue[Index / 2], YQueue[Index / 2]);
	printf("Avg.  |%-7u|%-7u\r\n", (Index - 2 * XPT2046_TRIM_NUM) ? (uint16_t)(XSum / (Index - 2 * XPT2046_TRIM_NUM)): 0, \
	                               (Index - 2 * XPT2046_TRIM_NUM) ? (uint16_t)(YSum / (Index - 2 * XPT2046_TRIM_NUM)): 0);
	printf("Flt.  |%-7u|%-7u\r\n", XPT2046->FilterX, XPT2046->FilterY);
	printf("Fin.  |%-7u|%-7u\r\n", XPT2046->X, XPT2046->Y);
	printf("Time Ellapse(ms): %u\r\n", HAL_GetTick() - TimeStamp);
	printf("XPT2046 Get Point Task Finished.\r\n");
	printf("Current Time(ms): %u\r\n", HAL_GetTick());
	printf("Cycle Time(ms): %u\r\n", HAL_GetTick() - ExpireTime + Timeout);
	printf("\r\n");
	#endif
	return (255 * Index / SampleTimes);
}
uint16_t XPT2046_GetRawX(XPT2046_TypeDef* XPT2046)
{
	HAL_GPIO_WritePin(XPT2046->CS_Port, XPT2046->CS_Pin, GPIO_PIN_RESET);
	XPT2046_Send(XPT2046->SPI, XPT2046_CMD_GET_X);
	XPT2046_Recv(XPT2046->SPI, &XPT2046->RawX);
	HAL_GPIO_WritePin(XPT2046->CS_Port, XPT2046->CS_Pin, GPIO_PIN_SET);
	return XPT2046->RawX;
}

uint16_t XPT2046_GetRawY(XPT2046_TypeDef* XPT2046)
{
	HAL_GPIO_WritePin(XPT2046->CS_Port, XPT2046->CS_Pin, GPIO_PIN_RESET);
	XPT2046_Send(XPT2046->SPI, XPT2046_CMD_GET_Y);
	XPT2046_Recv(XPT2046->SPI, &XPT2046->RawY);
	HAL_GPIO_WritePin(XPT2046->CS_Port, XPT2046->CS_Pin, GPIO_PIN_SET);
	return XPT2046->RawY;
}

void XPT2046_Send(SPI_HandleTypeDef* SPI, uint8_t Command)
{
	HAL_SPI_Transmit(SPI, &Command, sizeof(Command), 1000);
}

void XPT2046_Recv(SPI_HandleTypeDef* SPI, uint16_t* Data)
{
	uint8_t Buffer[2];
	HAL_SPI_Receive(SPI, Buffer, sizeof(Buffer), 1000);
	*Data = (Buffer[0] << 5) + (Buffer[1] >> 3);
}

void QSort(uint16_t* Array, uint16_t Left, uint16_t Right)
{
	if (Left >= Right) return;
	uint16_t i = Left;
	uint16_t j = Right;
	uint16_t k = Array[Left];
	while (i < j)
	{
		while (i < j && k >= Array[j]) j--;
		Array[i] = Array[j];
		while (i < j && k <= Array[i]) i++;
		Array[j] = Array[i];
	}
	Array[i] = k;
	if (i) QSort(Array, Left, i - 1);
	QSort(Array, i + 1, Right);
}
