#include "RC_CAN.h"

void CAN_Init(CAN_HandleTypeDef* CAN, CanTxMsgTypeDef* TxMsg, CanRxMsgTypeDef* RxMsg)
{
	CAN->pTxMsg = TxMsg;
	CAN->pRxMsg = RxMsg;
	CAN_FilterConfTypeDef CANFilterConf;
	CANFilterConf.FilterIdHigh = 0x0000;
	CANFilterConf.FilterIdLow = 0x0000;
	CANFilterConf.FilterMaskIdHigh = 0x0000;
	CANFilterConf.FilterMaskIdLow = 0x0000;
	CANFilterConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CANFilterConf.FilterNumber = 0;
	CANFilterConf.FilterMode = CAN_FILTERMODE_IDMASK;
	CANFilterConf.FilterScale = CAN_FILTERSCALE_32BIT;
	CANFilterConf.FilterActivation = ENABLE;
	CANFilterConf.BankNumber = 14;
	HAL_CAN_ConfigFilter(CAN, &CANFilterConf);
	HAL_CAN_Receive_IT(CAN, CAN_FIFO0);
}

uint8_t CAN_Send(CAN_HandleTypeDef* CAN, uint8_t* Data, uint8_t Length)
{
	CAN->pTxMsg->StdId = DEVICE_CAN_ID;
	CAN->pTxMsg->IDE = CAN_ID_STD;
	CAN->pTxMsg->RTR = CAN_RTR_DATA;
	CAN->pTxMsg->DLC = Length;
	memcpy(CAN->pTxMsg->Data, Data, Length);
	if (HAL_CAN_Transmit(CAN, 10) == HAL_OK) return 0;
	else return 1;
}

extern RM820R_TypeDef RM820R[4];

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CAN)
{
	uint32_t FrameID = CAN->pRxMsg->StdId;
	if (CAN == &hcan && FrameID >= 0x201 && FrameID <= 0x204 && CAN->pRxMsg->DLC == 8)
	{
        memcpy(&RM820R[FrameID - 0x200].Angle, &CAN->pRxMsg->Data[0], 2);
        memcpy(&RM820R[FrameID - 0x200].Speed, &CAN->pRxMsg->Data[2], 2);
		printf("ID:%-5uAngle:%-10uSpeed:%-10u\r\n", FrameID - 0x200, RM820R[FrameID - 0x200].Angle, RM820R[FrameID - 0x200].Speed);
	}
	HAL_CAN_Receive_IT(CAN, CAN_FIFO0);
}
