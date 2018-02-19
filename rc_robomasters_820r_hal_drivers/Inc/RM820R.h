#ifndef __RM820R_H
#define __RM820R_H

typedef struct
{
    int16_t Current;
    uint16_t Angle;
    uint16_t Speed;
}
RM820R_TypeDef;

void RM820R_Init(CAN_HandleTypeDef* CAN, CanTxMsgTypeDef* TxMsg, CanRxMsgTypeDef* RxMsg);
void RM820R_SetCurrentAll(int16_t Current1, int16_t Current2, int16_t Current3, int16_t Current4);
void RM820R_SetCurrentSingle(uint8_t ID, int16_t Current);

#endif
