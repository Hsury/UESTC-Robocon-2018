#ifndef __ELMO_H
#define __ELMO_H

#define CAN4Elmo CAN1
#define Elmo_NUM 4                                                                                 // 挂载的Elmo数量

#if CAN4Elmo == CAN1
    #define CANTxMsg4Elmo CAN1TxMessage
    #define CANSend4Elmo  CAN1_Send
#elif CAN4Elmo == CAN2
    #define CANTxMsg4Elmo CAN2TxMessage
    #define CANSend4Elmo  CAN2_Send
#endif

typedef enum
{
    Unknown,
    Release,
    Standby,
    PVM
}
Elmo_Mode_TypeDef;

typedef struct
{
    u8 id;
    Elmo_Mode_TypeDef mode;
    u32 speed;
}
Elmo_TypeDef;
