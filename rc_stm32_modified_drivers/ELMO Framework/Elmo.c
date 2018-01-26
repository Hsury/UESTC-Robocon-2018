#include "Elmo.h"

Elmo_TypeDef Elmo[ELMO_NUM];

u8 Elmo_Init()
{
    u16 i = 0;
    for (i = 0; i < ELMO_NUM; i++)
    {
        Elmo[i]->id    = i + 1;
        Elmo[i]->mode  = Unknown;
        Elmo[i]->speed = 0;
        NMTCmd(i + 1, 0x82)                                                                        // NMT RESET COMMUNICATION = 0x82
    }
    RSDO(0, 0x1A00, 0);                                                                            // 禁用PDO1
	RSDO(0, 0x1A01, 0);                                                                            // 禁用PDO2 最后使用的是PDO2
    NMTCmd(0, 0x01);                                                                               // NMT ENTER OPERATIONAL = 0x01
    RPDO2_Cmd_Data(0, (u8 *)"MO", 0);                                                              // 广播关闭所有电机
    RPDO2_Cmd_Data(0, (u8 *)"PM", 0x01);                                                           // 未知功能
    RPDO2_Cmd_Data(0, (u8 *)"AC", 1000000);                                                        // 设置加速度
    RPDO2_Cmd_Data(0, (u8 *)"DC", 1000000);                                                        // 设置减速度
	RPDO2_Cmd_Data(0, (u8 *)"UM", 0x02);                                                           // 进入SCM速度控制模式
    RPDO2_Cmd_Data(0, (u8 *)"MO", 1);                                                              // 广播关闭所有电机
    for (i = 0; i < ELMO_NUM; i++)
    {
        Elmo[i]->mode = Standby;
    }

}

u8 Elmo_PVM(u8 ElmoID, s32 Speed)
{
    if (ElmoID >= 1 & ElmoID <= ELMO_NUM)
    {
        if (Elmo[ElmoID - 1]->mode == Standby | Elmo[ElmoID - 1]->mode == PVM)
        {
            RPDO2_Cmd_Data(ElmoID, (u8 *)"JV", speed);
            RPDO2_Cmd_String(ElmoID, (u8 *)"BG");
            return 0;
        }
        else if (Elmo[ElmoID - 1]->mode == Release)
        {
            RPDO2_Cmd_Data(0, (u8 *)"MO", 1);
            RPDO2_Cmd_Data(ElmoID, (u8 *)"JV", speed);
            RPDO2_Cmd_String(ElmoID, (u8 *)"BG");
            return 0;
        }
        else
        {
            printf("This Elmo may have not been initialized!");
            return 1;
        }
    }
    else return 1;
}

void NMTCmd(u8 ElmoID, u8 NMTCmd)
{
    CANTxMsg4Elmo.COBID = 0x00;                                                                    // NMT Service COBID = 0x00
    CANTxMsg4Elmo.DLC   = 2;
    CANTxMsg4Elmo.DATA[0] = NMTCmd;
    CANTxMsg4Elmo.DATA[1] = ElmoID;
    CANSend4Elmo();
}

void RSDO(u8 ElmoID, u16 Index, u32 Data)
{
    CANTxMsg4Elmo.COBID = 0x600 + ElmoID;                                                          // RSDO COBID = 0x600
    CANTxMsg4Elmo.DLC   = 8;
    CANTxMsg4Elmo.DATA[0] = 0x22;
    CANTxMsg4Elmo.DATA[1] = (Index & 0xFF);
    CANTxMsg4Elmo.DATA[2] = (Index & 0xFF00) >> 8;
    CANTxMsg4Elmo.DATA[3] = 0x00;                                                                  // SubIndex = 0x00
    CANTxMsg4Elmo.DATA[4] = (Data & 0xFF);
    CANTxMsg4Elmo.DATA[5] = (Data & 0xFF00) >> 8;
    CANTxMsg4Elmo.DATA[6] = (Data & 0xFF0000) >> 16;
    CANTxMsg4Elmo.DATA[7] = (Data & 0xFF000000) >> 24;
    CANSend4Elmo();
}

void RPDO2_Cmd_Data(u8 ElmoID, u8* Cmd, u32 Data)
{
    CANTxMsg4Elmo.COBID = 0x300 + ElmoID;                                                          // RPDO2 COBID = 0x300
    CANTxMsg4Elmo.DLC   = 8;
    CANTxMsg4Elmo.DATA[0] = (*Cmd++);
    CANTxMsg4Elmo.DATA[1] = (*Cmd);
    CANTxMsg4Elmo.DATA[2] = 0x00;
    CANTxMsg4Elmo.DATA[3] = 0x00;
    CANTxMsg4Elmo.DATA[4] = (Data & 0xFF);
    CANTxMsg4Elmo.DATA[5] = (Data & 0xFF00) >> 8;
    CANTxMsg4Elmo.DATA[6] = (Data & 0xFF0000) >> 16;
    CANTxMsg4Elmo.DATA[7] = (Data & 0xFF000000) >> 24;
    CANSend4Elmo();
}

void RPDO2_Cmd_String(u8 ElmoID, u8* Cmd)
{
    CANTxMsg4Elmo.COBID = 0x300 + ElmoID;                                                          // RPDO2 COBID = 0x300
    CANTxMsg4Elmo.DLC   = 4;
    CANTxMsg4Elmo.DATA[0] = (*Cmd++);
    CANTxMsg4Elmo.DATA[1] = (*Cmd);
    CANTxMsg4Elmo.DATA[2] = 0x00;
    CANTxMsg4Elmo.DATA[3] = 0x00;
    CANSend4Elmo();
}