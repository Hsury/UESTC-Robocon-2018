/*
********************************************************************************
	*@  file: elmo.h
	*@  author: alone
	*@  data: 9/27/2016
	*@  version: v2.0
	*@  brief: 参数修改，不要乱动！！！
	*...............................................................................
	*@
	*@ Notes: (1) 修改ElMO_NUM的值确定Elmo个数,底盘为4个Elmo.
	*@            如需其他Elmo,在加号后面加上.
	*@        (2) 修改电机参数,额定电流,峰值电流,电机最大转速.
	*@            这些是底盘的电机的参数.如需使用其他Elmo,可直接调用函数.
	*@            尽量将除底盘外电机设置为大于5的ID号.
	*@        (3) ***********************************************************
	*@            *  MAXON电机型号       额定转速(RPM)  额定电流(A) 功率(W) *
	*@            *  EC-4pole-30-305013  16200          9.21        200     *
	*@            *  RE-40-148867        6930           5.77        150     *
	*@            *  RE-30-310007        8050           3.44        60      *
	*@            *  RE-25-339152        9620           1.42        20      *
	*@            ***********************************************************
	*@
	               
********************************************************************************
*/
#ifndef __ELMO_H
#define __ELMO_H
#include "stm32f4xx.h"
#include "math.h"
#include "string.h"
typedef uint32_t u32 ;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

/* 0:在速度控制模式下启动速度模式,此时加速度失效 */ 
/* 1:在位置控制模式下启动速度模式,此时加速度生效 */ 
#define  JV_IN_PCM          0


#define ELMO_NUM    (2)             //Elmo个数,必须严格按照挂载个数的ELMO配置，不得多配置！！！

#define COBID_RSDO 0x600
#define COBID_NMT_SERVICE   0x000


#define TYPE_INTEGER      0
#define TYPE_FLOAT        1

/* Binary Interpreter Command */
#define UM_IDLE             0x00
#define UM_UNC              0x06                      // 不能确定模式，适用于全部电机调用

#define MO_OFF            0
#define MO_ON             1


#if JV_IN_PCM
   #define UM_SCM           0x05                      // Position control mode，
#else
   #define UM_SCM           0x02                      // Speed control mode
#endif


#define COBID_RPDO2         0x300

/* NMT Command Specifier */
#define NMT_ENTER_OPERATIONAL      0x01
#define NMT_RESET_COMMUNICATION    0x82

#define globalCAN_ID_MASTER_CONTROL     (0x15)

typedef struct __ELMO
{
	u8 NodeID;	//节点ID
	u8 CurOPMode;   //当前运行模式
	int16_t currentActual;
	int32_t positionActual;
	int32_t velocityActual;
	
}Elmo;

typedef struct 
{
	Elmo ElmoBase;
	u8 Start;
	u8 End;

}ElmoGroupTypedef;

typedef struct
{
	uint8_t u08[8];  /* 定义字节型以满足需求 */
	uint16_t u16[4];
	uint32_t u32[2];
}T_CanData;
	
// 因为发送邮箱与接收邮箱数据结构基本相同，所以队列数据可直接使用
typedef struct 
{
	uint32_t IxR;	/* 与TIxR/RIxR对应 */                      //TIR
	uint32_t 	DTxR;	/* 与TDTxR/RDTxR对应 */                    //TDTR
	T_CanData 	Data; 	/* TLxR:THxR/RLxR:RHxR, 八字节数据 */      //TDLR,TDTR
	
}T_CanFrame;

u32 Elmo_Init(void);
u8 Elmo_PVM(u8 elmoID, s32 speed);
u8 Elmo_SetAcc(u8 elmoID, u32 acc);
u8 Elmo_SetDec(u8 elmoID, u32 dec);
u8 Elmo_Break(u8 elmoID);
u8 Elmo_Release(u8 elmoID);

#endif
