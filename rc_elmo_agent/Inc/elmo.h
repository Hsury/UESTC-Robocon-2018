/*
********************************************************************************
	*@  file: elmo.h
	*@  author: alone
	*@  data: 9/27/2016
	*@  version: v2.0
	*@  brief: �����޸ģ���Ҫ�Ҷ�������
	*...............................................................................
	*@
	*@ Notes: (1) �޸�ElMO_NUM��ֵȷ��Elmo����,����Ϊ4��Elmo.
	*@            ��������Elmo,�ڼӺź������.
	*@        (2) �޸ĵ������,�����,��ֵ����,������ת��.
	*@            ��Щ�ǵ��̵ĵ���Ĳ���.����ʹ������Elmo,��ֱ�ӵ��ú���.
	*@            ��������������������Ϊ����5��ID��.
	*@        (3) ***********************************************************
	*@            *  MAXON����ͺ�       �ת��(RPM)  �����(A) ����(W) *
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

/* 0:���ٶȿ���ģʽ�������ٶ�ģʽ,��ʱ���ٶ�ʧЧ */ 
/* 1:��λ�ÿ���ģʽ�������ٶ�ģʽ,��ʱ���ٶ���Ч */ 
#define  JV_IN_PCM          0


#define ELMO_NUM    (2)             //Elmo����,�����ϸ��չ��ظ�����ELMO���ã����ö����ã�����

#define COBID_RSDO 0x600
#define COBID_NMT_SERVICE   0x000


#define TYPE_INTEGER      0
#define TYPE_FLOAT        1

/* Binary Interpreter Command */
#define UM_IDLE             0x00
#define UM_UNC              0x06                      // ����ȷ��ģʽ��������ȫ���������

#define MO_OFF            0
#define MO_ON             1


#if JV_IN_PCM
   #define UM_SCM           0x05                      // Position control mode��
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
	u8 NodeID;	//�ڵ�ID
	u8 CurOPMode;   //��ǰ����ģʽ
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
	uint8_t u08[8];  /* �����ֽ������������� */
	uint16_t u16[4];
	uint32_t u32[2];
}T_CanData;
	
// ��Ϊ��������������������ݽṹ������ͬ�����Զ������ݿ�ֱ��ʹ��
typedef struct 
{
	uint32_t IxR;	/* ��TIxR/RIxR��Ӧ */                      //TIR
	uint32_t 	DTxR;	/* ��TDTxR/RDTxR��Ӧ */                    //TDTR
	T_CanData 	Data; 	/* TLxR:THxR/RLxR:RHxR, ���ֽ����� */      //TDLR,TDTR
	
}T_CanFrame;

u32 Elmo_Init(void);
u8 Elmo_PVM(u8 elmoID, s32 speed);
u8 Elmo_SetAcc(u8 elmoID, u32 acc);
u8 Elmo_SetDec(u8 elmoID, u32 dec);
u8 Elmo_Break(u8 elmoID);
u8 Elmo_Release(u8 elmoID);

#endif
