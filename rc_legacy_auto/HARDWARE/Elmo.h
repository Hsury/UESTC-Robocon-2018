#ifndef __ELMO_H
#define __ELMO_H
#include "Includes.h"

/*******************************************************************************
* �궨�� 
*******************************************************************************/
/* 0:���ٶȿ���ģʽ�������ٶ�ģʽ,��ʱ���ٶ�ʧЧ */ 
/* 1:��λ�ÿ���ģʽ�������ٶ�ģʽ,��ʱ���ٶ���Ч */ 
#define  JV_IN_PCM          0


/* ѭ�����в��� */
#define ELMO_NUM            3                               // Elmo����,�����ϸ��չ��ظ�����ELMO���ã����ö����ã�����
#define CAN_BUF_NUM         150                             //����ָ������  �ĳ�200�� ԭ����1000  �ּ�С����150
#define CAN_ID_DELAY        0x129                           //��ʱָ��

/* ELMO��ز��� */
#define RATE_CURRENT        9.210                           // �����(A)
#define PEAK_CURRENT        (9.210*2)                       // ��ֵ����(A)
#define MAX_VOLOCITY        16200                           // ������ת��(rpm)

/* ����������ز��� */
#define RATE_CUR            RATE_CURRENT                    // �����(A)          CL[1]
#define MAX_CURRENT         PEAK_CURRENT                    // ��ֵ����(A)          PL[1]
#define MAX_PM_SPEED        (MAX_VOLOCITY*2000/60)          // ���ƽ���ٶ�(cnt/s)  VH[2]
#define MIN_PM_SPEED        (uint32_t)(-MAX_PM_SPEED)            // ��Сƽ���ٶ�(cnt/s)  VL[2]
#define MAX_FB_SPEED        ((MAX_VOLOCITY+1000)*2000/60)   // ������ٶ�(cnt/s)  HL[2]
#define MIN_FB_SPEED        (uint32_t)(-MAX_FB_SPEED)            // ��С�����ٶ�(cnt/s)  LL[2]

/* ƽ���˶���ز��� */ 
#define PM_ACC              3500000                         // ƽ�����ٶ�(cnt/s^2)  AC
#define PM_DEC              3500000                         // ƽ�����ٶ�(cnt/s^2)  DC
#define QUICKSTOP_DEC       1000000000                      // ��ͣ���ٶ�(cnt/s^2)  SD
#define POSITION_LIMIT_MAX  1000000000                      // ���λ�ü���         VH[3] HL[3]
#define POSITION_LIMIT_MIN  (uint32_t)-1000000000                // ��Сλ�ü���         VL[3] LL[3]

/* CANopen COB identifiers */
#define COBID_NMT_SERVICE   0x000
#define COBID_SYNC          0x080
#define COBID_EMERGENCY     0x080
#define COBID_TIME_STAMP    0x100
#define COBID_TPDO1         0x180
#define COBID_RPDO1         0x200
#define COBID_TPDO2         0x280
#define COBID_RPDO2         0x300
#define COBID_TPDO3         0x380
#define COBID_RPDO3         0x400
#define COBID_TPDO4         0x480
#define COBID_RPDO4         0x500
#define COBID_TSDO          0x580
#define COBID_RSDO          0x600
#define COBID_HEARTBEAT     0x700

/* NMT Command Specifier */
#define NMT_ENTER_OPERATIONAL      0x01
#define NMT_ENTER_STOPPED          0x02
#define NMT_ENTER_PRE_OPERATIONAL  0x80
#define NMT_RESET_NODE             0x81
#define NMT_RESET_COMMUNICATION    0x82

/* Binary Interpreter Command */
#define UM_IDLE             0x00
#define UM_TCM              0x01                      // Torque control mode,���ؿ���ģʽ
#define UM_PCM              0x05                      // Position control mode��λ�ÿ���ģʽ
#define UM_UNC              0x06                      // ����ȷ��ģʽ��������ȫ���������

#if JV_IN_PCM
   #define UM_SCM           0x05                      // Position control mode��
#else
   #define UM_SCM           0x02                      // Speed control mode
#endif

#define TYPE_INTEGER      0
#define TYPE_FLOAT        1

#define MO_OFF            0
#define MO_ON             1

#define POS_REL           0
#define POS_ABS           1

/*****************************************************************************/
//����ѡ��ĵ��ID��Ϊ��������������
/*****************************************************************************/

/*���ѡ��*/
#define ID_0    //��ʾ���еĵ�� 
#define ID_1
#define ID_2
#define ID_3
//#define ID_4
//#define ID_5
//#define ID_6
//#define ID_7
//#define ID_8
//#define ID_9
//#define ID_10
//#define ID_11
//#define ID_12
//#define ID_13
//#define ID_14
//#define ID_15


/*******************************************************************************
* �ṹ��
*******************************************************************************/
/* CANѭ������Ԫ�� */ 
typedef struct __CANDATA 
{
   uint16_t COBID;               // CANopen COB identifier + NodeID
   uint8_t  DLC;                 // Data Length Code
   uint8_t  DATA[8];             // Data
} CANDATA;

/* CANѭ�����нṹ�� */
typedef struct __CANQUEUE
{
   uint16_t Front;        
   uint16_t Rear;
   CANDATA CANBUF[CAN_BUF_NUM];
} CANQUEUE;

/* Elmo�ṹ��,��¼�ڵ�ID,״̬�Ϳ��Ʋ��� */
typedef struct __Elmo
{
   uint8_t NodeID;         // elmo����
   uint8_t CurOPMode;      // ��ǰ����ģʽ 
}Elmo;
/*����ת��������*/
typedef union 
{
    uint32_t uint32_t_form;
    int32_t  int32_t_form;
    uint8_t  uint8_t_form[4];
    int8_t   int8_t_form[4];
    float    float_form;
}DataConvert;



/*******************************************************************************
* ��������
*******************************************************************************/
/* ELMO��ʼ������,������� */
extern uint32_t Elmo_Init( CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr);
extern void Elmo_Reinit(uint8_t elmoID);

/* ELMO���ƺ������������ */
extern uint8_t Elmo_PTM(uint8_t elmoID, float torque);
extern uint8_t Elmo_PVM(uint8_t elmoID, int32_t speed);  
extern uint8_t Elmo_PPM(uint8_t elmoID, uint32_t speed, int32_t position, uint8_t PPMmode);		//		POS_ABS  POS_REL 
extern uint8_t Elmo_Close(uint8_t elmoID);
extern uint8_t Elmo_Stop(uint8_t elmoID);
extern uint8_t Elmo_SetAcc(uint8_t elmoID, uint32_t acc, uint32_t dec);
extern void Elmo_Read_POS(uint8_t elmoID);
extern void Elmo_Set_POS(uint8_t elmoID,int32_t POS);
extern void Elmo_Read_ACT_CUR(uint8_t elmoID);
extern void Elmo_Pre_PVM(uint8_t elmoID);
extern void Omni_Elmo_Close(void);
extern void Omni_Elmo_Stop(void);
extern void Omni_Elmo_PVM(void);


/* CANOpen��ʵ�ֺ���,��������� */
static void NMTCmd(Elmo *elmo, uint8_t MNTCmd);
static void RSDO(Elmo *elmo, uint16_t Index, uint8_t SubIndex, uint32_t Data);
static void RPDO2_Cmd_data(Elmo *elmo, uint8_t *Cmd, uint8_t Index, uint8_t Type, uint32_t Data);
static void RPDO2_Cmd_string(Elmo *elmo, uint8_t *Cmd);

/* Ӳ����ʼ������,��������� */
//static void CAN_init(CAN_TypeDef* CANx);                    
static void TIM7_init(uint8_t PPr, uint8_t SPr);
static int Self_test(void);
static void Variate_init(void);

/* ���ݷ��͡�ת������ʱ����,��������� */
static void Elmo_SendCmd(void);
static void Elmo_CANSend(CANDATA *pCANDATA);
static void Elmo_Delay100us_IDx( Elmo *elmo , uint8_t N100us);
static void Elmo_software_delay_ms(unsigned int t);
static uint32_t f2h(float x);

#endif
