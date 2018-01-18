
/*
**********************************************************************************************************
    *@  file:007_elmo.c
    *@  author: 007
    *@  date: 3/20/2015
    *@  version: v2.0
    *@  brief: ���ӵ������õĿ��������Ϳ���ɲ������
    *@         ����������Ԥ������
    *@         ������note1�еĺ���ע��
    *@         ������note2�е�ע������
    *...............................................................................
    *@ Notes1: (1)��ʼ��������       
    *@            name      : Elmo_Init(CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr)
    *@            function  : ���ó�ʼ����������ʹ��CANͨ��ģ�飬���ΪCAN1��CAN2������ѡ��
    *@			                  CAN�ڣ�ȷ������ͨ�Ų���ȷ��ͨ���ٶ�Ϊ1Mbps��ͬʱѡ��TIM7���ж�
    *@	     		              ���ȼ���Ϊ�ײ㱨�ķ����ṩʱ��	
    *@            input     : CANx      CAN1 or CAN2
    *@						            PPr       TIM7����ռ���ȼ�
    *@                        SPr       TIM7�Ĵ����ȼ�
    *@            output    : 0	          ��ʼ���ɹ�
    *@                        0x80000000  ����û������CAN����
    *@ 							          ����         ��Ӧ��ELMO��IDλû����1
    *@	
    *@         (2)����ģʽ������        		
    *@            name      : Elmo_PTM(u8 elmoID, float torque);
    *@            function  : �������ؿ��ƺ�����ά�ֵ�����غ㶨,֧��ģʽ�л�  
    *@            input     : elmoID    ȡelmo�ڵ�ID     
    *@			    			        torque     Ŀ��ת��(A)
    *@            output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@ 	
    *@         (3)�ٶ�ģʽ������  
    *@            name      : Elmo_PVM(u8 elmoID, s32 speed);      
    *@            function  : �����ٶȿ��ƺ���,֧��ģʽ�л� ��֧���������ã�elmo 
    *@                        ���������ٶȻ���ٶ�ʹ�� ��ת�ٴﵽ�趨ֵ
    *@            input     : elmoID    ȡelmo�ڵ�ID      
    *@                        speed     Ŀ���ٶ�(cnt/s)
    *@            output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@
    *@         (4)λ��ģʽ������   
    *@            name      : Elmo_PPM(u8 elmoID, u32 speed, s32 position, u8 PPMmode);
    *@            function  : �����ٶȿ��ƺ�����֧��ģʽ�л���֧���������ã�����POS_ABSģʽ
    *@ 						            ����ָ���յ����λ��������ϵ��ĵ����λ�ã�Ҳ���ǵ�� �У�Ŀ    
    *@ 						            ��λ��ʼ�ľ���λ�á�����POS_RELģʽ�У����յ����λ��������elmo
    *@ 				                ����λ�üĴ����еľ���λ�ü���Ŀ��λ�ã�����λ�üĴ�����CAN_init 
    *@                        ��Ĭ��Ϊ0
    *@						  ����1��
    *@						  Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						  while(position != 500000);
    *@					   	  Elmo_PPM(1, 50000, 250000, POS_REL);		
    *@                             ���λ����750000��		
    *@						            ����2��
    *@						            Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						            while(position != 500000);
    *@						            Elmo_PPM(1, 50000, 250000, POS_ABS);			
    *@						            ���λ����250000��		
    *@						            ����3��
    *@						            Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						            while(position <= 250000);
    *@						            Elmo_PPM(1, 50000, 250000, POS_REL);			
    *@					              ���λ����750000��		
    *@            input     : elmoID      ȡelmo�ڵ�ID,�����ڵ��̿����е���1--4        
    *@ 						            speed       �ٶ�(cnt/s)
    *@ 					              position    Ŀ��λ��(cnt)
    *@ 						            PPMmode     ����ģʽ
    *@ 						            POS_ABS    // PPM���з�ʽ:����λ��
    *@ 		   			            POS_REL    // PPM���з�ʽ:���λ��
    *@  			    output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@
    *@         (5)����ͷź�����   
    *@            name      : Elmo_Close(u8 elmoID);
    *@            function  : ��������رգ���������Լ�����ʻ,֧���������ã�֧�����ϵ��ã���
    *@						            ���ڸ���elmo����ǰ����ã�������Ӱ�����������Ĺ���ʵ�ֺ�elmo��
    *@						            ����														
    *@            input     : elmoID      ȡelmo�ڵ�ID     
    *@  			    output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@         
    *@         (6)�������������
    *@            name      : Elmo_Stop(u8 elmoID);
    *@            function  : ���������ά�ֵ����ǰλ�ò��䣬֧���������ã�֧�����ϵ��ã���
    *@ 						            ���ڸ���elmo����ǰ����ã�������Ӱ�����������Ĺ���ʵ�ֺ�elmo��
    *@ 						            ����	
    *@            input     : elmoID      ȡelmo�ڵ�ID     
    *@  			    output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@	
    *@         (7)���ٶ����ú�����	
    *@  			    name      : Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec)
    *@            function  : �����ٶ�ģʽ��λ��ģʽ�ĵ���Ӽ��٣����û���ں����е��ã���ʼ��
    *@	                      ����ٶ�Ĭ��Ϊ1000000000�����ٶ�Ĭ��Ϊ1000000000
    *@			      input     : elmoID      ȡelmo�Ľڵ�ID
    *@                        acc         ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
    *@                        dec         ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
    *@  			    output    : 0         �������óɹ�
    *@         (8)λ�ö�ȡ������	
    *@  			    name      : Elmo_Read_POS(u8 elmoID)
    *@            function  : ��ȡ��������������ݣ��������ж��ж�ȡ���洢��ȫ�ֱ���Encoder_Data��
    *@			      input     : elmoID      ȡelmo�Ľڵ�ID
    *@  			    output    : NONE
    *@         (9)λ�����ú�����	
    *@  			    name      : Elmo_Set_POS(u8 elmoID,s32 POS)
    *@            function  : ���õ��������������
    *@			      input     : elmoID      ȡelmo�Ľڵ�ID
    *@                        POS         Ҫ��elmo���õĵ�ַ
    *@  			    output    : NONE
    *@         (10)������ȡ������	
    *@  			     name      : Elmo_Read_ACT_CUR(u8 elmoID)
    *@             function  : ��ȡ����й����������ݣ��������ж��ж�ȡ���洢��ȫ�ֱ���IQ��
    *@			       input     : elmoID      ȡelmo�Ľڵ�ID
    *@  			     output    : NONE
    *...............................................................................
    *@ Notes2: (1)��CAN������ 
    *@			     CAN1 RX -> PD0     CAN1 TX -> PD1
    *@               CAN2 RX -> PB5     CAN2 TX -> PB6
    *@         (2)���޸ĺ궨��CAN_BUF_NUM��ֵ�޸Ļ�����Ŀ
    *@         (3)���޸�TIM7_Configuration()��TIM7�����ж����ȼ�
    *@         (4)��ע��CANʱ��Ϊ42MHz,TIM7ʱ��Ϊ42MHz
    *@ 	       (5)��ע��д���ȼ����麯��
    *@         (6)��ע������ж��еĵ��ID
    *@         (7)����Elmo_Read_POS��Elmo_Read_ACT_CUR��������֮ǰҪ��ʱ1ms�����Է�ֹ��ϵ�ǰ������˶�״̬
    *@         (8)����Four_Elmo_PVM��������֮ǰҪ����Elmo_Pre_PVM��Ԥ����
    *...............................................................................
    *@ Notes3: (1)��û��д�ڵ�ID����ID�ķ��亯��
    *@	       (2)��û�мӷ������Ļ��� 
    *@		   (3)��û�м�ƽ���������ú���	    
    *@		   (4)��Elmo_Stop()��Elmo_PTM()����Ӧʱ��̫������Ҫ�Ż�			
    *@         (5)��û�мӽڵ㱣��������������������				 
    *@    	   (6)��Elmo_Delay100us()���ԸĽ������Ч��    
    *@         (7)��GroupID��ʱ��������	
**********************************************************************************************************
*/
#include "elmo.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"


typedef uint32_t 			t_ureg;				/* �޷��żĴ������� */
typedef int32_t				t_sreg;				/* �з��żĴ������� */
typedef uint16_t			t_ureg_opt;			/* �޷����Ż��Ĵ������� */
typedef int16_t				t_sreg_opt;			/* �з����Ż��Ĵ������� */

typedef t_ureg				t_bool;				/* boolean���� */
typedef t_ureg				t_err;				/* ���������� */
typedef uint32_t			t_size;				/* ���ݳ������� */
typedef uint16_t			t_size_opt;			/* �Ż����ݳ��� */

static Elmo elmo[ELMO_NUM+1];
static Elmo chassisGroup;
const	 T_CanFrame MainHeartFrame={(uint32_t)(0x700+globalCAN_ID_MASTER_CONTROL) << 21,0,{0,0,0,0,0,0,0,0}};
	
u32 ElmoHeartbeat = 0;   //������¼ELMO����
#define DEF_ElmoHeartbeat_VALUE     (0x1FF)  //ֻ����elmo�յ���ֵ��ʾ�ĸ�elmo������Ż��ͳ������ź���

/**
  ******************************************************************************
  * @name     : Variate_init
  * @function : ������ʼ��
  * @input    : None
  * @output   : None
  ******************************************************************************
  */
static void Variate_init(void)
{
	ElmoHeartbeat=0;
	memset(elmo,0,(ELMO_NUM+1)*sizeof(Elmo));
	memset(&chassisGroup,0,sizeof(Elmo));
}






/******CAN_SendFrame*******/
uint32_t CAN_SendFrame(T_CanFrame *pCanFrame,u8 NodeID)   //���ID    
{
	uint16_t i;
	hcan2.pTxMsg->StdId=pCanFrame->IxR;
	hcan2.pTxMsg->IDE=CAN_ID_STD; 
	hcan2.pTxMsg->RTR=CAN_RTR_DATA;
	hcan2.pTxMsg->DLC=pCanFrame->DTxR;
	for(i=0;i<hcan2.pTxMsg->DLC;i++)
		hcan2.pTxMsg->Data[i]=pCanFrame->Data.u08[i];
	
		if(HAL_CAN_Transmit(&hcan2,50)!=HAL_OK) return 1;
	return 0;
	
}



/**
  ******************************************************************************
  * @name     : RSDO
  * @function : ʹ������SDO����ָ���
  * @input    : elmo      ȡelmo�ڵ�ID
  *             Index     ����
  *             SubIndex  ������
  *             Data      ����
  * @output   : None
  ******************************************************************************
  */



void RSDO(u8 NodeID,u16 Index,u8 SubIndex,u32 Data)
{
	T_CanFrame CanFrame;
	
	CanFrame.IxR=COBID_RSDO+NodeID;
	CanFrame.DTxR=8;			// DLC
//	CanFrame.Data.u32[0]=((uint32_t)SubIndex) << 24 | ((uint32_t)Index<<8) | 0x22;
//	CanFrame.Data.u32[1]=Data;
	CanFrame.Data.u08[0]=0x22;// CS,������δȷ��
	CanFrame.Data.u08[1]=(Index&0xFF);               // Index
	CanFrame.Data.u08[2]=(Index&0xFF00)>>8;
	CanFrame.Data.u08[3]=(SubIndex); 
	CanFrame.Data.u08[4]=(Data&0xFF);                // Data
	CanFrame.Data.u08[5]=(Data&0xFF00)>>8;
	CanFrame.Data.u08[6]=(Data&0xFF0000)>>16;
	CanFrame.Data.u08[7]=(Data&0xFF000000)>>24;

	
	CAN_SendFrame(&CanFrame,NodeID);


}

/**
  ******************************************************************************
  * @name     : NMTCmd
  * @function : ����CANOPEN��NMT״̬����
  * @input    : elmo    ȡelmo�Ľڵ�ID
  *             MNTCmd  NMTָ��,NMT_xxx
  * @output   : None
  ******************************************************************************
  */
static void NMTCmd(u8 NodeID, u8 MNTCmd)
{
    T_CanFrame CanFrame;
    
    CanFrame.IxR  = COBID_NMT_SERVICE ;    //CanFrame.IxR  = (pCANDATA->COBID << 21);
    CanFrame.DTxR = 2;                          //CanFrame.DTxR = (pCANDATA->DLC)&0x0f;
    CanFrame.Data.u08[0] = MNTCmd;
    CanFrame.Data.u08[1] = NodeID;
    
    CAN_SendFrame(&CanFrame,NodeID);
}

/**
  ******************************************************************************
  * @name     : RPDO2_Cmd_data
  * @function : ʹ�ö����Ʊ����ELMO��������ָ��
  *             CANopen RPDO2 -> 0x2012 ����������-���ù���
  * @input    : elmo    ȡelmo�ڵ�ID
  *             Cmd     ����,���ַ�����ʽ����
  *             Index   ������±�           
  *             Type    ��������
  *             Data    ����
  * @output   : None
  ******************************************************************************
  */
static void RPDO2_Cmd_data(u8 NodeID, u8 *Cmd, u8 Index, u8 Type, u32 Data)
{
    T_CanFrame CanFrame;
    
    CanFrame.IxR  = COBID_RPDO2 + NodeID;  //CanFrame.IxR  = (pCANDATA->COBID << 21);
    CanFrame.DTxR = 8;                                  //CanFrame.DTxR = (pCANDATA->DLC)&0x0f;
    CanFrame.Data.u08[0] = (*Cmd++);
    CanFrame.Data.u08[1] = (*Cmd);
    CanFrame.Data.u08[2] = Index;
    CanFrame.Data.u08[3] = Type<<7;
  //  CanFrame.Data.u32[1] = Data;
		CanFrame.Data.u08[4] = (Data&0xFF);
		CanFrame.Data.u08[5] = (Data&0xFF00)>>8;
		CanFrame.Data.u08[6] = (Data&0xFF0000)>>16;
		CanFrame.Data.u08[7] = (Data&0xFF000000)>>24;
	
    CAN_SendFrame(&CanFrame,NodeID);
}

/* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ-----------------------*/
void Elmo0StateUpdate(void)
{
    u32 i;
    u8 tmpOPMode;
    tmpOPMode = elmo[1].CurOPMode;
    
    if(ELMO_NUM == 1)
    {
        elmo[0].CurOPMode = tmpOPMode;
    }
    else
    {
        for(i = 2; i <= ELMO_NUM; i++)
        {
            if(elmo[i].CurOPMode != tmpOPMode)
            {
                elmo[0].CurOPMode = UM_UNC;
                break;
            }
        }
    }
}
/**
  ******************************************************************************
  * @name     : RPDO2_Cmd_string
  * @function : ʹ�ö����Ʊ����ELMO�����ַ���ָ��
  *             CANopen RPDO2 -> 0x2012 ����������-ִ�й���
  * @input    : elmo    ȡelmo�ڵ�ID
                cmd     ����,���ַ�����ʽ����
  * @output   : None
  ******************************************************************************
  */
static void RPDO2_Cmd_string(u8 NodeID, u8 *Cmd)
{
    T_CanFrame CanFrame;
    
    CanFrame.IxR  = COBID_RPDO2 + NodeID;  //CanFrame.IxR  = (pCANDATA->COBID << 21);
    CanFrame.DTxR = 4;                                  //CanFrame.DTxR = (pCANDATA->DLC)&0x0f;
    CanFrame.Data.u08[0] = (*Cmd++);
    CanFrame.Data.u08[1] = (*Cmd);
    CanFrame.Data.u08[2] = 0;
    CanFrame.Data.u08[3] = 0;
    
    CAN_SendFrame(&CanFrame,NodeID);
}

u32 Elmo_Init(void)
{
		u8 i=0;
		int temp =0;

		/*���������ʼ��*/
		Variate_init();

		/*�رյ��*/
		RSDO(0, 0x6040, 0x00, 0x00);
		HAL_Delay(10);

		/* ��elmo����ڵ�ID */
//		for(i=0; i <= ELMO_NUM; i++)
//		{
//			elmo[i].NodeID = i;			
//		}
//		/* ��elmo������ID */
//		chassisGroup.NodeID = 64;   //1~4
//		
//		/* ��Elmo1~4  9~8 ����GroupID */
//		for(i = 1; i <= 4; i++)
//		{
//			RSDO(i, 0x2040, 0x00, chassisGroup.NodeID);
//			HAL_Delay(1);
//		}
//		/* ��ȫ��ڵ����ͨ�Ÿ�λ */
//		HAL_Delay(50);
//		NMTCmd(0, NMT_RESET_COMMUNICATION);
//		HAL_Delay(50);
//		
		    /* ��ȫ��ڵ����ͨ�Ÿ�λ */
    HAL_Delay(50);
    NMTCmd(0, NMT_RESET_COMMUNICATION);
    HAL_Delay(50);

    /* Elmo�������Start------------------------------------------------------*/
    RSDO(0, 0x1017, 0, (100&0xffff));//��������Ϊ100ms
    HAL_Delay(10);
    
    RSDO(0, 0x6007, 0, 3);//������ϣ�ST
    HAL_Delay(10);
        
    RSDO(0, 0x605A, 0, 5);
    HAL_Delay(10);

    RSDO(0, 0x1029, 1, 2);//ͨ�Ŵ�����Ϊ��ֹͣ
    HAL_Delay(10);
		
    RSDO(0, 0x1016, 1, (globalCAN_ID_MASTER_CONTROL<<16) | (150&0xffff));//����������ʱ���� ID��MASTER_CONTROL_ID T��150ms
    HAL_Delay(100);
		
		    /* Elmo�������End--------------------------------------------------------*/
    
    //Elmo_software_delay_ms(100);
    //	/* �ȴ�Elmo�������,�����յ�Boot up���� */
    //	Elmo_Delay100us(50);

    /* CANOpenͨ�Ų�����ʼ�� */
    /* RPDO1->0x6040,ָ����,2�ֽ�,�첽���� */
    // ������Ĭ��ӳ��,����Ҫ�޸� //

    /* RPDO2->0x2012,�����Ʊ�������,4�ֽ�,�첽���� */
    // ������Ĭ��ӳ��,����Ҫ�޸� //

    /* ����TPDO,Debugʱ����,������ʱ��ùر� */
    RSDO(0, 0x1A00, 0x00, 0);
    HAL_Delay(10);
    RSDO(0, 0x1A01, 0x00, 0);
    HAL_Delay(10);
		
		   /* ����NMT����״̬ */
    HAL_Delay(5);
    NMTCmd(0, NMT_ENTER_OPERATIONAL);
    HAL_Delay(5);
		
		    /*�˺��������ݻ��Ϊ0x05������״̬��Ϊ����̬*/

    /* �ر����� */
    RPDO2_Cmd_data(0, (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
    HAL_Delay(2);

    /* ��ʼ�����ٶ� */
    RPDO2_Cmd_data(0, (u8 *)"PM", 0, TYPE_INTEGER, 0x01);
    HAL_Delay(1);	
    RPDO2_Cmd_data(0, (u8 *)"AC", 0, TYPE_INTEGER, 100000000);
    HAL_Delay(1);
    RPDO2_Cmd_data(0, (u8 *)"DC", 0, TYPE_INTEGER, 100000000);
    HAL_Delay(1);

    /* Enter in SCM */
    RPDO2_Cmd_data(0, (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
    HAL_Delay(1);

    /* ʹ������ */
    RPDO2_Cmd_data(0, (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
    
    for(i = 0; i <= ELMO_NUM; i++)
    {
        elmo[i].CurOPMode = UM_SCM;			
    }
    
    HAL_Delay(2);
    
    return temp;
}

/**
  ******************************************************************************
  * @name     : Elmo_PVM
  * @function : �����ٶ�ģʽ����
  * @input    : elmoID      ȡelmo�ڵ�ID
  *             speed       �ٶ�(cnt/s)
  * @output   : 0       �������óɹ�
  *             1       ��������ʧ��
  ******************************************************************************
  */
u8 Elmo_PVM(u8 elmoID, s32 speed)
{
    u32 i;
    
    /* ��ǰģʽ��Ϊ�ٶ�ģʽ,�л�Ϊ�ٶ�ģʽ*/
    if(elmo[elmoID].CurOPMode != UM_SCM)
    {
        if(elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(elmoID, (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
            HAL_Delay(2);
        }
        
        RPDO2_Cmd_data(elmoID, (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
        HAL_Delay(1);

        RPDO2_Cmd_data(elmoID, (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
        HAL_Delay(2);
        
        if(elmoID == 0)            /* ���ʹ�ù㲥ģʽ��������״̬������ֵΪSCM */		
        {
            for(i = 0; i <= ELMO_NUM; i++)
            {
                elmo[i].CurOPMode = UM_SCM;			
            }
        }
        else                      /* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
        {
            elmo[elmoID].CurOPMode = UM_SCM;
            Elmo0StateUpdate();
        }
    }
    
    /* ����Ŀ���ٶ� */
    RPDO2_Cmd_data(elmoID, (u8 *)"JV", 0, TYPE_INTEGER, speed);
    HAL_Delay(1);

    /* �����ٶ�ģʽ */
    RPDO2_Cmd_string(elmoID, (u8 *)"BG");
    HAL_Delay(1);
    
    return 0;
}
/**
  ******************************************************************************
  * @name     : Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec)
  * @function : �����ٶ�ģʽ��λ��ģʽ�ĵ���Ӽ���
  * @input    : elmoID  ȡelmo�Ľڵ�ID
  *             acc     ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
  *             dec     ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
  * @output   : 0       �������óɹ�
  *             1       ��������ʧ��
  ******************************************************************************
  */
u8 Elmo_SetAcc(u8 elmoID, u32 acc)
{ 
    RPDO2_Cmd_data(elmoID, (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
    HAL_Delay(2);	
    RPDO2_Cmd_data(elmoID, (u8 *)"PM", 0, TYPE_INTEGER, 0x01);
    HAL_Delay(1);	
    RPDO2_Cmd_data(elmoID, (u8 *)"AC", 0, TYPE_INTEGER, acc);
    HAL_Delay(1);
    RPDO2_Cmd_data(elmoID, (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
    HAL_Delay(2);
    return 0;
}

u8 Elmo_SetDec(u8 elmoID, u32 dec)
{ 
    RPDO2_Cmd_data(elmoID, (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
    HAL_Delay(2);	
    RPDO2_Cmd_data(elmoID, (u8 *)"PM", 0, TYPE_INTEGER, 0x01);
    HAL_Delay(1);	
    RPDO2_Cmd_data(elmoID, (u8 *)"DC", 0, TYPE_INTEGER, dec);
    HAL_Delay(1);
    RPDO2_Cmd_data(elmoID, (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
    HAL_Delay(2);
    return 0;
}

u8 Elmo_Break(u8 elmoID)
{
	Elmo_PVM(elmoID, 0);
	return 0;
}

u8 Elmo_Release(u8 elmoID)
{
	u32 i;
	RPDO2_Cmd_data(elmoID, (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
	HAL_Delay(2);
	if(elmoID == 0)            /* ���ʹ�ù㲥ģʽ��������״̬������ֵΪIDLE */		
	{
		for(i = 0; i <= ELMO_NUM; i++)
		{
			elmo[i].CurOPMode = UM_IDLE;			
		}
	}
	else                      /* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
	{
		elmo[elmoID].CurOPMode = UM_IDLE;
		Elmo0StateUpdate();
	}
	return 0;
}
