
/*
**********************************************************************************************************
    *@  file:007_elmo.c
    *@  author: 007
    *@  date: 3/20/2015
    *@  version: v2.0
    *@  brief: 增加底盘试用的快速启动和快速刹车函数
    *@         增加了启动预处理函数
    *@         更新了note1中的函数注释
    *@         更新了note2中的注意事项
    *...............................................................................
    *@ Notes1: (1)初始化函数：       
    *@            name      : Elmo_Init(CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr)
    *@            function  : 调用初始化函数即可使用CAN通信模块，入参为CAN1或CAN2，用来选择
    *@			                  CAN口，确保物理通信层正确，通信速度为1Mbps，同时选择TIM7的中断
    *@	     		              优先级，为底层报文发送提供时序	
    *@            input     : CANx      CAN1 or CAN2
    *@						            PPr       TIM7的抢占优先级
    *@                        SPr       TIM7的从优先级
    *@            output    : 0	          初始化成功
    *@                        0x80000000  主控没有连上CAN总线
    *@ 							          其他         对应的ELMO的ID位没有置1
    *@	
    *@         (2)力矩模式函数：        		
    *@            name      : Elmo_PTM(u8 elmoID, float torque);
    *@            function  : 单轴力矩控制函数，维持电机力矩恒定,支持模式切换  
    *@            input     : elmoID    取elmo节点ID     
    *@			    			        torque     目标转矩(A)
    *@            output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@ 	
    *@         (3)速度模式函数：  
    *@            name      : Elmo_PVM(u8 elmoID, s32 speed);      
    *@            function  : 单轴速度控制函数,支持模式切换 ，支持连续调用，elmo 
    *@                        将以最大加速度或减速度使电 机转速达到设定值
    *@            input     : elmoID    取elmo节点ID      
    *@                        speed     目标速度(cnt/s)
    *@            output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@
    *@         (4)位置模式函数：   
    *@            name      : Elmo_PPM(u8 elmoID, u32 speed, s32 position, u8 PPMmode);
    *@            function  : 单轴速度控制函数，支持模式切换，支持连续调用，对于POS_ABS模式
    *@ 						            终是指最终电机的位置相对于上电后的电机的位置，也就是电机 中，目    
    *@ 						            标位置始的绝对位置。对于POS_REL模式中，最终电机的位置是现在elmo
    *@ 				                绝对位置寄存器中的绝对位置加上目标位置，绝对位置寄存器在CAN_init 
    *@                        后默认为0
    *@						  例子1：
    *@						  Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						  while(position != 500000);
    *@					   	  Elmo_PPM(1, 50000, 250000, POS_REL);		
    *@                             电机位置在750000处		
    *@						            例子2：
    *@						            Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						            while(position != 500000);
    *@						            Elmo_PPM(1, 50000, 250000, POS_ABS);			
    *@						            电机位置在250000处		
    *@						            例子3：
    *@						            Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						            while(position <= 250000);
    *@						            Elmo_PPM(1, 50000, 250000, POS_REL);			
    *@					              电机位置在750000处		
    *@            input     : elmoID      取elmo节点ID,请勿在底盘控制中调用1--4        
    *@ 						            speed       速度(cnt/s)
    *@ 					              position    目标位置(cnt)
    *@ 						            PPMmode     运行模式
    *@ 						            POS_ABS    // PPM运行方式:绝对位置
    *@ 		   			            POS_REL    // PPM运行方式:相对位置
    *@  			    output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@
    *@         (5)电机释放函数：   
    *@            name      : Elmo_Close(u8 elmoID);
    *@            function  : 驱动输出关闭，电机靠惯性继续行驶,支持连续调用，支持联合调用（能
    *@						            够在各种elmo函数前后调用），不会影响其他函数的功能实现和elmo稳
    *@						            定性														
    *@            input     : elmoID      取elmo节点ID     
    *@  			    output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@         
    *@         (6)电机抱死函数：
    *@            name      : Elmo_Stop(u8 elmoID);
    *@            function  : 电机抱死，维持电机当前位置不变，支持连续调用，支持联合调用（能
    *@ 						            够在各种elmo函数前后调用），不会影响其他函数的功能实现和elmo稳
    *@ 						            定性	
    *@            input     : elmoID      取elmo节点ID     
    *@  			    output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@	
    *@         (7)加速度设置函数：	
    *@  			    name      : Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec)
    *@            function  : 设置速度模式和位置模式的电机加减速，如果没有在函数中调用，初始化
    *@	                      后加速度默认为1000000000，减速度默认为1000000000
    *@			      input     : elmoID      取elmo的节点ID
    *@                        acc         加速度,加速度最大不能超过1000000000,同时应考虑电机性能
    *@                        dec         减速度,减速度最大不能超过1000000000,同时应考虑电机性能
    *@  			    output    : 0         函数调用成功
    *@         (8)位置读取函数：	
    *@  			    name      : Elmo_Read_POS(u8 elmoID)
    *@            function  : 读取电机编码器的数据，数据在中断中读取，存储在全局变量Encoder_Data中
    *@			      input     : elmoID      取elmo的节点ID
    *@  			    output    : NONE
    *@         (9)位置设置函数：	
    *@  			    name      : Elmo_Set_POS(u8 elmoID,s32 POS)
    *@            function  : 设置电机编码器的数据
    *@			      input     : elmoID      取elmo的节点ID
    *@                        POS         要给elmo设置的地址
    *@  			    output    : NONE
    *@         (10)电流读取函数：	
    *@  			     name      : Elmo_Read_ACT_CUR(u8 elmoID)
    *@             function  : 读取电机有功电流的数据，数据在中断中读取，存储在全局变量IQ中
    *@			       input     : elmoID      取elmo的节点ID
    *@  			     output    : NONE
    *...............................................................................
    *@ Notes2: (1)、CAN口引脚 
    *@			     CAN1 RX -> PD0     CAN1 TX -> PD1
    *@               CAN2 RX -> PB5     CAN2 TX -> PB6
    *@         (2)、修改宏定义CAN_BUF_NUM的值修改缓冲数目
    *@         (3)、修改TIM7_Configuration()中TIM7更新中断优先级
    *@         (4)、注意CAN时钟为42MHz,TIM7时钟为42MHz
    *@ 	       (5)、注意写优先级分组函数
    *@         (6)、注意更改中断中的电机ID
    *@         (7)、在Elmo_Read_POS和Elmo_Read_ACT_CUR函数调用之前要延时1ms，可以防止打断当前电机的运动状态
    *@         (8)、在Four_Elmo_PVM函数调用之前要调用Elmo_Pre_PVM做预处理
    *...............................................................................
    *@ Notes3: (1)、没有写节点ID和组ID的分配函数
    *@	       (2)、没有加反馈报文机制 
    *@		   (3)、没有加平滑因子设置函数	    
    *@		   (4)、Elmo_Stop()对Elmo_PTM()的响应时间太长，需要优化			
    *@         (5)、没有加节点保护函数和总线心跳报文				 
    *@    	   (6)、Elmo_Delay100us()可以改进，提高效率    
    *@         (7)、GroupID暂时不能用了	
**********************************************************************************************************
*/
#include "elmo.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"


typedef uint32_t 			t_ureg;				/* 无符号寄存器类型 */
typedef int32_t				t_sreg;				/* 有符号寄存器类型 */
typedef uint16_t			t_ureg_opt;			/* 无符号优化寄存器类型 */
typedef int16_t				t_sreg_opt;			/* 有符号优化寄存器类型 */

typedef t_ureg				t_bool;				/* boolean类型 */
typedef t_ureg				t_err;				/* 错误码类型 */
typedef uint32_t			t_size;				/* 数据长度类型 */
typedef uint16_t			t_size_opt;			/* 优化数据长度 */

static Elmo elmo[ELMO_NUM+1];
static Elmo chassisGroup;
const	 T_CanFrame MainHeartFrame={(uint32_t)(0x700+globalCAN_ID_MASTER_CONTROL) << 21,0,{0,0,0,0,0,0,0,0}};
	
u32 ElmoHeartbeat = 0;   //用来记录ELMO心跳
#define DEF_ElmoHeartbeat_VALUE     (0x1FF)  //只有在elmo收到该值表示的各elmo心跳后才会送出心跳信号量

/**
  ******************************************************************************
  * @name     : Variate_init
  * @function : 参数初始化
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
uint32_t CAN_SendFrame(T_CanFrame *pCanFrame,u8 NodeID)   //电机ID    
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
  * @function : 使用下载SDO进行指令发送
  * @input    : elmo      取elmo节点ID
  *             Index     索引
  *             SubIndex  子索引
  *             Data      数据
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
	CanFrame.Data.u08[0]=0x22;// CS,传输量未确认
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
  * @function : 对于CANOPEN的NMT状态设置
  * @input    : elmo    取elmo的节点ID
  *             MNTCmd  NMT指令,NMT_xxx
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
  * @function : 使用二进制编码对ELMO发送数据指令
  *             CANopen RPDO2 -> 0x2012 二进制输入-设置功能
  * @input    : elmo    取elmo节点ID
  *             Cmd     命令,以字符串形式输入
  *             Index   命令的下标           
  *             Type    数据类型
  *             Data    数据
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

/* 用来判断是否所有电机模式相同，从而确定elmo[0]的模式-----------------------*/
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
  * @function : 使用二进制编码对ELMO发送字符串指令
  *             CANopen RPDO2 -> 0x2012 二进制输入-执行功能
  * @input    : elmo    取elmo节点ID
                cmd     命令,以字符串形式输入
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

		/*定义变量初始化*/
		Variate_init();

		/*关闭电机*/
		RSDO(0, 0x6040, 0x00, 0x00);
		HAL_Delay(10);

		/* 对elmo分配节点ID */
//		for(i=0; i <= ELMO_NUM; i++)
//		{
//			elmo[i].NodeID = i;			
//		}
//		/* 对elmo分配组ID */
//		chassisGroup.NodeID = 64;   //1~4
//		
//		/* 对Elmo1~4  9~8 设置GroupID */
//		for(i = 1; i <= 4; i++)
//		{
//			RSDO(i, 0x2040, 0x00, chassisGroup.NodeID);
//			HAL_Delay(1);
//		}
//		/* 对全体节点进行通信复位 */
//		HAL_Delay(50);
//		NMTCmd(0, NMT_RESET_COMMUNICATION);
//		HAL_Delay(50);
//		
		    /* 对全体节点进行通信复位 */
    HAL_Delay(50);
    NMTCmd(0, NMT_RESET_COMMUNICATION);
    HAL_Delay(50);

    /* Elmo心跳检测Start------------------------------------------------------*/
    RSDO(0, 0x1017, 0, (100&0xffff));//设置心跳为100ms
    HAL_Delay(10);
    
    RSDO(0, 0x6007, 0, 3);//电机故障，ST
    HAL_Delay(10);
        
    RSDO(0, 0x605A, 0, 5);
    HAL_Delay(10);

    RSDO(0, 0x1029, 1, 2);//通信错误行为：停止
    HAL_Delay(10);
		
    RSDO(0, 0x1016, 1, (globalCAN_ID_MASTER_CONTROL<<16) | (150&0xffff));//消费者心跳时间间隔 ID：MASTER_CONTROL_ID T：150ms
    HAL_Delay(100);
		
		    /* Elmo心跳检测End--------------------------------------------------------*/
    
    //Elmo_software_delay_ms(100);
    //	/* 等待Elmo启动完毕,即接收到Boot up报文 */
    //	Elmo_Delay100us(50);

    /* CANOpen通信参数初始化 */
    /* RPDO1->0x6040,指令字,2字节,异步传输 */
    // 驱动器默认映射,不需要修改 //

    /* RPDO2->0x2012,二进制编译输入,4字节,异步传输 */
    // 驱动器默认映射,不需要修改 //

    /* 禁用TPDO,Debug时开启,电流环时最好关闭 */
    RSDO(0, 0x1A00, 0x00, 0);
    HAL_Delay(10);
    RSDO(0, 0x1A01, 0x00, 0);
    HAL_Delay(10);
		
		   /* 进入NMT操作状态 */
    HAL_Delay(5);
    NMTCmd(0, NMT_ENTER_OPERATIONAL);
    HAL_Delay(5);
		
		    /*此后心跳数据会变为0x05，网络状态变为运行态*/

    /* 关闭驱动 */
    RPDO2_Cmd_data(0, (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
    HAL_Delay(2);

    /* 初始化加速度 */
    RPDO2_Cmd_data(0, (u8 *)"PM", 0, TYPE_INTEGER, 0x01);
    HAL_Delay(1);	
    RPDO2_Cmd_data(0, (u8 *)"AC", 0, TYPE_INTEGER, 100000000);
    HAL_Delay(1);
    RPDO2_Cmd_data(0, (u8 *)"DC", 0, TYPE_INTEGER, 100000000);
    HAL_Delay(1);

    /* Enter in SCM */
    RPDO2_Cmd_data(0, (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
    HAL_Delay(1);

    /* 使能驱动 */
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
  * @function : 单轴速度模式函数
  * @input    : elmoID      取elmo节点ID
  *             speed       速度(cnt/s)
  * @output   : 0       函数调用成功
  *             1       函数调用失败
  ******************************************************************************
  */
u8 Elmo_PVM(u8 elmoID, s32 speed)
{
    u32 i;
    
    /* 当前模式不为速度模式,切换为速度模式*/
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
        
        if(elmoID == 0)            /* 如果使用广播模式，将所有状态变量赋值为SCM */		
        {
            for(i = 0; i <= ELMO_NUM; i++)
            {
                elmo[i].CurOPMode = UM_SCM;			
            }
        }
        else                      /* 用来判断是否所有电机模式相同，从而确定elmo[0]的模式 */
        {
            elmo[elmoID].CurOPMode = UM_SCM;
            Elmo0StateUpdate();
        }
    }
    
    /* 设置目标速度 */
    RPDO2_Cmd_data(elmoID, (u8 *)"JV", 0, TYPE_INTEGER, speed);
    HAL_Delay(1);

    /* 运行速度模式 */
    RPDO2_Cmd_string(elmoID, (u8 *)"BG");
    HAL_Delay(1);
    
    return 0;
}
/**
  ******************************************************************************
  * @name     : Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec)
  * @function : 设置速度模式和位置模式的电机加减速
  * @input    : elmoID  取elmo的节点ID
  *             acc     加速度,加速度最大不能超过1000000000,同时应考虑电机性能
  *             dec     减速度,减速度最大不能超过1000000000,同时应考虑电机性能
  * @output   : 0       函数调用成功
  *             1       函数调用失败
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
	if(elmoID == 0)            /* 如果使用广播模式，将所有状态变量赋值为IDLE */		
	{
		for(i = 0; i <= ELMO_NUM; i++)
		{
			elmo[i].CurOPMode = UM_IDLE;			
		}
	}
	else                      /* 用来判断是否所有电机模式相同，从而确定elmo[0]的模式 */
	{
		elmo[elmoID].CurOPMode = UM_IDLE;
		Elmo0StateUpdate();
	}
	return 0;
}
