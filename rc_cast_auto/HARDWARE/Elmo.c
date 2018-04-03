#include "Elmo.h"

#ifdef ID_0 
  static CANQUEUE QUEUE_CAN_ID0;
#endif
#ifdef ID_1
  static CANQUEUE QUEUE_CAN_ID1;
#endif
#ifdef ID_2
  static CANQUEUE QUEUE_CAN_ID2;
#endif
#ifdef ID_3
  static CANQUEUE QUEUE_CAN_ID3;
#endif
#ifdef ID_4
  static CANQUEUE QUEUE_CAN_ID4;
#endif
#ifdef ID_5
  static CANQUEUE QUEUE_CAN_ID5;
#endif
#ifdef ID_6
  static CANQUEUE QUEUE_CAN_ID6;
#endif
#ifdef ID_7
  static CANQUEUE QUEUE_CAN_ID7;
#endif
#ifdef ID_8
  static CANQUEUE QUEUE_CAN_ID8;
#endif
#ifdef ID_9
  static CANQUEUE QUEUE_CAN_ID9;
#endif
#ifdef ID_10
  static CANQUEUE QUEUE_CAN_ID10;
#endif
#ifdef ID_11
  static CANQUEUE QUEUE_CAN_ID11;
#endif
#ifdef ID_12
  static CANQUEUE QUEUE_CAN_ID12;
#endif
#ifdef ID_13
  static CANQUEUE QUEUE_CAN_ID13;
#endif
#ifdef ID_14
  static CANQUEUE QUEUE_CAN_ID14;
#endif
#ifdef ID_15
  static CANQUEUE QUEUE_CAN_ID15;
#endif

CANQUEUE *QUEUE_CAN_IDx;         //指向队列的指针

static Elmo elmo[ELMO_NUM + 1];
static Elmo elmogroup;
static CAN_TypeDef* can = CAN1;         //初始化选择CANx的接口变量
static uint8_t CAN_Error = 0;         //如果CAN发送失败，CAN_Error置1
static uint32_t Elmo_Init_Flag = 0;   //用来记录ELMO实际的初始化的状态

//extern Arm_Struct Im_Arm;
/*
********************************************************************************
  *@  name      : CAN_init
  *@  function  : Initialization for CAN
  *@  input     : CANx      CAN1 or CAN2
  *@              PPr       TIM7的抢占优先级
  *@              SPr       TIM7的从优先级
  *@  output    : 0            初始化成功
  *@              1         初始化失败
********************************************************************************
*/
uint32_t Elmo_Init(CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr)
{  
    uint8_t i = 0;
    int temp = 0;
    /* 硬件及全局变量初始化 */ 
    //CAN_init( CANx );
    can = CANx;
    TIM7_init( PPr, SPr);    

    /*定义变量初始化*/
    Variate_init();

    /* 对elmo分配节点ID */
    for(i=0; i <= ELMO_NUM; i++)
    {
        elmo[i].NodeID = i;            
    }

    /* 对elmo分配组ID */
    elmogroup.NodeID = 64;

    /* 对全体节点进行通信复位 */
    for(i=1; i <= ELMO_NUM; i++)
    {
        NMTCmd(&elmo[i], NMT_RESET_COMMUNICATION);
        Elmo_Delay100us_IDx(&elmo[i],50);
    }
    
    /* 自检功能函数，CAN所有ELMO初始化成功，返回数值0，否则对应ELMO位置0，如果主控没有上线，返回0x80000000 */
    if((temp = Self_test()) == 0x80000000 )
    {
        return temp;        
    }
    

    //    /* 等待Elmo启动完毕,即接收到Boot up报文 */
    //    Elmo_Delay100us_IDx(&elmo[elmoID],50);

    /* CANOpen通信参数初始化 */
    /* RPDO1->0x6040,指令字,2字节,异步传输 */
    // 驱动器默认映射,不需要修改 //

    /* RPDO2->0x2012,二进制编译输入,4字节,异步传输 */
    // 驱动器默认映射,不需要修改 //

    /* 禁用TPDO,Debug时开启,电流环时最好关闭 */
    RSDO(&elmo[0], 0x1A00, 0x00, 0);//禁用PDO1
    Elmo_Delay100us_IDx(&elmo[0],150);
    RSDO(&elmo[0], 0x1A01, 0x00, 0);//禁用PDO2 最后使用的是Pdo2
    Elmo_Delay100us_IDx(&elmo[0],150);

    /* 进入NMT操作状态 */
    Elmo_Delay100us_IDx(&elmo[0],40);
    NMTCmd(&elmo[0], NMT_ENTER_OPERATIONAL);
    Elmo_Delay100us_IDx(&elmo[0],40);

    /* 关闭驱动 */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    Elmo_Delay100us_IDx(&elmo[0],80);

    /* 初始化加速度 */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"PM", 0, TYPE_INTEGER, 0x01);
    Elmo_Delay100us_IDx(&elmo[0],40);    
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"AC", 0, TYPE_INTEGER, 250000000);
    Elmo_Delay100us_IDx(&elmo[0],40);
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"DC", 0, TYPE_INTEGER, 500000000);
    Elmo_Delay100us_IDx(&elmo[0],40);

    /* Enter in SCM */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
    Elmo_Delay100us_IDx(&elmo[0],40);
    elmogroup.CurOPMode = UM_SCM;
    for(i=0;i<=ELMO_NUM;i++)
    {
        elmo[i].CurOPMode = UM_SCM;            
    }

    /* 使能驱动 */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    Elmo_Delay100us_IDx(&elmo[0],250);
    Elmo_Delay100us_IDx(&elmo[0],250);
    Elmo_Delay100us_IDx(&elmo[0],50);
    return temp;
}

void Elmo_Reinit(uint8_t elmoID)
{
    
    RSDO(&elmo[elmoID], 0x6040, 0x00, 0x80);  //清除错误标志位
    Elmo_Delay100us_IDx(&elmo[elmoID],20);
    
    RSDO(&elmo[elmoID], 0x2f41, 0x00, 0x02);     //  启动PPM 
    Elmo_Delay100us_IDx(&elmo[elmoID],20);
    
    NMTCmd(&elmo[elmoID], NMT_RESET_COMMUNICATION);
    Elmo_Delay100us_IDx(&elmo[elmoID],30);
    
    /* 进入NMT操作状态 */
    NMTCmd(&elmo[elmoID], NMT_ENTER_OPERATIONAL);
    Elmo_Delay100us_IDx(&elmo[elmoID],30);
    
    /* 初始化加速度 */
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"PM", 0, TYPE_INTEGER, 0x01);
    Elmo_Delay100us_IDx(&elmo[elmoID],20); //最大加速度1000000000 1000000
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"AC", 0, TYPE_INTEGER, 250000000);
    Elmo_Delay100us_IDx(&elmo[elmoID],40); //最大减速度1000000000
    RPDO2_Cmd_data(&elmo[0], (uint8_t *)"DC", 0, TYPE_INTEGER, 500000000);
    Elmo_Delay100us_IDx(&elmo[elmoID],40);             
    
    /* 关闭驱动 */
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    Elmo_Delay100us_IDx(&elmo[elmoID],30);
    
    /* 使能驱动 */
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    Elmo_Delay100us_IDx(&elmo[elmoID],60);
}
/*
********************************************************************************
  *@  name      : Elmo_PTM
  *@  function  : 单轴力矩模式函数
  *@  input     : elmoID    取elmo节点ID,请勿在底盘控制中调用elmo[1]~elmo[4]
  *@               torque    目标转矩(A)
  *@  output    : 0         函数调用成功
  *@              1         函数调用失败
********************************************************************************
*/
uint8_t Elmo_PTM(uint8_t elmoID, float torque)
{
    uint8_t i = 0;
    
    /* 如果使用广播控制, 当前模式不为电流模式,切换为电流模式*/
    if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_TCM)  
    {
        if( elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data( &elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID],100);
        }        
        
        /* 将所有状态变量赋值为TCM */
        elmogroup.CurOPMode = UM_TCM;            
        for(i=0;i<=ELMO_NUM;i++)
        {
            elmo[i].CurOPMode = UM_TCM;            
        }
        
        RPDO2_Cmd_data( &elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_TCM);
        Elmo_Delay100us_IDx(&elmo[elmoID],10);
        
        RPDO2_Cmd_data( &elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID],30);
    }  
    /* 使用单个ELMO控制,当前模式不为电流模式,切换为电流模式 */
     else if( elmo[elmoID].CurOPMode != UM_TCM)         
    {
        if( elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data( &elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID],100);
        }
        elmo[elmoID].CurOPMode = UM_TCM;
        RPDO2_Cmd_data( &elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_TCM);
        Elmo_Delay100us_IDx(&elmo[elmoID],10);

        RPDO2_Cmd_data( &elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID],30);
        
        /* 如果ELMO只挂载了1个，不管怎么改其的模式，elmo[0]的模式和它一样 */        
        if(ELMO_NUM == 1)
        {
            elmo[0].CurOPMode = UM_TCM;
        }    
        /* 用来判断是否所有电机模式相同，从而确定elmo[0]的模式 */
        else 
        {
            for(i=1;i<ELMO_NUM;i++)  
            {
                if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
                {
                    elmo[0].CurOPMode = UM_TCM;
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }            
        }
    }
        
    /* 设置目标转矩并运行电流模式 */
    RPDO2_Cmd_data( &elmo[elmoID], (uint8_t *)"TC", 0, TYPE_FLOAT, f2h(torque));
    Elmo_Delay100us_IDx(&elmo[elmoID],10);
    return 0;
}
/*
********************************************************************************
  *@  name      : Elmo_Pre_PVM
  *@  function  : 速度模式预处理函数
  *@  input     : elmoID   取elmo节点ID
  *@  output    : none
********************************************************************************
*/
void Elmo_Pre_PVM(uint8_t elmoID)
{
     uint8_t i = 0;
    
    /* 如果使用广播控制,当前模式不为速度模式,切换为速度模式 */
    if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_SCM)
    {
        if(elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID],100);
        }    
        
        /* 将所有状态变量赋值为SCM */
        elmogroup.CurOPMode = UM_SCM;            
        for(i=0;i<=ELMO_NUM;i++)
        {
            elmo[i].CurOPMode = UM_SCM;            
        }

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
        Elmo_Delay100us_IDx(&elmo[elmoID],10);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID],30);
        
    }  
    /* 使用单个ELMO控制,当前模式不为速度模式,切换为速度模式*/
    else if(elmo[elmoID].CurOPMode != UM_SCM)
    {
        if(elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID],100);
        }

        elmo[elmoID].CurOPMode = UM_SCM;
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
        Elmo_Delay100us_IDx(&elmo[elmoID],10);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID],30);
    
        /* 如果ELMO只挂载了1个，不管怎么改其的模式，elmo[0]的模式和它一样 */                
        if(ELMO_NUM == 1)       
        {
            elmo[0].CurOPMode = UM_SCM;
        }    
        /* 用来判断是否所有电机模式相同，从而确定elmo[0]的模式 */
        else
        {
            for(i=1;i<ELMO_NUM;i++) 
            {
                if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
                {
                    elmo[0].CurOPMode = UM_SCM;
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }                
        }
    }
}

/*
********************************************************************************
  *@  name      : Elmo_PVM
  *@  function  : 单轴速度模式函数
  *@  input     : elmoID   取elmo节点ID
  *@              speed    目标速度(cnt/s)
  *@  output    : 0         函数调用成功
  *@              1         函数调用失败
********************************************************************************
*/


uint8_t Elmo_PVM(uint8_t elmoID, int32_t speed)
{
     uint8_t i = 0;
    
    

//    //调试*******************************
//    #ifdef CASE5_ENABLE
//    if(elmoID == 8 && Im_Arm.Arm_Init_success == 1)
//    {
//        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].mode=1;
//        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].speed=speed;
//        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].pos=0;
//        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].OS_t=OSTime;
//        SLIDEWAY_ELMO_SAVE_count++;
//        if(SLIDEWAY_ELMO_SAVE_count>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count=SLIDEWAY_ELMO_SAVE_Max;  //防止越界
//    }
//        if(elmoID == 10 && Im_Arm.Arm_Init_success == 1)
//    {
//        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].mode=1;
//        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].speed=speed;
//        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].pos=0;
//        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].OS_t=OSTime;
//        SLIDEWAY_ELMO_SAVE_count_Y++;
//        if(SLIDEWAY_ELMO_SAVE_count_Y>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count_Y=SLIDEWAY_ELMO_SAVE_Max;  //防止越界
//    }
//    #endif
        //调试*******************************
    
    
    
    /* 如果使用广播控制,当前模式不为速度模式,切换为速度模式 */
    if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_SCM)
    {
        if(elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID],100);
        }    
        
        /* 将所有状态变量赋值为SCM */
        elmogroup.CurOPMode = UM_SCM;            
        for(i=0;i<=ELMO_NUM;i++)
        {
            elmo[i].CurOPMode = UM_SCM;            
        }

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
        Elmo_Delay100us_IDx(&elmo[elmoID],10);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID],30);
        
    }  
    /* 使用单个ELMO控制,当前模式不为速度模式,切换为速度模式*/
    else if(elmo[elmoID].CurOPMode != UM_SCM)
    {
        if(elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID],100);
        }

        elmo[elmoID].CurOPMode = UM_SCM;
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
        Elmo_Delay100us_IDx(&elmo[elmoID],10);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID],30);
    
        /* 如果ELMO只挂载了1个，不管怎么改其的模式，elmo[0]的模式和它一样 */                
        if(ELMO_NUM == 1)       
        {
            elmo[0].CurOPMode = UM_SCM;
        }    
        /* 用来判断是否所有电机模式相同，从而确定elmo[0]的模式 */
        else
        {
            for(i=1;i<ELMO_NUM;i++) 
            {
                if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
                {
                    elmo[0].CurOPMode = UM_SCM;
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }                
        }
    }

    /* 设置目标速度 */
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"JV", 0, TYPE_INTEGER, speed);
    Elmo_Delay100us_IDx(&elmo[elmoID],10);

    /* 运行速度模式 */
    RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"BG");
    Elmo_Delay100us_IDx(&elmo[elmoID],10);
    return 0;
}

/*
********************************************************************************
  *@  name      : Elmo_RunPPM
  *@  function  : 单轴平滑速度的位置模式
  *@  input     : elmoID    取elmo节点ID
  *@              speed     速度(cnt/s)
  *@              position  目标位置(cnt)
  *@              PPMmode   运行模式
  *@                    POS_ABS // PPM运行方式:绝对位置
  *@                    POS_REL // PPM运行方式:相对位置
  *@  output    : 0         函数调用成功
  *@              1         函数调用失败
********************************************************************************
*/
uint8_t Elmo_PPM(uint8_t elmoID, uint32_t speed, int32_t position, uint8_t PPMmode)
{
    uint8_t i = 0;

    //调试*******************************
//    #ifdef CASE5_ENABLE
//    if(elmoID == 8 && Im_Arm.Arm_Init_success == 1)
//    {
//        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].mode=2;
//        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].speed=0;
//        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].pos=position;
//        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].OS_t=OSTime;
//        SLIDEWAY_ELMO_SAVE_count++;
//        if(SLIDEWAY_ELMO_SAVE_count>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count=SLIDEWAY_ELMO_SAVE_Max;  //防止越界
//    }
//        if(elmoID == 10 && Im_Arm.Arm_Init_success == 1)
//    {
//        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].mode=2;
//        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].speed=speed;
//        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].pos=0;
//        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].OS_t=OSTime;
//        SLIDEWAY_ELMO_SAVE_count_Y++;
//        if(SLIDEWAY_ELMO_SAVE_count_Y>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count_Y=SLIDEWAY_ELMO_SAVE_Max;  //防止越界
//    }
//    #endif
        //调试*******************************

    /* 如果使用广播控制,当前模式不为位置模式,切换为位置模式 */
    if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_PCM)
    {
        if(elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID],90);
        }    
        /* 将所有状态变量赋值为PCM */        
        elmogroup.CurOPMode = UM_PCM;            
        for(i=0;i<=ELMO_NUM;i++)
        {
            elmo[i].CurOPMode = UM_PCM;            
        }

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_PCM);
        Elmo_Delay100us_IDx(&elmo[elmoID],20);
        
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID],30);
    }  
    /* 使用单个ELMO控制,当前模式不为位置模式,切换为位置模式 */
    else if(elmo[elmoID].CurOPMode != UM_PCM)
    {
        if(elmo[elmoID].CurOPMode != UM_IDLE)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
            Elmo_Delay100us_IDx(&elmo[elmoID],90);
        }

        elmo[elmoID].CurOPMode = UM_PCM;
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_PCM);
        Elmo_Delay100us_IDx(&elmo[elmoID],200);

        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID],30);


        /* 如果ELMO只挂载了1个，不管怎么改其的模式，elmo[0]的模式和它一样 */                
        if(ELMO_NUM == 1)      
        {
            elmo[0].CurOPMode = UM_PCM;
        }        
        /* 用来判断是否所有电机模式相同，从而确定elmo[0]的模式 */
        else 
        {
            for(i=1;i<ELMO_NUM;i++) 
            {
                if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
                {
                    elmo[0].CurOPMode = UM_PCM;
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }            
        }
    }
  //RPDO2_Cmd_data(Elmo *elmo, uint8_t *Cmd, uint8_t Index, uint8_t Type, uint32_t Data)
    /* 设置目标速度 */
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"SP", 0, TYPE_INTEGER, speed);
    Elmo_Delay100us_IDx(&elmo[elmoID],20);

    /* 根据运行模式设置位置 */
    if(PPMmode)
    {
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"PA", 0, TYPE_INTEGER, position);
        Elmo_Delay100us_IDx(&elmo[elmoID],20);
    }
    else
    {
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"PR", 0, TYPE_INTEGER, position);
        Elmo_Delay100us_IDx(&elmo[elmoID],20);
    }
    
    /* 运行位置模式 */
    RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"BG");
    Elmo_Delay100us_IDx(&elmo[elmoID],10);
    return 0;
}


/*
********************************************************************************
  *@  name      : Elmo_Close
  *@  function  : 驱动输出关闭，电机靠惯性继续行驶
  *@  input     : elmoID    取elmo节点ID,请勿在底盘控制中调用elmo[1]~elmo[4]        
  *@  output    : 0         函数调用成功
  *@              1         函数调用失败
********************************************************************************
*/
uint8_t Elmo_Close(uint8_t elmoID)
{
    uint8_t i = 0;
    
    /* 如果使用广播控制,将所有ELMO失能 */
    if(elmoID == 0)
    {
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
        Elmo_Delay100us_IDx(&elmo[elmoID],100);
        
        /* 将所有状态变量赋值为IDLE */        
        elmogroup.CurOPMode = UM_IDLE;            
        for(i=0;i<=ELMO_NUM;i++)
        {
            elmo[i].CurOPMode = UM_IDLE;            
        }        
    }
    /* 使用单个ELMO控制 */
    else 
    {
        RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
        Elmo_Delay100us_IDx(&elmo[elmoID],100);
        elmo[elmoID].CurOPMode = UM_IDLE;        

        /* 如果ELMO只挂载了1个，不管怎么改其的模式，elmo[0]的模式和它一样 */            
        if(ELMO_NUM == 1)       
        {
            elmo[0].CurOPMode = UM_IDLE;
        }        
        /* 用来判断是否所有电机模式相同，从而确定elmo[0]的模式 */
        else 
        {
            for(i=1;i<ELMO_NUM;i++)  
            {
                if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
                {
                    elmo[0].CurOPMode = UM_IDLE;
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }            
        }
    }
    return 0;
}


/*
********************************************************************************
  *@  name      : Elmo_Stop
  *@  function  : 刹车，电机抱死
  *@  input     : elmoID      取elmo节点ID,请勿在底盘控制中调用elmo[1]~elmo[4]        
  *@  output    : 0         函数调用成功
  *@              1         函数调用失败
********************************************************************************
*/
uint8_t Elmo_Stop(uint8_t elmoID)
{
    uint8_t i = 0;

        //调试*******************************
    #ifdef CASE5_ENABLE
    if(elmoID == 8 && Im_Arm.Arm_Init_success == 1)
    {
        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].mode=3;
        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].speed=0;
        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].pos=0;
        SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].OS_t=OSTime;
        SLIDEWAY_ELMO_SAVE_count++;
        if(SLIDEWAY_ELMO_SAVE_count>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count=SLIDEWAY_ELMO_SAVE_Max;  //防止越界
    }
        if(elmoID == 10 && Im_Arm.Arm_Init_success == 1)
    {
        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].mode=3;
        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].speed=0;
        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].pos=0;
        SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].OS_t=OSTime;
        SLIDEWAY_ELMO_SAVE_count_Y++;
        if(SLIDEWAY_ELMO_SAVE_count_Y>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count_Y=SLIDEWAY_ELMO_SAVE_Max;  //防止越界
    }
    #endif
    /* 如果CAN总线上轴与轴的控制模式不相同，且发出广播指令，依次关闭 */
    if(elmoID == 0 && elmo[elmoID].CurOPMode == UM_UNC)  
    {
        /* 循环判断个个电机的状态，并关闭每一个电机 */
        for(i=1;i<=ELMO_NUM;i++)            
        {        
            /* 当前模式为释放电机,先打开电机，再抱死 */        
            if (elmo[i].CurOPMode == UM_IDLE) 
            {
                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
                Elmo_Delay100us_IDx(&elmo[elmoID],30);                
            }
            /* 当前模式为力矩模式,先切换为速度模式，再关闭 */
            else if(elmo[i].CurOPMode == UM_TCM)
            {
                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);    
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);            
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);            
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);    
                Elmo_Delay100us_IDx(&elmo[i],200);            

                elmo[i].CurOPMode = UM_SCM;            

                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
                Elmo_Delay100us_IDx(&elmo[i],10);

                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
                Elmo_Delay100us_IDx(&elmo[i],30);

                /* 设置目标速度 */
                RPDO2_Cmd_data(&elmo[i], (uint8_t *)"JV", 0, TYPE_INTEGER, 0);
                Elmo_Delay100us_IDx(&elmo[i],10);

                /* 运行速度模式 */
                RPDO2_Cmd_string(&elmo[i], (uint8_t *)"BG");
                Elmo_Delay100us_IDx(&elmo[i],10);
            }    
                
            RPDO2_Cmd_string(&elmo[i], (uint8_t *)"ST");
            Elmo_Delay100us_IDx(&elmo[i],10);
        }    
        
        /* 用来判断是否所有电机模式相同，从而确定elmo[0]的模式,能够进入这个if的不可能是在CAN线上只有单轴 */
        for(i=1;i<ELMO_NUM;i++)  
        {
            if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
            {
                elmo[0].CurOPMode = UM_SCM;         //如果在STOP模式里面电机模式统一了，只有可能是SCM模式
            }
            else
            {
                elmo[0].CurOPMode = UM_UNC;
                break;
            }
        }                                            
    }
    /* 发出广播指令，但是总线上多轴的控制模式都相同，这时候可以一起关闭 */
    else if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_UNC)  
    {
        /* 当前模式为释放电机,先打开电机，再抱死 */        
        if (elmo[elmoID].CurOPMode == UM_IDLE) 
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
            Elmo_Delay100us_IDx(&elmo[elmoID],30);                
        }
        /* 当前模式为力矩模式,先切换为速度模式，再关闭 */
        else if(elmo[elmoID].CurOPMode == UM_TCM)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);            
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);            
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);            

            /* 广播模式下，所有速度模式都改为SCM */
            for(i=0;i<=ELMO_NUM;i++) 
            {
                elmo[i].CurOPMode = UM_SCM;            
            }
        
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
            Elmo_Delay100us_IDx(&elmo[elmoID],10);

            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
            Elmo_Delay100us_IDx(&elmo[elmoID],30);

            /* 设置目标速度 */
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"JV", 0, TYPE_INTEGER, 0);
            Elmo_Delay100us_IDx(&elmo[elmoID],10);

            /* 运行速度模式 */
            RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"BG");
            Elmo_Delay100us_IDx(&elmo[elmoID],10);
        }    
            
        RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"ST");
        Elmo_Delay100us_IDx(&elmo[elmoID],10);    
        
    }
    /* 使用单个ELMO控制 */
    else    
    {
        /* 当前模式为释放电机,先打开电机，再抱死 */        
        if (elmo[elmoID].CurOPMode == UM_IDLE) 
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
            Elmo_Delay100us_IDx(&elmo[elmoID],30);                
        }
        /* 当前模式为力矩模式,先切换为速度模式，再关闭 */
        else if(elmo[elmoID].CurOPMode == UM_TCM)
        {
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);            
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);            
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);    
            Elmo_Delay100us_IDx(&elmo[elmoID],200);            

            elmo[elmoID].CurOPMode = UM_SCM;            

            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
            Elmo_Delay100us_IDx(&elmo[elmoID],10);

            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
            Elmo_Delay100us_IDx(&elmo[elmoID],30);

            /* 设置目标速度 */
            RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"JV", 0, TYPE_INTEGER, 0);
            Elmo_Delay100us_IDx(&elmo[elmoID],10);

            /* 运行速度模式 */
            RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"BG");
            Elmo_Delay100us_IDx(&elmo[elmoID],10);

            /* 如果ELMO只挂载了1个，不管怎么改其的模式，elmo[0]的模式和它一样 */            
            for(i=1;i<ELMO_NUM;i++)  //用来判断是否所有电机模式相同，从而确定elmo[0]的模式
            {
                if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
                {
                    elmo[0].CurOPMode = UM_SCM;    //如果在STOP模式里面电机模式统一了，只有可能是SCM模式
                }
                else
                {
                    elmo[0].CurOPMode = UM_UNC;
                    break;
                }
            }                
        }    
            
        RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"ST");
        Elmo_Delay100us_IDx(&elmo[elmoID],10);        
    }
    return 0;        
}

/*
********************************************************************************
  *@  name      : Omni_Elmo_Close
  *@  function  : 释放底盘电机
  *@  input     : 无
  *@  output    : 无
********************************************************************************
*/
void Omni_Elmo_Close()
{
    RPDO2_Cmd_data(&elmo[1], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    RPDO2_Cmd_data(&elmo[2], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    RPDO2_Cmd_data(&elmo[3], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    Elmo_Delay100us_IDx(&elmo[1],30);
    Elmo_Delay100us_IDx(&elmo[2],30);
    Elmo_Delay100us_IDx(&elmo[3],30);
    
//    Elmo_Close(1);
//    Elmo_Close(2);
//    Elmo_Close(3);
    
    VelX = 0;
    VelY = 0;
    VelZ = 0;
}

/*
********************************************************************************
  *@  name      : Omni_Elmo_Stop
  *@  function  : 刹车，电机抱死
  *@  input     : 无
  *@  output    : 无
********************************************************************************
*/
void Omni_Elmo_Stop()
{
    RPDO2_Cmd_string(&elmo[1], (uint8_t *)"ST");
    RPDO2_Cmd_string(&elmo[2], (uint8_t *)"ST");
    RPDO2_Cmd_string(&elmo[3], (uint8_t *)"ST");
    Elmo_Delay100us_IDx(&elmo[1],10);
    Elmo_Delay100us_IDx(&elmo[2],10);
    Elmo_Delay100us_IDx(&elmo[3],10);
    
//    Elmo_Stop(1);
//    Elmo_Stop(2);
//    Elmo_Stop(3);
    
    VelX = 0;
    VelY = 0;
    VelZ = 0;
}

/*
********************************************************************************
  *@  name      : Omni_Elmo_PVM
  *@  function  : 三轮速度模式函数
  *@  input     : 无
  *@  output    : 无
********************************************************************************
*/
void Omni_Elmo_PVM()
{
    /* 设置目标速度 */
    RPDO2_Cmd_data(&elmo[1], (uint8_t *)"JV", 0, TYPE_INTEGER, Real2ElmoHead);
    RPDO2_Cmd_data(&elmo[2], (uint8_t *)"JV", 0, TYPE_INTEGER, Real2ElmoLeft);
    RPDO2_Cmd_data(&elmo[3], (uint8_t *)"JV", 0, TYPE_INTEGER, Real2ElmoRight);
    Elmo_Delay100us_IDx(&elmo[1],10);
    Elmo_Delay100us_IDx(&elmo[2],10);
    Elmo_Delay100us_IDx(&elmo[3],10);

    /* 运行速度模式 */
    RPDO2_Cmd_string(&elmo[1], (uint8_t *)"BG");
    RPDO2_Cmd_string(&elmo[2], (uint8_t *)"BG");
    RPDO2_Cmd_string(&elmo[3], (uint8_t *)"BG");
    Elmo_Delay100us_IDx(&elmo[1],10);
    Elmo_Delay100us_IDx(&elmo[2],10);
    Elmo_Delay100us_IDx(&elmo[3],10);
    
//    Elmo_PVM(1, Real2ElmoHead);
//    Elmo_PVM(2, Real2ElmoLeft);
//    Elmo_PVM(3, Real2ElmoRight);
}

/*
********************************************************************************
  *@  name      : Elmo_SetAcc(uint8_t elmoID, uint32_t acc, uint32_t dec)
  *@  function  : 设置速度模式和位置模式的电机加减速
  *@  input     : elmoID     取elmo的节点ID
  *@                 acc        加速度,加速度最大不能超过1000000000,同时应考虑电机性能
  *@                 dec        减速度,减速度最大不能超过1000000000,同时应考虑电机性能
  *@  output    : 0         函数调用成功
  *@              1         函数调用失败
********************************************************************************
*/
uint8_t Elmo_SetAcc(uint8_t elmoID, uint32_t acc, uint32_t dec)
{ 
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    Elmo_Delay100us_IDx(&elmo[elmoID],30);    
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"PM", 0, TYPE_INTEGER, 0x01);
    Elmo_Delay100us_IDx(&elmo[elmoID],10);    
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"AC", 0, TYPE_INTEGER, acc);
    Elmo_Delay100us_IDx(&elmo[elmoID],10);
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"DC", 0, TYPE_INTEGER, dec);
    Elmo_Delay100us_IDx(&elmo[elmoID],10);
    RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    Elmo_Delay100us_IDx(&elmo[elmoID],100);
    return 0;
}


/*
********************************************************************************
  *@  name      : NMTCmd
  *@  function  : 对于CANOPEN的NMT状态设置
  *@  input     : elmo     取elmo的节点ID
                  MNTCmd   NMT指令,NMT_xxx
  *@  output    : None
********************************************************************************
*/
static void NMTCmd(Elmo *elmo, uint8_t MNTCmd)
{
    uint16_t tmp_rear;

    switch(elmo->NodeID)
    {
        #ifdef ID_0 
            case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
        #endif
        #ifdef ID_1
            case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
        #endif
        #ifdef ID_2
            case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
        #endif
        #ifdef ID_3
            case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
        #endif
        #ifdef ID_4
            case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
        #endif
        #ifdef ID_5
            case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
        #endif
        #ifdef ID_6
            case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
        #endif
        #ifdef ID_7
            case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
        #endif
        #ifdef ID_8
            case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
        #endif
        #ifdef ID_9
            case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
        #endif
        #ifdef ID_10
            case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
        #endif
        #ifdef ID_11
            case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
        #endif
        #ifdef ID_12
            case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
        #endif
        #ifdef ID_13
            case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
        #endif
        #ifdef ID_14
            case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
        #endif
        #ifdef ID_15
            case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
        #endif
    }
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if(tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if(tmp_rear == QUEUE_CAN_IDx->Front)
    {
        /* 缓冲区已满 */
        return;
    }
        /* 填充缓冲区 */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   =  COBID_NMT_SERVICE;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     =  2;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] =  MNTCmd;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] =  elmo->NodeID;

    /* 有效数据加1 */
    QUEUE_CAN_IDx->Rear++;
    if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
    {
        QUEUE_CAN_IDx->Rear = 0;
    }
}


/*
********************************************************************************
  *@  name      : RSDO
  *@  function  : 使用下载SDO进行指令发送
  *@  input     : elmo      取elmo节点ID
                                    Index     索引
                  SubIndex  子索引
                  Data      数据
  *@  output    : None
********************************************************************************
*/
static void RSDO(Elmo *elmo, uint16_t Index, uint8_t SubIndex, uint32_t Data)
{
    uint16_t tmp_rear;

    switch(elmo->NodeID)
    {
        #ifdef ID_0 
            case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
        #endif
        #ifdef ID_1
            case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
        #endif
        #ifdef ID_2
            case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
        #endif
        #ifdef ID_3
            case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
        #endif
        #ifdef ID_4
            case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
        #endif
        #ifdef ID_5
            case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
        #endif
        #ifdef ID_6
            case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
        #endif
        #ifdef ID_7
            case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
        #endif
        #ifdef ID_8
            case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
        #endif
        #ifdef ID_9
            case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
        #endif
        #ifdef ID_10
            case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
        #endif
        #ifdef ID_11
            case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
        #endif
        #ifdef ID_12
            case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
        #endif
        #ifdef ID_13
            case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
        #endif
        #ifdef ID_14
            case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
        #endif
        #ifdef ID_15
            case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
        #endif
    }
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if(tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if(tmp_rear == QUEUE_CAN_IDx->Front)
    {
        /* 缓冲区已满 */
        return;
    }
    /* 填充缓冲区 */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   = COBID_RSDO + elmo->NodeID;  // COBID
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     = 8;                          // DLC
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] =  0x22;                       // CS,传输量未确认
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = (Index&0xFF);               // Index
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[2] = (Index&0xFF00)>>8;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[3] = (SubIndex);                 // SubIndex
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[4] = (Data&0xFF);                // Data
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[5] = (Data&0xFF00)>>8;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[6] = (Data&0xFF0000)>>16;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[7] = (Data&0xFF000000)>>24;

    /* 有效数据加1 */
    QUEUE_CAN_IDx->Rear++;
    if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
    {
        QUEUE_CAN_IDx->Rear = 0;
    }
}


/*
********************************************************************************
  *@  name      : RPDO2_Cmd_data
  *@  function  : 使用二进制编码对ELMO发送数据指令
                  CANopen RPDO2 -> 0x2012 二进制输入-设置功能
  *@  input     : elmo   取elmo节点ID
                  Cmd    命令,以字符串形式输入
                  Index  命令的下标           
                  Type   数据类型
                  Data   数据
  *@  output    : None
********************************************************************************
*/
static void RPDO2_Cmd_data(Elmo *elmo, uint8_t *Cmd, uint8_t Index, uint8_t Type, uint32_t Data)
{
    uint16_t tmp_rear;
    switch(elmo->NodeID)
    {
        #ifdef ID_0 
            case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
        #endif
        #ifdef ID_1
            case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
        #endif
        #ifdef ID_2
            case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
        #endif
        #ifdef ID_3
            case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
        #endif
        #ifdef ID_4
            case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
        #endif
        #ifdef ID_5
            case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
        #endif
        #ifdef ID_6
            case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
        #endif
        #ifdef ID_7
            case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
        #endif
        #ifdef ID_8
            case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
        #endif
        #ifdef ID_9
            case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
        #endif
        #ifdef ID_10
            case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
        #endif
        #ifdef ID_11
            case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
        #endif
        #ifdef ID_12
            case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
        #endif
        #ifdef ID_13
            case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
        #endif
        #ifdef ID_14
            case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
        #endif
        #ifdef ID_15
            case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
        #endif
    }
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if(tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if(tmp_rear == QUEUE_CAN_IDx->Front)
    {
        /* 缓冲区已满 */
        return;
    }
    /* 填充缓冲区 */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   = COBID_RPDO2 + elmo->NodeID;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     = 8;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = (*Cmd++);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = (*Cmd);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[2] = (Index);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[3] = (Type<<7);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[4] = (Data&0xFF);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[5] = (Data&0xFF00)>>8;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[6] = (Data&0xFF0000)>>16;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[7] = (Data&0xFF000000)>>24;

    /* 有效数据加1 */
    QUEUE_CAN_IDx->Rear++;
    if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
    {
        QUEUE_CAN_IDx->Rear = 0;
    }
}


/*
********************************************************************************
  *@  name      : RPDO2_Cmd_string
  *@  function  : 使用二进制编码对ELMO发送字符串指令
                  CANopen RPDO2 -> 0x2012 二进制输入-执行功能
  *@  input     : elmo   取elmo节点ID
                                    cmd    命令,以字符串形式输入
  *@  output    : None
********************************************************************************
*/
static void RPDO2_Cmd_string(Elmo *elmo, uint8_t *Cmd)
{
    uint16_t tmp_rear;

    switch(elmo->NodeID)
    {
        #ifdef ID_0 
            case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
        #endif
        #ifdef ID_1
            case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
        #endif
        #ifdef ID_2
            case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
        #endif
        #ifdef ID_3
            case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
        #endif
        #ifdef ID_4
            case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
        #endif
        #ifdef ID_5
            case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
        #endif
        #ifdef ID_6
            case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
        #endif
        #ifdef ID_7
            case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
        #endif
        #ifdef ID_8
            case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
        #endif
        #ifdef ID_9
            case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
        #endif
        #ifdef ID_10
            case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
        #endif
        #ifdef ID_11
            case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
        #endif
        #ifdef ID_12
            case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
        #endif
        #ifdef ID_13
            case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
        #endif
        #ifdef ID_14
            case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
        #endif
        #ifdef ID_15
            case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
        #endif
    }
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if(tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if(tmp_rear == QUEUE_CAN_IDx->Front)
    {
        /* 缓冲区已满 */
        return;
    }

    /* 填充缓冲区 */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   = COBID_RPDO2 + elmo->NodeID;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     = 4;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = (*Cmd++);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = (*Cmd);
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[2] = 0x00;
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[3] = 0x00;
    
    QUEUE_CAN_IDx->Rear++;
    if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
    {
        QUEUE_CAN_IDx->Rear = 0;
    }
 
    
}



/*
********************************************************************************
  *@  name      : CAN_init
  *@  function  : Initialization for CAN
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
//static void CAN_init(CAN_TypeDef* CANx)
//{
//    GPIO_InitTypeDef         GPIO_InitStructure;
//    CAN_InitTypeDef          CAN_InitStructure;
//    CAN_FilterInitTypeDef    CAN_FilterInitStructure;      
//    NVIC_InitTypeDef         NVIC_InitStructure;


//    if( CANx == CAN1 )
//    {  
//        can = CAN1;                //为底层发送报文选择can口
//        /* Enable GPIO clock */
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

//        /* Enable CAN clock */
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

//        /* Configure CAN RX and TX pins */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//        GPIO_Init(GPIOA, &GPIO_InitStructure);

//        /* Connect CAN pins to AF9 */
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);                
//    }
//    else if( CANx == CAN2 )
//    {  
//        can = CAN2;                //为底层发送报文选择can口
//        /* Enable GPIO clock */
//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

//        /* Enable CAN clock */
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

//        /* Configure CAN RX and TX pins */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//        GPIO_Init(GPIOB, &GPIO_InitStructure);

//        /* Connect CAN pins to AF9 */
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);            
//    }

//    /* CAN register init */
//    CAN_DeInit(CANx);
//    CAN_StructInit(&CAN_InitStructure);     

//    /* CAN cell init */
//    CAN_InitStructure.CAN_TTCM = DISABLE;    
//    CAN_InitStructure.CAN_ABOM = DISABLE;    
//    CAN_InitStructure.CAN_AWUM = DISABLE;  
//    CAN_InitStructure.CAN_NART = DISABLE;    
//    CAN_InitStructure.CAN_RFLM = DISABLE;  
//    CAN_InitStructure.CAN_TXFP = ENABLE;    
//    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
//    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;

//    /* Baudrate = 1Mbps (CAN clocked at 42 MHz) */
//    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
//    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
//    CAN_InitStructure.CAN_Prescaler = 3;     //CAN波特率42/(1+9+4)/3=1Mbps
//    CAN_Init(CANx, &CAN_InitStructure);

//    /* CAN filter init */
//    CAN_FilterInitStructure.CAN_FilterNumber = 0;                        
//    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;      
//    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     
//    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000<<5;               
//    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;                     
//    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000<<5;           
//    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0006;               
//    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;               
//    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
//    CAN_FilterInit(&CAN_FilterInitStructure);

//    if( CANx == CAN1 )//用来自检
//    {  
//        /* CAN FIFO0 message pending interrupt enable */ 
//        CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);

//        /* Enable the CAN1 global Interrupt */
//        NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&NVIC_InitStructure);
//    }
//    else if( CANx == CAN2 )
//    {
//        /* CAN FIFO0 message pending interrupt enable */ 
//        CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE);

//        /* Enable the CAN2 global Interrupt */
//        NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//        NVIC_Init(&NVIC_InitStructure);    
//    }
//}


/*
********************************************************************************
  *@  name      : CAN1_RX0_IRQHandler & CAN2_RX0_IRQHandler
  *@  function  : CAN中断处理函数
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
//void CAN1_RX0_IRQHandler()
//{
//    
//}


/*
********************************************************************************
  *@  name      : TIM7_init
  *@  function  : TIM7初始化，使CAN报文每100us发送一次
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
static void TIM7_init(uint8_t PPr, uint8_t SPr)
{
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    NVIC_InitTypeDef          NVIC_InitStructure;

    /* TIM7 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);

    /* Enable the TIM7 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PPr;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SPr;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Time base configuration (TIM7 clocked at 84 MHz)*/
    TIM_TimeBaseStructure.TIM_Period = 840;
    TIM_TimeBaseStructure.TIM_Prescaler = 10-1;                
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;           

    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); 

    /* TIM7 IT enable */
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM7, TIM_FLAG_Update); 

    /* TIM7 enable counter */
    TIM_Cmd(TIM7, ENABLE);
}


/*
********************************************************************************
  *@  name      : TIM7_IRQHandler
  *@  function  : TIM7中断处理函数，每100us用来发送CAN报文
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
void TIM7_IRQHandler(void)
{
//    OS_ERR err;
//    OSIntEnter();
//    taskENTER_CRITICAL();
    /* 100us */
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
    Elmo_SendCmd();
//    taskEXIT_CRITICAL();
//    OSIntExit();  
}


/*
********************************************************************************
  *@  name      : Self_test
  *@  function  : Elmo上电自检
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
int Self_test(void)
{
    uint32_t cnt_delay = 0x3FFF;
    
    do
    {
       __nop();
    }
    while( --cnt_delay );
    if( CAN_Error == 1) //如果初始化失败，关闭TIM7的中断和TIM7
    {
        TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
        TIM_Cmd(TIM7, DISABLE);        
        return 0x80000000;        
    }
    
    Elmo_software_delay_ms(500);
    
    while((Elmo_Init_Flag & ((1<<ELMO_NUM) - 1)) != ((1<<ELMO_NUM) - 1))
    {
       //canopen设备上线之后会发送报文，报文和心跳报文一样，可以检测设备有无上线。
          return Elmo_Init_Flag;
    }
//    CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);
//    CAN_ITConfig(CAN2,CAN_IT_FMP0, DISABLE);
    
    return 0;
}


/*
********************************************************************************
  *@  name      : Variate_init
  *@  function  : 参数初始化
  *@  input     : None
  *@  output    : None
********************************************************************************
*/

static void Variate_init_QUEUE_CAN_IDx(CANQUEUE *QUEUE_CAN_IDx)
{
    memset (&QUEUE_CAN_IDx, 0, sizeof(QUEUE_CAN_IDx));
    memset (elmo, 0, sizeof(Elmo)*(ELMO_NUM + 1));
    memset (&elmogroup, 0, sizeof(Elmo));
    QUEUE_CAN_IDx->Front = 0;
    QUEUE_CAN_IDx->Rear  = 0;

}

static void Variate_init(void)
{
    #ifdef ID_0 
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID0);
    #endif
    #ifdef ID_1
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID1);
    #endif
    #ifdef ID_2
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID2);
    #endif
    #ifdef ID_3
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID3);
    #endif
    #ifdef ID_4
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID4);
    #endif
    #ifdef ID_5
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID5);
    #endif
    #ifdef ID_6
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID6);
    #endif
    #ifdef ID_7
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID7);
    #endif
    #ifdef ID_8
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID8);
    #endif
    #ifdef ID_9
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID9);
    #endif
    #ifdef ID_10
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID10);
    #endif
    #ifdef ID_11
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID11);
    #endif
    #ifdef ID_12
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID12);
    #endif
    #ifdef ID_13
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID13);
    #endif
    #ifdef ID_14
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID14);
    #endif
    #ifdef ID_15
        Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID15);
    #endif
    
    VelX = 0;
    VelY = 0;
    VelZ = 0;
}


/*
********************************************************************************
  *@  name      : Elmo_SendCmd_QUEUE_CAN_IDx
  *@  function  : 发送命令，有可能是CAN报文，也有可能是延迟报文
  *@  input     : CANQUEUE *QUEUE_CAN_IDx 要发送的缓冲区
  *@  output    : None
********************************************************************************
*/



extern uint8_t TIM7_OSTimeSave_Enable;
uint16_t TIM7_OSTime_Count=0;
uint16_t TIM7_OSTime_Count_ten=0;

void Elmo_SendCmd_QUEUE_CAN_IDx(CANQUEUE *QUEUE_CAN_IDx)   //发送指针指向的缓冲区的数据
{
     /* 判断缓冲区是否有数据 */
    if(QUEUE_CAN_IDx->Rear != QUEUE_CAN_IDx->Front)
    {        
        /* 有数据,判断是否是延时命令 */
        if(QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front].COBID == CAN_ID_DELAY)
        {
            /* 是延时指令,判断是否延时完毕 */
            if(QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front].DATA[0] > 1) //发现一个BUG，改了效果好很多
            {
                /* 延时未完,延时时间减1 */
                QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front].DATA[0]--;
            }
            else
            {
                /* 延时完毕,队首加1 */      
                QUEUE_CAN_IDx->Front++;
                if( QUEUE_CAN_IDx->Front >= CAN_BUF_NUM)
                {
                    QUEUE_CAN_IDx->Front = 0;
                }
            }
        }
        else
        {
            /* 不是延时指令,发送CAN报文 */
            Elmo_CANSend( &QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front] );

            /*队首加1*/   
            QUEUE_CAN_IDx->Front++;
            if( QUEUE_CAN_IDx->Front >= CAN_BUF_NUM)
            {
                QUEUE_CAN_IDx->Front = 0;
            }
        }
    }
    else
    {
        /* 队列为空 */ 
        return;   
    }

}


/*
********************************************************************************
  *@  name      : Elmo_SendCmd
  *@  function  : 发送命令，有可能是CAN报文，也有可能是延迟报文
  *@  input     : None
  *@  output    : None
********************************************************************************
*/

static void Elmo_SendCmd(void)
{
    #ifdef ID_0 
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID0);
    #endif
    #ifdef ID_1
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID1);
    #endif
    #ifdef ID_2
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID2);
    #endif
    #ifdef ID_3
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID3);
    #endif
    #ifdef ID_4
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID4);    
    #endif
    #ifdef ID_5
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID5);
    #endif
    #ifdef ID_6
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID6);
    #endif
    #ifdef ID_7
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID7);
    #endif
    #ifdef ID_8
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID8);
    #endif
    #ifdef ID_9
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID9);
    #endif
    #ifdef ID_10
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID10);
    #endif
    #ifdef ID_11
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID11);
    #endif
    #ifdef ID_12
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID12);
    #endif
    #ifdef ID_13
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID13);
    #endif
    #ifdef ID_14
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID14);
    #endif
    #ifdef ID_15
        Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID15);
    #endif
    

}


/*
********************************************************************************
  *@  name      : Elmo_CANSend
  *@  function  : 底层发送CAN报文
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
static void Elmo_CANSend(CANDATA *pCANDATA)
{
    uint8_t i, TransmitMailbox;
    uint32_t cnt_delay;
    CanTxMsg elmoCAN;

    elmoCAN.IDE    =  CAN_ID_STD;                          // 标准帧
    elmoCAN.RTR    =  CAN_RTR_DATA;                        // 数据帧
    elmoCAN.StdId  =  pCANDATA->COBID;                     // COBID
    elmoCAN.DLC    =  pCANDATA->DLC;                       // DLC
    for(i=0;i<elmoCAN.DLC;i++)                             // Data
    {
        elmoCAN.Data[i] = pCANDATA->DATA[i];        
    }
    TransmitMailbox = CAN_Transmit(can, &elmoCAN);                          // 调用发送报文函数   
    
    cnt_delay = 0x2FFF;
    do
    {
       __nop();
    }
    while((CAN_TransmitStatus(can,TransmitMailbox) != CANTXOK) && (--cnt_delay));
    if (cnt_delay <= 0x01 )
        CAN_Error = 1;
    else
        CAN_Error = 0;
    
}


/*
********************************************************************************
  *@  name      : Elmo_Delay100us_IDx
  *@  function  : 延迟报文用来延迟下一个报文的发送，提高发送报文的稳定性
                  N最大为255（N次100us）
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
static void Elmo_Delay100us_IDx( Elmo *elmo , uint8_t N100us)
{
    uint16_t tmp_rear;
    switch(elmo->NodeID)
    {
        #ifdef ID_0 
            case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
        #endif
        #ifdef ID_1
            case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
        #endif
        #ifdef ID_2
            case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
        #endif
        #ifdef ID_3
            case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
        #endif
        #ifdef ID_4
            case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
        #endif
        #ifdef ID_5
            case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
        #endif
        #ifdef ID_6
            case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
        #endif
        #ifdef ID_7
            case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
        #endif
        #ifdef ID_8
            case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
        #endif
        #ifdef ID_9
            case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
        #endif
        #ifdef ID_10
            case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
        #endif
        #ifdef ID_11
            case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
        #endif
        #ifdef ID_12
            case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
        #endif
        #ifdef ID_13
            case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
        #endif
        #ifdef ID_14
            case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
        #endif
        #ifdef ID_15
            case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
        #endif
    }
    
    /* 判断缓冲区是否已满 */
    tmp_rear = QUEUE_CAN_IDx->Rear + 1;
    if(tmp_rear >= CAN_BUF_NUM)
    {
        tmp_rear = 0;
    }
    if(tmp_rear == QUEUE_CAN_IDx->Rear)
    {
        /* 缓冲区已满 */
        return ;
    }

    /* 填充缓冲区 */
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   = CAN_ID_DELAY;        // 延时用的COBID 
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     = 1;                   // DLC为1r 
    QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = N100us;              // 延时,记录时长 

    /* 有效数据加1 */
    QUEUE_CAN_IDx->Rear++;
    if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
    {
        QUEUE_CAN_IDx->Rear = 0;
    }
}


/*
********************************************************************************
  *@  name      : Elmo_software_delay_ms
  *@  function  : Elmo软件延时
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
void Elmo_software_delay_ms(unsigned int t)
{
    int i;
    for( i=0;i<t;i++)
    {
        int a = 41580; //at 168MHz 41580 is ok
         while(a--);
    }
}


/*
********************************************************************************
  *@  name      : f2h
  *@  function  : 将浮点数转化为8字节十六进制数(IEEE754)
  *@  input     : x   浮点数 
  *@  output    : None
********************************************************************************
*/
static uint32_t f2h(float x)
{
    uint32_t *p = (uint32_t *)&x;
    return ((uint32_t)*p);
}
/*
********************************************************************************
  *@  name      : Elmo_Read_POS
  *@  function  : 读取对应的elmo的编码器的数据
  *@  input     : elmoID   无符号8位数据 
  *@  output    : None
********************************************************************************
*/
void Elmo_Read_POS(uint8_t elmoID)
{
    #if ENABLE_SLING_DEBUG
    //开启TPDO2
    RSDO(&elmo[elmoID], 0x1A01, 0x00,1);
    Elmo_Delay100us_IDx(&elmo[elmoID],5);
    #endif
    //读取elmo编码器的数值
    RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"PX");
    Elmo_Delay100us_IDx(&elmo[elmoID],2);
    #if ENABLE_SLING_DEBUG
    //关闭TPDO2
    RSDO(&elmo[elmoID], 0x1A01, 0x00,0);
    Elmo_Delay100us_IDx(&elmo[elmoID],5);
    #endif
}

/**
  * @brief  Elmo_Read_POS,读取对应的elmo的编码器的数据
  * @param  elmoID   无符号8位数据
  *         pEncData int32_t 型指针，指向返回数据
  * @retval 0   ：成功
  *         1   ：失败
  * @time   RSDORead
  *         -   1ms间隔连续发送1000次，最大读取(发送完命令至接收到数据)时间未超过 80000/84us = 952.38us
  *         -   此处等待1.5ms
  * @note   中断中读取
  *         RxMsg.StdId == COBID_TSDO + ID;
  *         if ((RxMsg.Data[2]<<8 | RxMsg.Data[1]) == 0x6064)
  *             memcpy(&Encoder_Data, &RxMsg.Data[4],sizeof(int32_t));
  *         目前仅支持 ID：MOTOR_ID_HIT_MAIN，MOTOR_ID_HIT_SERVE ！！！
  */
//uint32_t readElmo_Pos(uint8_t elmoID, int32_t *pEncData)
//{
//    uint32_t err = 1;
//    /* 现在始终开启CAN1接收中断，故不必再开启CAN1接收中断 */
//    //CAN1->IER |= CAN_IT_FMP0;

//    /* 读取elmo编码器的数值 */
//    Elmo_Read_POS(elmoID);
//    
////    if(elmoID == MOTOR_ID_ARM_X)
////    {
////        if(xSemaphoreTake(PosArmXSem, 3*portTICK_MS) == pdTRUE)   //1.5ms  CAN线阻塞
////        {
////            *pEncData = EncoderData_1;
////            err = 0;
////        }
////        else
////            err = 1;
////    }
////    else if(elmoID == MOTOR_ID_CLIP)
////    {
////        if(xSemaphoreTake(PosClipSem, 3*portTICK_MS) == pdTRUE)   //1.5ms
////        {
////            *pEncData = EncoderData_3;
////            err = 0;
////        }
////        else
////            err = 1;
////    }
//    
//    /* 关闭CAN1接收中断 */
//    //CAN1->IER &= ~CAN_IT_FMP0;
//    return err;
//}
/*
********************************************************************************
  *@  name      : Elmo_Set_POS
  *@  function  : 读取对应的elmo的编码器的数据
  *@  input     : elmoID   无符号8位数据 
  *@  output    : None
********************************************************************************
*/
void Elmo_Set_POS(uint8_t elmoID,int32_t POS)
{
      //关闭电机，这是设置位置值的先决条件
      RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
        Elmo_Delay100us_IDx(&elmo[elmoID],90);
    
      //对编码器的值进行设置
      RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"PX", 0, TYPE_INTEGER, POS);
        Elmo_Delay100us_IDx(&elmo[elmoID],90);
    
      //开启电机
      RPDO2_Cmd_data(&elmo[elmoID], (uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
        Elmo_Delay100us_IDx(&elmo[elmoID],90);
      //更新当前电机状态
      elmo[elmoID].CurOPMode = UM_UNC;
    

}

/*
********************************************************************************
  *@  name      : Elmo_Read_ACT_CUR
  *@  function  : 读取对应的elmo的有效电流
  *@  input     : elmoID   无符号8位数据 
  *@  output    : None
********************************************************************************
*/
void Elmo_Read_ACT_CUR(uint8_t elmoID)
{
    //开启TPDO2
    RSDO(&elmo[0], 0x1A01, 0x00,1);
    Elmo_Delay100us_IDx(&elmo[elmoID],50);
    //读取elmo编码器的数值
    RPDO2_Cmd_string(&elmo[elmoID], (uint8_t *)"IQ");
    Elmo_Delay100us_IDx(&elmo[elmoID],20);
    //关闭TPDO2
    RSDO(&elmo[0], 0x1A01, 0x00,0);
    Elmo_Delay100us_IDx(&elmo[elmoID],50);

}
