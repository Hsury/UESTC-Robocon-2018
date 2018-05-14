#include "Tasks.h"

SemaphoreHandle_t PauseSemaphore;
SemaphoreHandle_t ArrivedSemaphore;

TaskHandle_t FlashTask_Handler;
TaskHandle_t BeepTask_Handler;
TaskHandle_t ReportTask_Handler;
TaskHandle_t WirelessTask_Handler;
TaskHandle_t MoveTask_Handler;
TaskHandle_t FlowTask_Handler;

void FlashTask(void *pvParameters)
{
    while (1)
    {
        RGBShowRed();
        delay_ms(333);
        RGBShowGreen();
        delay_ms(333);
        RGBShowBlue();
        delay_ms(333);
    }
}

void BeepTask(void *pvParameters)
{
    while (1)
    {
        Buzzer_On();
        delay_ms(250);
        Buzzer_Off();
        delay_ms(250);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void ReportTask(void *pvParameters)
{
    while (1)
    {
        Probe_ReportOnline();
        #if ENABLE_ACC_REPORT
        PackUp(0xA1, (uint8_t*)(&RealAccX));
        PackUp(0xA2, (uint8_t*)(&RealAccY));
        PackUp(0xA3, (uint8_t*)(&RealAccZ));
        #endif
        #if ENABLE_VEL_REPORT
        PackUp(0xA4, (uint8_t*)(&RealVelX));
        PackUp(0xA5, (uint8_t*)(&RealVelY));
        PackUp(0xA6, (uint8_t*)(&RealVelZ));
        #endif
        #if ENABLE_POS_REPORT
        PackUp(0xA7, (uint8_t*)(&PosX));
        PackUp(0xA8, (uint8_t*)(&PosY));
        PackUp(0xA9, (uint8_t*)(&PosZ));
        #endif
        delay_ms(500);
    }
}

void WirelessTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //if (AirUART_Send_And_Wait(&Zone, 1, 5)) xTaskNotifyGive(BeepTask_Handler); //433MHz 透传模块
        for (uint8_t i = 0; i < 10; i++)
        {
            Printf(ESP8266, "%c", 0xA0);
            delay_ms(5);
        }
    }
}

void MoveTask(void *pvParameters)
{
    uint8_t Path = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint8_t NewPath;
    PathParam_t* PathParam;
    while (1)
    {
        uint32_t StartTS = millis();
        uint32_t PauseDuration = 0;
        bool Arrived = false;
        GEErrorTS = StartTS;
        Move_PID_Start();
        switch (Path)
        {
            case LOCKPOINT: GoalX = PosX; GoalY = PosY; GoalZ = PosZ; goto LockPoint;
            case SZ_TZ1: PathParam = &PathParam_SZ_TZ1; break;
            case TZ1_TZ2: PathParam = &PathParam_TZ1_TZ2; break;
            case TZ2_TZ3: PathParam = &PathParam_TZ2_TZ3; break;
            case SZ_TZ2: PathParam = &PathParam_SZ_TZ2; break;
            case SZ_TZ3: PathParam = &PathParam_SZ_TZ3; break;
            case TZ1_SZ: PathParam = &PathParam_TZ1_SZ; break;
            case TZ2_SZ: PathParam = &PathParam_TZ2_SZ; break;
            case TZ3_SZ: PathParam = &PathParam_TZ3_SZ; break;
            default: Path = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); continue;
        }
        if (DeltaPos(PosX - PathParam->P0[0], PosY - PathParam->P0[1]) > 0.50 || fabs(PosZ - PathParam->P0[2]) > 30) //路径启动位置保护
        {
            while (1)
            {
                Buzzer_On(); delay_ms(500); Buzzer_Off(); delay_ms(500);
                Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(750);
            }
        }
        Move_PID_SetTunings(PathParam);
        Move_PID_SetLimits(PathParam);
        while (millis() - StartTS - PauseDuration <= PathParam->Duration)
        {
            if (xSemaphoreTake(PauseSemaphore, 0)) //底盘运动暂停
            {
                uint32_t PauseTS = millis();
                xTaskNotifyGive(BeepTask_Handler);
                Omni_Elmo_Stop();
                xSemaphoreTake(PauseSemaphore, portMAX_DELAY);
                xTaskNotifyGive(BeepTask_Handler);
                PauseDuration += millis() - PauseTS;
            }
            if (!GyroEncoder_ReadFlag()) //陀螺仪与码盘数据不可用
            {
                Buzzer_On();
                Omni_Elmo_Stop();
                while (!GyroEncoder_ReadFlag()) delay_ms(5);
                Buzzer_Off();
                PauseDuration += GEErrorDuration;
            }
            Move_SetGoal(PathParam, millis() - StartTS - PauseDuration);
            Move_PID_Compute();
            Move_PID_Apply();
            //Move_AddConstantFiducial(1);
            //Move_AddDerivativeFiducial(PathParam, millis() - StartTS, 0.25, 0.2, 0.8);
            Omni_Elmo_PVM();
            //Printf(ESP8266, "T = %u, Err = (%.3f, %.3f, %.3f), Vel = (%.3f, %.3f, %.3f)\r\n", millis() - StartTS, PosX - GoalX, PosY - GoalY, PosZ - GoalZ, VelX, VelY, VelZ);
            delay_ms(5);
        }
        if (!Move_Relocate()) Buzzer_On(); //尝试重定位，不成功则拉高蜂鸣器
        LockPoint:
        //Printf(ESP8266, "Enter Lock Point Mode\r\n");
        Move_PID_SetTunings(&PathParam_LockPoint);
        Move_PID_SetLimits(&PathParam_LockPoint);
        do
        {
            if (!Arrived && DeltaPos(GoalX - PosX, GoalY - PosY) <= 0.03 && fabs(GoalZ - PosZ) <= 0.5 && DeltaPos(RealVelX, RealVelY) <= 0.20)
            {
                if (Path != LOCKPOINT && PIN_WHEN_ARRIVE)
                {
                    Omni_Elmo_Stop();
                    Pin();
                }
                Move_UpdateZone();
                xSemaphoreGive(ArrivedSemaphore);
                Cradle_ArriveNotify(Zone);
                Probe_SetTimer(Zone - 1, millis() - StartTS);
                //Printf(ESP8266, "Arrived\r\n");
                xTaskNotifyGive(BeepTask_Handler);
                Arrived = true;
            }
            if (Path == LOCKPOINT || !PIN_WHEN_ARRIVE || !Arrived)
            {
                if (!GyroEncoder_ReadFlag()) //陀螺仪与码盘数据不可用
                {
                    Buzzer_On();
                    Omni_Elmo_Stop();
                    while (!GyroEncoder_ReadFlag()) delay_ms(5);
                    Buzzer_Off();
                }
                Move_PID_Compute();
                Move_PID_Apply();
                //Move_DeadzoneCtrl(0.01, 0.1);
                Omni_Elmo_PVM();
            }
            NewPath = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
            //Printf(ESP8266, "T = %u, Goal = (%.3f, %.3f, %.3f), Err = (%.3f, %.3f, %.3f), Vel = (%.3f, %.3f, %.3f)\r\n", millis() - StartTS, GoalX, GoalY, GoalZ, PosX - GoalX, PosY - GoalY, PosZ - GoalZ, VelX, VelY, VelZ);
        }
        while (!NewPath);
        Path = NewPath;
        Move_PID_Stop();
        if (Path != LOCKPOINT && PIN_WHEN_ARRIVE) Unpin();
        //xTaskNotifyGive(BeepTask_Handler);
        for (uint8_t i = 0; i < 3; i++) {Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(750);}
    }
}

void FlowTask(void *pvParameters)
{
    PauseSemaphore = xSemaphoreCreateBinary();
    ArrivedSemaphore = xSemaphoreCreateBinary();
    
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != GET_READY);                // 等待[预备]指令
    
    xTaskNotifyGive(BeepTask_Handler);                                           // 蜂鸣器叫一下
    Cradle_ReturnNotify();                                                       // 通知二维平台初始化
    delay_ms(50);                                                                // 按键消抖
    ulTaskNotifyTake(pdTRUE, 0);                                                 // 清除残余的任务通知    
    
    while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)) != GET_READY)           // 等待第二次[预备]指令
    {
        xTaskNotifyGive(BeepTask_Handler);                                       // 蜂鸣器2s叫一下
    }
    
    xTaskNotifyGive(BeepTask_Handler);                                           // 蜂鸣器叫一下
    Move_Init();                                                                 // 底盘电机抱死
    GyroEncoder_Off();                                                           // 暂停接收陀螺仪与码盘的数据
    delay_ms(10);                                                                // 等待0.01s，确保再次进入中断时不会覆盖已设置的坐标
    PosX = PathParam_SZ_TZ1.P0[0];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    PosY = PathParam_SZ_TZ1.P0[1];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    PosZ = PathParam_SZ_TZ1.P0[2];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    GyroEncoder_SetPos();                                                        // 更新码盘的坐标信息
    GyroEncoder_SetAng();                                                        // 更新陀螺仪的坐标信息
    GyroEncoder_On();                                                            // 继续接收陀螺仪与码盘的数据
    ulTaskNotifyTake(pdTRUE, 0);                                                 // 清除残余的任务通知
    
    while (1)
    {
        switch (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)))                   // 等待第三次[预备]或[发车]指令
        {
            case GET_READY:                                                      // 执行陀螺仪重新采样
            xTaskNotifyGive(BeepTask_Handler);                                   // 蜂鸣器叫一下
            Move_Init();                                                         // 底盘电机抱死
            GyroEncoder_Off();                                                   // 暂停接收陀螺仪与码盘的数据
            GyroEncoder_Reset();                                                 // 陀螺仪重新采样
            delay_ms(3000);                                                      // 等待3s，直到陀螺仪采样完毕
            PosX = PathParam_SZ_TZ1.P0[0];                                       // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
            PosY = PathParam_SZ_TZ1.P0[1];                                       // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
            PosZ = PathParam_SZ_TZ1.P0[2];                                       // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
            GyroEncoder_SetPos();                                                // 更新码盘的坐标信息
            GyroEncoder_SetAng();                                                // 更新陀螺仪的坐标信息
            GyroEncoder_On();                                                    // 继续接收陀螺仪与码盘的数据
            ulTaskNotifyTake(pdTRUE, 0);                                         // 清除残余的任务通知
            break;
            
            case LAUNCH:                                                         // 执行发车
            goto Setout;
        }
        xTaskNotifyGive(BeepTask_Handler);                                       // 蜂鸣器1s叫一下
    }
    
    Setout:
    xTaskNotifyGive(BeepTask_Handler);                                           // 蜂鸣器叫一下
    delay_ms(1000);
    if (!Retry)                                                                  // 如果处于重试状态，在这里进行路径分支选择
    {
        xTaskNotify(MoveTask_Handler, SZ_TZ1, eSetValueWithOverwrite);           // 底盘开始跑SZ->TZ1的路径
        xSemaphoreTake(ArrivedSemaphore, 0);                                     // 清除残余的底盘到达信号量
        xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                         // 等待底盘到达信号量被释放
        Zone = TZ1;                                                              // 将区域变量设置为TZ1
        ulTaskNotifyTake(pdTRUE, 0);                                             // 清除残余的任务通知
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != MOVE_ON);              // 等待[继续]指令
        
        xTaskNotify(MoveTask_Handler, TZ1_TZ2, eSetValueWithOverwrite);          // 底盘开始跑TZ1->TZ2的路径
        xSemaphoreTake(ArrivedSemaphore, 0);                                     // 清除残余的底盘到达信号量
        xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                         // 等待底盘到达信号量被释放
        Zone = TZ2;                                                              // 将区域变量设置为TZ2
        ulTaskNotifyTake(pdTRUE, 0);                                             // 清除残余的任务通知
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != MOVE_ON);              // 等待[继续]指令
    }
    else
    {
        xTaskNotify(MoveTask_Handler, SZ_TZ2, eSetValueWithOverwrite);           // 底盘开始跑SZ->TZ2的路径
        xSemaphoreTake(ArrivedSemaphore, 0);                                     // 清除残余的底盘到达信号量
        xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                         // 等待底盘到达信号量被释放
        Zone = TZ2;                                                              // 将区域变量设置为TZ2
        ulTaskNotifyTake(pdTRUE, 0);                                             // 清除残余的任务通知
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != MOVE_ON);              // 等待[继续]指令
    }
    xTaskNotify(MoveTask_Handler, TZ2_TZ3, eSetValueWithOverwrite);              // 底盘开始跑TZ2->TZ3的路径
    xSemaphoreTake(ArrivedSemaphore, 0);                                         // 清除残余的底盘到达信号量
    xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                             // 等待底盘到达信号量被释放
    Zone = TZ3;                                                                  // 将区域变量设置为TZ3
    ulTaskNotifyTake(pdTRUE, 0);                                                 // 清除残余的任务通知
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != LAUNCH);                   // 等待[回归(发车)]指令
    
    xTaskNotify(MoveTask_Handler, TZ3_SZ, eSetValueWithOverwrite);               // 底盘开始跑TZ3->SZ的路径
    xSemaphoreTake(ArrivedSemaphore, 0);                                         // 清除残余的底盘到达信号量
    xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                             // 等待底盘到达信号量被释放
    
    vTaskDelete(NULL);
}
