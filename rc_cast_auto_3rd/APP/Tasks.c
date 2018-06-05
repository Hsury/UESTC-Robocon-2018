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
        delay_ms(50);
    }
}

void WirelessTask(void *pvParameters)
{
    //uint8_t TxData = 0x00;
    while (1)
    {
        uint8_t tmp = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(25));
        /*
        if (tmp && tmp < 0x40) //MR => AR
        {
            if (tmp == 0x11 || tmp == 0x12 || tmp == 0x21) Cradle_RetryNotify(tmp);
            Printf(ESP8266, "%c", tmp + 0x80);
        }
        else if (tmp && tmp < 0x80) TxData = tmp; //AR => MR
        else if (tmp > 0x80 && tmp - 0x80 == TxData) TxData = 0x00; //收到当前在发数据的确认响应
        if (TxData) Printf(ESP8266, "%c", TxData);
        //if (AirUART_Send_And_Wait(&Zone, 1, 5)) xTaskNotifyGive(BeepTask_Handler); //433MHz 透传模块
        */
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
        bool Relocated = false;
        bool Arrived = false;
        bool Virgin = true;
        GEErrorTS = StartTS;
        Printf(ESP8266, "Path %u was loaded\r\n", Path);
        switch (Path)
        {
            case SZ_TZ1: PathParam = &PathParam_SZ_TZ1; break;
            case TZ1_TZ2: PathParam = &PathParam_TZ1_TZ2; break;
            case TZ2_TZ3: PathParam = &PathParam_TZ2_TZ3; break;
            case SZ_TZ2: PathParam = &PathParam_SZ_TZ2; break;
            case SZ_TZ3: PathParam = &PathParam_SZ_TZ3; break;
            case TZ3_SZ: PathParam = &PathParam_TZ3_SZ; break;
            case LOCKPOINT: Move_PID_SetTunings(&PathParam_LockPoint); Move_PID_SetLimits(&PathParam_LockPoint); goto LockPoint;
            case DASH: Move_PID_SetTunings(&PathParam_Dash); Move_PID_SetLimits(&PathParam_Dash); goto LockPoint;
            default: Path = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); continue;
        }
        while (fabs(PosX - PathParam->P0[0]) > 0.25 || fabs(PosY - PathParam->P0[1]) > 0.25 || fabs(PosZ - PathParam->P0[2]) > 30.00) //路径启动位置保护
        {
            Buzzer_On(); delay_ms(500); Buzzer_Off(); delay_ms(500);
            Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(750);
        }
        Move_PID_SetTunings(PathParam);
        Move_PID_SetLimits(PathParam);
        while (millis() - StartTS - PauseDuration <= PathParam->Duration || fabs(GoalX - PosX) > 0.25 || fabs(GoalY - PosY) > 0.25 || fabs(GoalZ - PosZ) > 5.00)
        {
            if (!Relocated && millis() - StartTS - PauseDuration > PathParam->Duration * 0.9 && fabs(GoalX - PosX) <= 0.50 && fabs(GoalY - PosY) <= 0.50) Relocated = Move_Relocate(); //描点过程中尝试重定位
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
            //Move_AddDerivativeFiducial(PathParam, millis() - StartTS - PauseDuration, 0.25, 0.2, 0.8);
            Omni_Elmo_PVM();
            Printf(ESP8266, "%ums, P(%.2f, %.2f, %.2f), G(%.2f, %.2f, %.2f), E(%.2f, %.2f, %.2f), GV(%.2f, %.2f, %.2f), RV(%.2f, %.2f, %.2f)\r\n", millis() - StartTS, PosX, PosY, PosZ, GoalX, GoalY, GoalZ, PosX - GoalX, PosY - GoalY, PosZ - GoalZ, VelX, VelY, VelZ, RealVelX, RealVelY, RealVelZ);
            delay_ms(5);
        }
        Relocated = Move_Relocate(); //描点结束后再次尝试重定位，不成功则拉高蜂鸣器
        if (!Relocated) Buzzer_On();
        Move_PID_SetTunings(&PathParam_LockPoint);
        Move_PID_SetLimits(&PathParam_LockPoint);
        LockPoint:
        Printf(ESP8266, "Switched to Lock-Point mode\r\n");
        do
        {
            if (Virgin && Path != LOCKPOINT && Path != DASH && !Relocated) Relocated = Move_Relocate(); //若该段路径在锁点部分未重定位成功，则持续重试
            if (Virgin && fabs(RealVelX) <= 0.20f && fabs(RealVelY) <= 0.20f && fabs(RealVelZ) <= 20.00f && ((fabs(GoalX - PosX) <= 0.03 && fabs(GoalY - PosY) <= 0.03 && (Path == DASH || fabs(GoalZ - PosZ) <= 0.5)) || (Path != LOCKPOINT && Path != DASH && millis() - StartTS - PauseDuration > PathParam->Timeout)))
            {
                Buzzer_On();
                Omni_Elmo_Stop();
                if (PIN_WHEN_ARRIVE) Pin();
                Arrived = true;
                Probe_SetTimer(Path - 1, millis() - StartTS - PauseDuration);
                //if (Path == TZ1_TZ2 || Path == TZ2_TZ3) delay_ms(200);
                Move_UpdateZone();
                if (Path != LOCKPOINT && Path != DASH)
                {
                    Cradle_ArriveNotify(Zone);
                    xSemaphoreGive(ArrivedSemaphore);
                }
                else if (Path == LOCKPOINT) Cradle_AdjustNotify();
                Virgin = false;
                Buzzer_Off();
                Printf(ESP8266, "First-Blood was taken at %ums\r\n", millis() - StartTS);
            }
            if (!Arrived && Path != DASH && fabs(GoalX - PosX) <= 0.03 && fabs(GoalY - PosY) <= 0.03 && fabs(GoalZ - PosZ) <= 0.5) //进入到达误差容许范围
            {
                Omni_Elmo_Stop();
                if (PIN_WHEN_ARRIVE) Pin();
                Arrived = true;
                Printf(ESP8266, "Lock-Point stopped at %ums\r\n", millis() - StartTS);
            }
            else if (ADJUST_AFTER_PIN && Arrived && Path != DASH && (fabs(GoalX - PosX) > 0.05 || fabs(GoalY - PosY) > 0.05 || fabs(GoalZ - PosZ) > 0.8)) //超出校准误差容许范围
            {
                if (PIN_WHEN_ARRIVE) Unpin();
                Arrived = false;
                Printf(ESP8266, "Lock-Point started at %ums\r\n", millis() - StartTS);
            }
            if (!Arrived) //执行锁点
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
                Printf(ESP8266, "%ums, P(%.2f, %.2f, %.2f), G(%.2f, %.2f, %.2f), E(%.2f, %.2f, %.2f), GV(%.2f, %.2f, %.2f), RV(%.2f, %.2f, %.2f)\r\n", millis() - StartTS, PosX, PosY, PosZ, GoalX, GoalY, GoalZ, PosX - GoalX, PosY - GoalY, PosZ - GoalZ, VelX, VelY, VelZ, RealVelX, RealVelY, RealVelZ);
            }
            NewPath = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        }
        while (!NewPath);
        Path = NewPath;
        Omni_Elmo_Stop();
        if (PIN_WHEN_ARRIVE) Unpin();
        xTaskNotifyGive(BeepTask_Handler);
        //for (uint8_t i = 0; i < 3; i++) {Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(750);}
    }
}

void FlowTask(void *pvParameters)
{
    PauseSemaphore = xSemaphoreCreateBinary();
    ArrivedSemaphore = xSemaphoreCreateBinary();
    
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != GET_READY);                // 等待[预备]指令
    
    xTaskNotifyGive(BeepTask_Handler);                                           // 蜂鸣器叫一下
    Elmo_Reinit(0);                                                              // 底盘抱死
    Pin();                                                                       // 吸盘吸地
    Cradle_ReturnNotify();                                                       // 通知二维平台初始化
    delay_ms(1000);                                                              // 等待二维平台复位
    Unpin();                                                                     // 吸盘释放
    Elmo_Close(0);                                                               // 底盘释放
    ulTaskNotifyTake(pdTRUE, 0);                                                 // 清除残余的任务通知    
    
    while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)) != GET_READY)           // 等待第二次[预备]指令
    {
        xTaskNotifyGive(BeepTask_Handler);                                       // 蜂鸣器2s叫一下
    }
    
    xTaskNotifyGive(BeepTask_Handler);                                           // 蜂鸣器叫一下
    Move_Init();                                                                 // 底盘电机抱死
    GyroEncoder_Off();                                                           // 暂停接收陀螺仪与码盘的数据
    delay_ms(5);                                                                 // 等待5ms，确保再次进入中断时不会覆盖已设置的坐标
    PosX = PathParam_SZ_TZ1.P0[0];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    PosY = PathParam_SZ_TZ1.P0[1];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    PosZ = PathParam_SZ_TZ1.P0[2];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    GyroEncoder_SetPos();                                                        // 更新码盘的坐标信息
    GyroEncoder_SetAng();                                                        // 更新陀螺仪的坐标信息
    delay_ms(5);                                                                 // 等待5ms，确保设置的坐标已经在陀螺仪与码盘上生效
    GyroEncoder_On();                                                            // 继续接收陀螺仪与码盘的数据
    delay_ms(200);                                                               // 按键消抖
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
            delay_ms(10);                                                        // 等待10ms，确保设置的坐标已经在陀螺仪与码盘上生效
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
    switch (Retry)                                                               // 如果处于重试状态，在这里进行路径分支选择
    {
        case 0:                                                                  // >正常发车
        xTaskNotify(MoveTask_Handler, SZ_TZ1, eSetValueWithOverwrite);           // 底盘开始跑SZ->TZ1的路径
        xSemaphoreTake(ArrivedSemaphore, 0);                                     // 清除残余的底盘到达信号量
        xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                         // 等待底盘到达信号量被释放
        ulTaskNotifyTake(pdTRUE, 0);                                             // 清除残余的任务通知
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != MOVE_ON);              // 等待[继续]指令
        xTaskNotify(MoveTask_Handler, TZ1_TZ2, eSetValueWithOverwrite);          // 底盘开始跑TZ1->TZ2的路径
        xSemaphoreTake(ArrivedSemaphore, 0);                                     // 清除残余的底盘到达信号量
        xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                         // 等待底盘到达信号量被释放
        ulTaskNotifyTake(pdTRUE, 0);                                             // 清除残余的任务通知
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != MOVE_ON);              // 等待[继续]指令
        xTaskNotify(MoveTask_Handler, TZ2_TZ3, eSetValueWithOverwrite);          // 底盘开始跑TZ2->TZ3的路径
        xSemaphoreTake(ArrivedSemaphore, 0);                                     // 清除残余的底盘到达信号量
        xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                         // 等待底盘到达信号量被释放
        break;
        
        case 1:                                                                  // >重试TZ2
        xTaskNotify(MoveTask_Handler, SZ_TZ2, eSetValueWithOverwrite);           // 底盘开始跑SZ->TZ2的路径
        xSemaphoreTake(ArrivedSemaphore, 0);                                     // 清除残余的底盘到达信号量
        xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                         // 等待底盘到达信号量被释放
        ulTaskNotifyTake(pdTRUE, 0);                                             // 清除残余的任务通知
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != MOVE_ON);              // 等待[继续]指令
        xTaskNotify(MoveTask_Handler, TZ2_TZ3, eSetValueWithOverwrite);          // 底盘开始跑TZ2->TZ3的路径
        xSemaphoreTake(ArrivedSemaphore, 0);                                     // 清除残余的底盘到达信号量
        xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                         // 等待底盘到达信号量被释放
        break;
        
        case 2:                                                                  // >重试TZ3
        xTaskNotify(MoveTask_Handler, SZ_TZ3, eSetValueWithOverwrite);           // 底盘开始跑SZ->TZ3的路径
        xSemaphoreTake(ArrivedSemaphore, 0);                                     // 清除残余的底盘到达信号量
        xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                         // 等待底盘到达信号量被释放
        break;
    }
    ulTaskNotifyTake(pdTRUE, 0);                                                 // 清除残余的任务通知
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != LAUNCH);                   // 等待[回归(发车)]指令
    
    xTaskNotify(MoveTask_Handler, TZ3_SZ, eSetValueWithOverwrite);               // 底盘开始跑TZ3->SZ的路径
    xSemaphoreTake(ArrivedSemaphore, 0);                                         // 清除残余的底盘到达信号量
    xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                             // 等待底盘到达信号量被释放
    
    NVIC_SystemReset();                                                          // 复位主控
    vTaskDelete(NULL);
}
