#include "Tasks.h"

SemaphoreHandle_t ArrivedSemaphore;
SemaphoreHandle_t FiredSemaphore;

TaskHandle_t FlashTask_Handler;
TaskHandle_t BeepTask_Handler;
TaskHandle_t ReportTask_Handler;
TaskHandle_t MoveTask_Handler;
TaskHandle_t WirelessTask_Handler;
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
        //printf("Serial Test\r\n");
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
    delay_ms(100);
    Probe_ReportOnline();
    while (1)
    {
        delay_ms(5);
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
    }
}

void MoveTask(void *pvParameters)
{
    uint8_t Path = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    PathParam_t* PathParam;
    while (1)
    {
        switch (Path)
        {
            case SZ_TZ1: PathParam = &PathParam_SZ_TZ1; break;
            case TZ1_TZ2: PathParam = &PathParam_TZ1_TZ2; break;
            case TZ2_TZ3: PathParam = &PathParam_TZ2_TZ3; break;
//            case TZ3_TZ2: PathParam = &PathParam_TZ3_TZ2; break;
            case SZ_TZ2: PathParam = &PathParam_SZ_TZ2; break;
            default: continue;
        }
        Move_PID_SetTunings(PathParam);
        Move_PID_SetLimits(PathParam);
        Move_PID_Start();
        uint32_t StartTS = millis();
        bool Arrived = false;
        while (millis() - StartTS <= PathParam->Duration)
        {
            Move_UpdateZone();
            Move_SetGoal(PathParam, millis() - StartTS);
            Move_PID_Compute();
            Move_PID_Apply();
            //Printf(ESP8266, "T=%u, X=%f, Y=%f, Z=%f\r\n", millis() - StartTS, VelX, VelY, VelZ);
            //Move_AddConstantFiducial(1);
            Move_AddDerivativeFiducial(PathParam, millis() - StartTS, 0.775, 0.125, 0.825);
            Omni_Elmo_PVM();
            delay_ms(5);
        }
        Move_PID_SetTunings(&PathParam_LockPoint);
        do
        {
            Path = ulTaskNotifyTake(pdTRUE, 0);
            Move_UpdateZone();
            Move_PID_Compute();
            Move_PID_Apply();
            Move_DeadzoneCtrl(0.01, 0.2);
            Omni_Elmo_PVM();
            delay_ms(5);
            if (!Arrived && DeltaPos(GoalX - PosX, GoalY - PosY) <= 0.025) //Zone == (Path <= TZ2_TZ3 ? Path : TZ2)
            {
                xSemaphoreGive(ArrivedSemaphore);
                Cradle_ArriveNotify(Zone);
                Probe_SetTimer(Zone - 1, millis() - StartTS);
                xTaskNotifyGive(BeepTask_Handler);
                Arrived = true;
            }
        }
        while (!Path);
        Move_PID_Stop();
        xTaskNotifyGive(BeepTask_Handler);
    }
}

void WirelessTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (AirUART_Send_And_Wait(&Zone, 1, 5)) xTaskNotifyGive(BeepTask_Handler);
    }
}

void FlowTask(void *pvParameters)
{
    ArrivedSemaphore = xSemaphoreCreateBinary();
    FiredSemaphore = xSemaphoreCreateBinary();
    
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != GET_READY);                // 等待[预备]指令
    xTaskNotifyGive(BeepTask_Handler);                                           // 蜂鸣器叫一下
    Move_Init();                                                                 // 底盘电机抱死
    GyroEncoder_Reset();                                                         // 陀螺仪重新采样
    delay_ms(3000);                                                              // 等待3s，直到陀螺仪采样完毕
    GyroEncoder_Off();                                                           // 暂停接收陀螺仪与码盘的数据
    PosX = PathParam_SZ_TZ1.P0[0];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    PosY = PathParam_SZ_TZ1.P0[1];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    PosZ = PathParam_SZ_TZ1.P0[2] + 180;                                         // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    GyroEncoder_SetPos();                                                        // 更新码盘的坐标信息
    GyroEncoder_SetAng();                                                        // 更新陀螺仪的坐标信息
    GyroEncoder_On();                                                            // 继续接收陀螺仪与码盘的数据
    Zone = SZ;                                                                   // 将区域变量设置为SZ
    xTaskNotifyGive(BeepTask_Handler);                                           // 外设准备完毕，蜂鸣器再叫一下
    ulTaskNotifyTake(pdTRUE, 0);                                                 // 清除残余的任务通知
    
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != LAUNCH);                   // 等待[发车]指令
    
    Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(750);                     // 蜂鸣器叫一下
    
    xTaskNotify(MoveTask_Handler, SZ_TZ1, eSetValueWithOverwrite);               // 底盘开始跑SZ->TZ1的路径
    xSemaphoreTake(ArrivedSemaphore, 0);                                         // 清除残余的底盘到达信号量
    xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                             // 等待底盘到达信号量被释放
    
    xTaskNotifyGive(BeepTask_Handler);
    
    vTaskDelete(NULL);
}
