#include "Tasks.h"

SemaphoreHandle_t ArrivedSemaphore;
SemaphoreHandle_t FiredSemaphore;

SemaphoreHandle_t JustThrowSemaphore;

TaskHandle_t FlashTask_Handler;
TaskHandle_t BeepTask_Handler;
TaskHandle_t ReportTask_Handler;
TaskHandle_t MoveTask_Handler;
TaskHandle_t FireTask_Handler;
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
        #if ENABLE_SLING_DEBUG
        Sling_QueryPos();
        delay_ms(5);
        printf("S=%-10dR=%-10dA=%-10d\r\n", SliderPos, RotorPos, (int)ArmPos);
        #endif
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
        uint32_t StartTS = millis();
        bool Arrived = false;
        Move_PID_Start();
        switch (Path)
        {
            case LOCKPOINT: GoalX = PosX; GoalY = PosY; GoalZ = PosZ; goto LockPoint;
            case SZ_TZ1: PathParam = &PathParam_SZ_TZ1; break;
            case TZ1_TZ2: PathParam = &PathParam_TZ1_TZ2; break;
            case TZ2_TZ3: PathParam = &PathParam_TZ2_TZ3; break;
//            case TZ3_TZ2: PathParam = &PathParam_TZ3_TZ2; break;
            case SZ_TZ2: PathParam = &PathParam_SZ_TZ2; break;
            default: continue;
        }
        Move_PID_SetTunings(PathParam);
        Move_PID_SetLimits(PathParam);
        while (millis() - StartTS <= PathParam->Duration)
        {
            Move_SetGoal(PathParam, millis() - StartTS);
            Move_PID_Compute();
            Move_PID_Apply();
            //Move_AddConstantFiducial(1);
            //Move_AddDerivativeFiducial(PathParam, millis() - StartTS, 0.25, 0.2, 0.8);
            //Printf(ESP8266, "T = %u, Err = (%.3f, %.3f, %.3f), Vel = (%.3f, %.3f, %.3f)\r\n", millis() - StartTS, PosX - GoalX, PosY - GoalY, PosZ - GoalZ, VelX, VelY, VelZ);
            Omni_Elmo_PVM();
            delay_ms(5);
        }
        LockPoint:
        Printf(ESP8266, "Enter Lock Point Mode\r\n");
        Move_PID_SetTunings(&PathParam_LockPoint);
        Move_PID_SetLimits(&PathParam_LockPoint);
        do
        {
            Move_PID_Compute();
            Move_PID_Apply();
            Move_DeadzoneCtrl(0.01, 0.1);
            //if (millis() - StartTS <= 5000)
            //Printf(ESP8266, "T = %u, Err = (%.3f, %.3f, %.3f), Vel = (%.3f, %.3f, %.3f)\r\n", millis() - StartTS, PosX - GoalX, PosY - GoalY, PosZ - GoalZ, VelX, VelY, VelZ);
            Omni_Elmo_PVM();
            delay_ms(5);
            if (!Arrived && DeltaPos(GoalX - PosX, GoalY - PosY) <= 0.025 && fabs(GoalZ - PosZ) <= 0.5) //Zone == (Path <= TZ2_TZ3 ? Path : TZ2)
            {
                Move_UpdateZone();
                Printf(ESP8266, "Arrived\r\n");
                xSemaphoreGive(ArrivedSemaphore);
                Cradle_ArriveNotify(Zone);
                Probe_SetTimer(Zone - 1, millis() - StartTS);
                xTaskNotifyGive(BeepTask_Handler);
                /*
                //Update 2018/04/16：色标传感器往前安装，正常情况下路径中可进行两次重定位，无需进行该操作
                if (Zone == TZ1 || Zone == TZ2) //强行创造误差，使车再次重定位
                {
                    GyroEncoder_Off();
                    PosY -= 0.10f;
                    GyroEncoder_SetPos();
                    GyroEncoder_On();
                }
                */
                Arrived = true;
            }
            Path = ulTaskNotifyTake(pdTRUE, 0);
        }
        while (!Path);
        Move_PID_Stop();
    }
}

void FireTask(void *pvParameters)
{
    JustThrowSemaphore = xSemaphoreCreateBinary();
    while (1)
    {
        uint8_t FireZone = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (xSemaphoreTake(JustThrowSemaphore, 0))
        {
            switch (FireZone)
            {
                case TZ1: Sling_Fire(&ArmParam_TZ1, true); break;
                case TZ2: Sling_Fire(&ArmParam_TZ2, true); break;
                case TZ3: Sling_Fire(&ArmParam_TZ3, true); break;
            }
            Sling_ArmAdjust();
            xTaskNotifyGive(BeepTask_Handler);
        }
        else
        {
            switch (FireZone)
            {
                case TZ1:
                xSemaphoreTake(DimmerSemaphore, 0);                                          // 清除残余的光电门信号量
                xSemaphoreTake(DimmerSemaphore, portMAX_DELAY);                              // 等待手动车到达TZ1触发的光电门信号量被释放
                Cylinder_Off(5);                                                             // 关闭稳球装置
                Sling_SliderShift(SLIDER_LOAD_POS, 125000, 0.005, 5000);                     // 滑块移动到给悬臂上弹位置
                Sling_RotorSweep(0.50, 35000, 0.01, 5000);                                   // 转轴旋转到180度
                Sling_SliderShift(SLIDER_TZ2_POS, 125000, 0.005, 0);                         // 滑块移动到TZ2的球的上弹位置
                Sling_RotorSweep(0.55, 17500, 0.01, 0);                                      // 同时，转轴旋转到5度
                
                Sling_Fire(&ArmParam_TZ1, true);                                             // 同时，悬臂抛射
                Sling_ArmAdjust();                                                           // 回收悬臂
                
                Sling_SliderShift(SLIDER_TZ2_POS, 125000, 0.005, 5000);                      // 等待滑块移动到TZ2的球的上弹位置
                Sling_RotorSweep(0, 17500, 0.01, 5000);                                      // 等待转轴旋转到5度
                            
                Sling_SliderShift(SLIDER_LOAD_POS, 125000, 0.005, 5000);                     // 滑块移动到给悬臂上弹位置
                Sling_RotorSweep(0.45, 35000, 0.01, 5000);                                   // 转轴旋转到180度
                
                xSemaphoreGive(FiredSemaphore);                                              // 同时，释放抛射机构处理完毕信号量
                
                Sling_SliderShift(SLIDER_TZ3_POS, 125000, 0.005, 5000);                      // 滑块移动到TZ3的球的上弹位置
                Sling_RotorSweep(0.50, 35000, 0.01, 5000);                                   // 同时，转轴旋转到0度
                Cylinder_On(5);
                //xSemaphoreGive(FiredSemaphore);                                              // 同时，释放抛射机构处理完毕信号量
                break;
                
                case TZ2:
                Cylinder_Off(5);
                delay_ms(100);
                Sling_Fire(&ArmParam_TZ2, true);
                Sling_ArmAdjust();
                xSemaphoreTake(DimmerSemaphore, 0);                                          // 清除残余的光电门信号量
                xSemaphoreTake(DimmerSemaphore, portMAX_DELAY);                              // 等待手动车到达TZ2触发的光电门信号量被释放
                Sling_SliderShift(SLIDER_LOAD_POS, 125000, 0.005, 5000);
                Sling_RotorSweep(0.70, 35000, 0.01, 5000);
                Cylinder_On(5);
                Sling_SliderShift(SLIDER_TZ3_POS, 125000, 0.005, 0);
                Sling_RotorSweep(0.30, 35000, 0.01, 0);
                xSemaphoreGive(FiredSemaphore);
                break;
                
                case TZ3:
                Cylinder_Off(5);
                delay_ms(250);
                Sling_Fire(&ArmParam_TZ3, true);
                xSemaphoreGive(FiredSemaphore);
                Sling_ArmAdjust();
                break;
            }
        }
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
    Sling_Init();                                                                // 抛射机构电机抱死
    GyroEncoder_Reset();                                                         // 陀螺仪重新采样
    delay_ms(3000);                                                              // 等待3s，直到陀螺仪采样完毕
    GyroEncoder_Off();                                                           // 暂停接收陀螺仪与码盘的数据
    PosX = PathParam_SZ_TZ1.P0[0];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    PosY = PathParam_SZ_TZ1.P0[1];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    PosZ = PathParam_SZ_TZ1.P0[2];                                               // 将启动区的位置设置为当前位置，并更新到陀螺仪与码盘
    GyroEncoder_SetPos();                                                        // 更新码盘的坐标信息
    GyroEncoder_SetAng();                                                        // 更新陀螺仪的坐标信息
    GyroEncoder_On();                                                            // 继续接收陀螺仪与码盘的数据
    Cylinder_On(5);                                                              // 打开稳球装置
    Sling_ArmAdjust();                                                           // 使用PID调节抛射臂的角度
    Sling_RotorSweep(0.82, 30000, 0.01, 0);                                      // 将旋转臂转到初始位置
    Sling_SliderShift(SLIDER_TZ1_POS, 100000, 0.005, 5000);                      // 将滑轨移动到初始位置
    Zone = SZ;                                                                   // 将区域变量设置为SZ
    xTaskNotifyGive(BeepTask_Handler);                                           // 外设准备完毕，蜂鸣器再叫一下
    ulTaskNotifyTake(pdTRUE, 0);                                                 // 清除残余的任务通知
    
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != LAUNCH);                   // 等待[发车]指令
    xTaskNotifyGive(BeepTask_Handler);                                           // 蜂鸣器叫一下
    delay_ms(1000);
    xTaskNotifyGive(BeepTask_Handler);                                           // 蜂鸣器叫一下
    
    //测试入口
//    Zone = TZ1;
//    xTaskNotifyGive(FireTask_Handler);                                           // 抛射机构开始执行上弹及抛射任务
//    xSemaphoreTake(FiredSemaphore, 0);                                           // 清除残余的抛射完成信号量
//    xSemaphoreTake(FiredSemaphore, portMAX_DELAY);                               // 等待抛射完成信号量被释放
//    Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(750);                     // 蜂鸣器叫一下
//    vTaskDelete(NULL);
    //测试结束
    
    xTaskNotify(MoveTask_Handler, SZ_TZ1, eSetValueWithOverwrite);               // 底盘开始跑SZ->TZ1的路径
    xSemaphoreTake(ArrivedSemaphore, 0);                                         // 清除残余的底盘到达信号量
    xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                             // 等待底盘到达信号量被释放
    Zone = TZ1;                                                                  // 强制将区域设置到TZ1
    xTaskNotify(FireTask_Handler, TZ1, eSetValueWithOverwrite);                  // 抛射机构开始执行上弹及抛射任务
    xSemaphoreTake(FiredSemaphore, 0);                                           // 清除残余的抛射完成信号量
    xSemaphoreTake(FiredSemaphore, portMAX_DELAY);                               // 等待抛射完成信号量被释放
    
    for (uint8_t i = 0; i < 3; i++)
    {
        Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(250);
    }
    
    xTaskNotify(MoveTask_Handler, TZ1_TZ2, eSetValueWithOverwrite);              // 底盘开始跑TZ1->TZ2的路径
    xSemaphoreTake(ArrivedSemaphore, 0);                                         // 清除残余的底盘到达信号量
    xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                             // 等待底盘到达信号量被释放
    Zone = TZ2;                                                                  // 强制将区域设置到TZ2
    xTaskNotify(FireTask_Handler, TZ2, eSetValueWithOverwrite);                  // 抛射机构开始执行上弹及抛射任务
    xSemaphoreTake(FiredSemaphore, 0);                                           // 清除残余的抛射完成信号量
    xSemaphoreTake(FiredSemaphore, portMAX_DELAY);                               // 等待抛射完成信号量被释放
    
    //ulTaskNotifyTake(pdTRUE, 0);
    //while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != LAUNCH);
    xTaskNotifyGive(BeepTask_Handler);
    delay_ms(250);
    
    xTaskNotify(MoveTask_Handler, TZ2_TZ3, eSetValueWithOverwrite);              // 底盘开始跑TZ2->TZ3的路径
    xSemaphoreTake(ArrivedSemaphore, 0);                                         // 清除残余的底盘到达信号量
    xSemaphoreTake(ArrivedSemaphore, portMAX_DELAY);                             // 等待底盘到达信号量被释放
    Zone = TZ3;                                                                  // 强制将区域设置到TZ3
    delay_ms(500);
    xTaskNotify(FireTask_Handler, TZ3, eSetValueWithOverwrite);                  // 抛射机构开始执行上弹及抛射任务
    xSemaphoreTake(FiredSemaphore, 0);                                           // 清除残余的抛射完成信号量
    xSemaphoreTake(FiredSemaphore, portMAX_DELAY);                               // 等待抛射完成信号量被释放
    
    Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(750);
    Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(750);
    Buzzer_On(); delay_ms(250); Buzzer_Off(); delay_ms(750);
    
    while (1)
    {
        xSemaphoreTake(DimmerSemaphore, 0);                                          // 清除残余的光电门信号量
        xSemaphoreTake(DimmerSemaphore, portMAX_DELAY);                              // 等待TZ3触发的光电门信号量被释放
        Cylinder_On(5);
        delay_ms(250);
        Cylinder_Off(5);
        delay_ms(100);
        xTaskNotify(FireTask_Handler, TZ3, eSetValueWithOverwrite);                  // 抛射机构开始执行上弹及抛射任务
        xSemaphoreTake(FiredSemaphore, 0);                                           // 清除残余的抛射完成信号量
        xSemaphoreTake(FiredSemaphore, portMAX_DELAY);                               // 等待抛射完成信号量被释放
    }
    
    //vTaskDelete(NULL);
}
