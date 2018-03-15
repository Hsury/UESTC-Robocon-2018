#include "Tasks.h"

TaskHandle_t FlashTask_Handler;
TaskHandle_t BeepTask_Handler;
TaskHandle_t ReportTask_Handler;
TaskHandle_t MoveTask_Handler;

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
        BuzzerOn();
        delay_ms(250);
        BuzzerOff();
        delay_ms(250);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void ReportTask(void *pvParameters)
{
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
    while (1)
    {
        uint8_t Path = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (Path < 1 || Path > 3) continue;
        for (uint8_t i = 0; i < Path; i++)
        {
            BuzzerOn();
            delay_ms(50);
            BuzzerOff();
            delay_ms(50);
        }
        Elmo_Init(CAN1, 9, 0);
        GyroEncoder_Reset();
        delay_ms(250);
        for (uint8_t i = 0; i < 3; i++)
        {
            BuzzerOn();
            delay_ms(250);
            BuzzerOff();
            delay_ms(750);
        }
        delay_ms(100);
        GyroEncoder_Off();
        switch (Path)
        {
            case 1:
            PosX = 0.55;
            PosY = 7.54;
            break;
            
            case 2:
            PosX = 2.80;
            PosY = 3.37;
            break;
            
            case 3:
            PosX = 2.80;
            PosY = 1.27;
            break;
        }
        GyroEncoder_SetPos();
        GyroEncoder_On();
        delay_ms(100);
        Move_PID_Init();
        if (Path == 3) Move_PID_SetTunings(1.5, 0, 0,
                                           2.5, 0, 0,
                                           5.0, 0, 0);
        Move_PID_Start();
        uint32_t ExpDuration;
        bool Arrived = false;
        uint32_t StartTS = millis();
        while (millis() - StartTS < 5000)
        {
            switch (Path)
            {
                case 1:
                ExpDuration = 2250;
                GoalX = CubicBezier(0.55, 1.25, 1.25, 2.80, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalY = CubicBezier(7.54, 4.00, 3.37, 3.37, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalZ = 0;
                break;
                
                case 2:
                ExpDuration = 2500;
                GoalX = CubicBezier(2.80, 1.15, 1.15, 2.80, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalY = CubicBezier(3.37, 2.87, 1.57, 1.27, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalZ = 0;
                break;
                
                case 3:
                ExpDuration = 2000;
                GoalX = 2.8 + clamp(4.0 * (millis() - StartTS) / (ExpDuration - 1250), 0, 4.0);
                GoalY = 1.27;
                GoalZ = clamp(11.0 * (millis() - StartTS) / (ExpDuration - 250), 0, 11.0);
                break;
            }
            Move_PID_Compute();
            Move_PID_Apply();
            Omni_Elmo_PVM();
            delay_ms(2);
            if (millis() - StartTS > ExpDuration && !Arrived && DeltaPos(GoalX - PosX, GoalY - PosY) <= 0.05)
            {
                Probe_SetTimer(Path - 1, millis() - StartTS);
                Probe_SetArrive(Path);
                xTaskNotifyGive(BeepTask_Handler);
                Arrived = true;
            }
        }
        Move_PID_Stop();
        Omni_Elmo_Stop();
        delay_ms(250);
        Elmo_Close(0);
        xTaskNotifyGive(BeepTask_Handler);
        ulTaskNotifyTake(pdTRUE, 0);
        
/*
        uint32_t Duration;
        Duration = PIDMove( 1.05, 2.25,  1.5, 0, 0, 
                           -3.15, 2.75, 2.35, 0, 0, 
                               0,   90,  7.5, 0, 0, 
                           WHEN_Y_DIST_LESS_THAN, 0.05, 3000);
        Probe_SetTimer(0, Duration);
        xTaskNotifyGive(BeepTask_Handler);
        Duration += PIDMove( 2.15, 2.45, 2.45, 0.2, 0, 
                            -3.15, 2.45, 2.40, 0.8, 0, 
                                0,   90,  7.5, 0, 0, 
                            5, 0.05, 8000);
        Probe_SetTimer(0, Duration);
        xTaskNotifyGive(BeepTask_Handler);
//        PIDMove( 2.15, 2.25, 2.25, 0, 0, 
//                -3.15, 2.45, 2.40, 0, 0, 
//                    0,   90,  7.5, 0, 0, 
//                3, 0.05, 5000);
//        xTaskNotifyGive(BeepTask_Handler);
*/

//        Omni_Elmo_Stop();
//        PackUp(0xB1, (uint8_t*)(&Duration));
        
//        Duration = UniformPlusP(0.9, -4.5, 0, 3000);
//        xTaskNotifyGive(BeepTask_Handler);
//        PackUp(0xB1, (uint8_t*)(&Duration));
//        Duration = UniformPlusP(2.3, -4.5, 0, 3000);
//        xTaskNotifyGive(BeepTask_Handler);
//        PackUp(0xB1, (uint8_t*)(&Duration));
    }
}
