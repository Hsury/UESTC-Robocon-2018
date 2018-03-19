#include "Tasks.h"

TaskHandle_t FlashTask_Handler;
TaskHandle_t BeepTask_Handler;
TaskHandle_t ReportTask_Handler;
TaskHandle_t MoveTask_Handler;
TaskHandle_t LoadTask_Handler;

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
    while (1)
    {
        uint8_t Path = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (Path < 1 || Path > 5) continue;
        for (uint8_t i = 0; i < Path; i++)
        {
            Buzzer_On();
            delay_ms(50);
            Buzzer_Off();
            delay_ms(50);
        }
        Elmo_Reinit(1);
        Elmo_Reinit(2);
        Elmo_Reinit(3);
        GyroEncoder_Reset();
        delay_ms(250);
        for (uint8_t i = 0; i < 3; i++)
        {
            Buzzer_On();
            delay_ms(250);
            Buzzer_Off();
            delay_ms(750);
        }
        delay_ms(100);
        GyroEncoder_Off();
        switch (Path)
        {
            case 1:
            PosX = 0.55;
            PosY = 7.54;
            PosZ = 0.00;
            break;
            
            case 2:
            PosX = 2.80;
            PosY = 3.37;
            PosZ = 0.00;
            break;
            
            case 3:
            PosX = 2.80;
            PosY = 1.27;
            PosZ = 0.00;
            break;
            
            case 4:
            PosX = 6.80;
            PosY = 1.27;
            PosZ = 11.00;
            break;
            
            case 5:
            PosX = 0.55;
            PosY = 7.54;
            PosZ = 0.00;
            break;
        }
        GyroEncoder_SetPos();
        GyroEncoder_SetAng();
        GyroEncoder_On();
        delay_ms(100);
        Move_PID_Init();
        if (Path == 3 || Path == 4) Move_PID_SetTunings(2.25, 0, 0,
                                                        2.5, 0, 0,
                                                        5.0, 0, 0);
        Move_PID_Start();
        uint32_t ExpDuration;
        bool Arrived = false;
        bool Relocated = false;
        uint32_t StartTS = millis();
        while (millis() - StartTS < 7500)
        {
            switch (Path)
            {
                case 1:
                ExpDuration = 2250;
                GoalX = CubicBezier(0.55, 1.25, 1.25, 2.80, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalY = CubicBezier(7.54, 4.00, 3.37, 3.37, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalZ = 0.00;
                break;
                
                case 2:
                ExpDuration = 2250;
                GoalX = CubicBezier(2.80, 1.15, 1.15, 2.80, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalY = CubicBezier(3.37, 2.87, 1.57, 1.27, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalZ = 0.00;
                break;
                
                case 3:
                ExpDuration = 2250;
                GoalX = CubicBezier(2.80, 4.80, 6.79, 6.80, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalY = CubicBezier(1.27, 1.27, 1.27, 1.27, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalZ = CubicBezier(0.00, 3.50, 7.50, 11.00, 1.0f * (millis() - StartTS) / ExpDuration);
                break;
                
                case 4:
                ExpDuration = 2250;
                GoalX = CubicBezier(6.80, 4.80, 2.81, 2.80, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalY = CubicBezier(1.27, 1.27, 1.27, 1.27, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalZ = CubicBezier(11.00, 7.50, 3.50, 0.00, 1.0f * (millis() - StartTS) / ExpDuration);
                break;
                
                case 5:
                ExpDuration = 3000;
                GoalX = CubicBezier(0.55, 1.25, 1.25, 2.80, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalY = CubicBezier(7.54, 2.00, 1.27, 1.27, 1.0f * (millis() - StartTS) / ExpDuration);
                GoalZ = 0.00;
                break;
            }
            Move_PID_Compute();
            Move_PID_Apply();
            Omni_Elmo_PVM();
            delay_ms(2);
            if (millis() - StartTS > ExpDuration && !Arrived && DeltaPos(GoalX - PosX, GoalY - PosY) <= 0.1)
            {
                Arrived = true;
            }
            if (Arrived && !Relocated && GY53_Relocate())
            {
                Relocated = true;
            }
            if (Arrived && Relocated && fabs(VelX) <= 0.01 && fabs(VelY) <= 0.01 && fabs(VelZ) <= 0.1)
            {
                xTaskNotifyGive(BeepTask_Handler);
                break;
            }
        }
        Probe_SetTimer(Path - 1, millis() - StartTS);
        Probe_SetArrive(Path <= 3 ? Path : 2);
        Move_PID_Stop();
        Omni_Elmo_Stop();
        delay_ms(250);
        Omni_Elmo_Close();
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

void LoadTask(void *pvParameters)
{
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
    {
        Loader_Init();
        Buzzer_On();
        delay_ms(100);
        Buzzer_Off();
        delay_ms(100);
    }
    while (1)
    {
        uint8_t Event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        switch (Event)
        {
            case 1:
            Loader_Shift(2);
            break;
            
            case 2:
            Loader_Sweep();
            break;
        }
        xTaskNotifyGive(BeepTask_Handler);
        delay_ms(500);
        ulTaskNotifyTake(pdTRUE, 0);
//        Loader_QueryPos();
//        delay_ms(10);
//        printf("Slider:%d, Tong:%d\r\n", SliderPos, TongPos);
    }
}
