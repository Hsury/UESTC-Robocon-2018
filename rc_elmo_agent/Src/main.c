/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "elmo.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_FilterConfTypeDef  CAN1_FilterConf;
CAN_FilterConfTypeDef  CAN2_FilterConf;
int status;
int addr;
int32_t flag;
int32_t value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	hcan1.pRxMsg=(CanRxMsgTypeDef* )malloc(sizeof(CanRxMsgTypeDef));
  hcan1.pTxMsg=(CanTxMsgTypeDef* )malloc(sizeof(CanTxMsgTypeDef));
  hcan2.pRxMsg=(CanRxMsgTypeDef* )malloc(sizeof(CanRxMsgTypeDef));
  hcan2.pTxMsg=(CanTxMsgTypeDef* )malloc(sizeof(CanTxMsgTypeDef));
  //static CanTxMsgTypeDef        TxMessage;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();

  /* USER CODE BEGIN 2 */
	
	/* Configure CAN Filter */
	CAN1_FilterConf.FilterNumber = 0;
  CAN1_FilterConf.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN1_FilterConf.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN1_FilterConf.FilterIdHigh = 0x0000;
  CAN1_FilterConf.FilterIdLow = 0x0000;
  CAN1_FilterConf.FilterMaskIdHigh = 0x0000;
  CAN1_FilterConf.FilterMaskIdLow = 0x0000;
  CAN1_FilterConf.FilterFIFOAssignment = 0;
  CAN1_FilterConf.FilterActivation = ENABLE;
  CAN1_FilterConf.BankNumber = 14;
  HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterConf);
	
	CAN2_FilterConf.FilterNumber = 14;
  CAN2_FilterConf.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN2_FilterConf.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN2_FilterConf.FilterIdHigh = 0x0000;
  CAN2_FilterConf.FilterIdLow = 0x0000;
  CAN2_FilterConf.FilterMaskIdHigh = 0x0000;
  CAN2_FilterConf.FilterMaskIdLow = 0x0000;
  CAN2_FilterConf.FilterFIFOAssignment = 0;
  CAN2_FilterConf.FilterActivation = ENABLE;
  //CAN2_FilterConf.BankNumber = 14;  //双控制器分界线，无需重复设置
  HAL_CAN_ConfigFilter(&hcan2, &CAN2_FilterConf);
	
	/* 注册消息 */
//  hcan2.pRxMsg = &RxMessage;
	
	/* 报文示例 */
//	hcan2.pTxMsg->StdId = 0x123;
//  hcan2.pTxMsg->RTR = CAN_RTR_DATA;
//  hcan2.pTxMsg->IDE = CAN_ID_STD;
//  hcan2.pTxMsg->DLC = 8;
//  hcan2.pTxMsg->Data[0] = 'C';
//  hcan2.pTxMsg->Data[1] = 'A';
//  hcan2.pTxMsg->Data[2] = 'N';
//  hcan2.pTxMsg->Data[3] = ' ';
//  hcan2.pTxMsg->Data[4] = 'T';
//  hcan2.pTxMsg->Data[5] = 'e';
//  hcan2.pTxMsg->Data[6] = 's';
//  hcan2.pTxMsg->Data[7] = 't';

	/* 注册中断 */
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	
	BLUE_ON;
	Elmo_Init(); 
	BLUE_OFF;
	
	//Elmo_SetAcc(1, 100000);
	//Elmo_PVM(1,50000);
	
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
	switch (flag) {
		case 1: //设置速度
			Elmo_PVM(addr, value);
			flag = 0;
			break;
		case 2: //设置加速度
			Elmo_SetAcc(addr, value);
			flag = 0;
			break;
		case 3: //设置减速度
			Elmo_SetDec(addr, value);
			flag = 0;
			break;
		case 4: //抱死
			Elmo_Break(addr);
			flag = 0;
			break;
		case 5: //释放
			Elmo_Release(addr);
			flag = 0;
			break;
	}
	
	if (status == 0 && HAL_CAN_Transmit(&hcan2, 10) != HAL_OK)
	{
		HAL_TIM_Base_Stop_IT(&htim3);
		GREEN_OFF;
		BLUE_OFF;
		RED_ON;
		status = 1;
	}
	else if (status == 1 && HAL_CAN_Transmit(&hcan2, 10) == HAL_OK)
	{
		BLUE_ON;
		Elmo_Init();
		BLUE_OFF;
		HAL_TIM_Base_Start_IT(&htim3);
		RED_OFF;
		status = 0;
	}
		/*
		if(HAL_CAN_Receive(&hcan1, CAN_FIFO0, 10) == HAL_OK){
			printf("get");
      printf("StdId : %x\r\n",hcan1.pRxMsg->StdId);
      printf("RxMsg : %s",hcan1.pRxMsg->Data);
      printf("\r\n\r\n");
		}
		
		if(HAL_CAN_Receive(&hcan2, CAN_FIFO0, 10) == HAL_OK && hcan2.pRxMsg->StdId!=0x701){
			printf("get_ELMO");
      printf("StdId : %x\r\n",hcan2.pRxMsg->StdId);
      printf("RxMsg : %s",hcan2.pRxMsg->Data);
      printf("\r\n\r\n");
		}
		*/
			/*##-3- Start the CAN2 Transmission process #####################################*/
		
//      if(HAL_CAN_Transmit(&hcan2, 10) != HAL_OK)
//      {
//          /* Transmition Error */
//					printf("1\r\n\r\n");
//          Error_Handler();
//      }
//			printf("continue\r\n\r\n"); 
//      if(HAL_CAN_GetState(&hcan2) != HAL_CAN_STATE_READY)
//      {
//        printf("2\r\n\r\n");  
//				Error_Handler();
//            
//      }
//      /*##-4- Start the CAN1 Reception process ########################################*/
//      if(HAL_CAN_Receive(&hcan1, CAN_FIFO0,10) != HAL_OK)
//      {
//          /* Reception Error */
//        printf("3\r\n\r\n");  
//				Error_Handler();    
//      }
//      if(HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_READY)
//      {
//        printf("4\r\n\r\n");  
//				Error_Handler();
//      }
//      printf("StdId : %x\r\n",hcan1.pRxMsg->StdId);
//      printf("RxMsg : %s",hcan1.pRxMsg->Data);
//      printf("\r\n\r\n");
//			HAL_Delay(1000);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/* 定时器中断回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim3.Instance)
		{
			/* Toggle LED */
			GREEN_TOGGLE;
			printf("-1s\r\n");
		}
}

/* CAN 中断回调函数 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if (hcan == &hcan1 && hcan1.pRxMsg->StdId < 10 && hcan1.pRxMsg->DLC == 8) { //自定义报文有效
		addr = hcan1.pRxMsg->StdId;
		flag = (hcan1.pRxMsg->Data[3] << 24) + (hcan1.pRxMsg->Data[2] << 16) + (hcan1.pRxMsg->Data[1] << 8) + hcan1.pRxMsg->Data[0];
		value = (hcan1.pRxMsg->Data[7] << 24) + (hcan1.pRxMsg->Data[6] << 16) + (hcan1.pRxMsg->Data[5] << 8) + hcan1.pRxMsg->Data[4];
		printf("StdId : %x\r\nRxMsg : %s\r\nFlag : %i\r\nValue : %i\r\n", hcan1.pRxMsg->StdId, hcan1.pRxMsg->Data, flag, value);
	}
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0); //重新开启中断
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
