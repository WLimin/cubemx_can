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
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "diag/Trace.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//uint8_t ubKeyNumber = 0x0;
volatile uint8_t MsgFlag = 0;
static CanTxMsgTypeDef TxMessage;
static CanRxMsgTypeDef RxMessage;
uint8_t txBuf[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void CAN_Config(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define LED_R(Op) HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_##Op)
#define LED_G(Op) HAL_GPIO_WritePin(LED_G_GPIO_Port,  LED_G_Pin, GPIO_PIN_##Op)
#define LED_B(Op) HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_##Op)

/**
 * @brief  Configures the CAN.
 * @param  None
 * @retval None
 */
static void CAN_Config(void) {
	CAN_FilterConfTypeDef sFilterConfig;
	/*##-1- Configure the CAN peripheral #######################################*/
	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;
	/*
	 CAN bps = RCC_APB1Periph_CAN1 / Prescaler / (SJW + BS1 + BS2);

	 SJW = synchronisation_jump_width
	 BS = bit_segment

	 设置CAN波特率为500Kbsp
	 CAN 波特率 = 360000000 / 6 / (1 + 5 + 6) / = 500Kbsp
	 */

	/*
	 TTCM = time triggered communication mode
	 ABOM = automatic bus-off management
	 AWUM = automatic wake-up mode
	 NART = no automatic retransmission
	 RFLM = receive FIFO locked mode
	 TXFP = transmit FIFO priority
	 */
	hcan.Init.TTCM = DISABLE; 			// 禁止时间触发模式（不生成时间戳), T
	hcan.Init.ABOM = ENABLE; 			// 禁止自动总线关闭管理
	hcan.Init.AWUM = DISABLE; 			// 禁止自动唤醒模式
	hcan.Init.NART = ENABLE; 			// 禁止仲裁丢失或出错后的自动重传功能
	hcan.Init.RFLM = DISABLE; 			// 禁止接收FIFO加锁模式
	hcan.Init.TXFP = DISABLE; 			// 禁止传输FIFO优先级
	hcan.Init.Mode = CAN_MODE_NORMAL; 	// 设置CAN为正常工作模式
//	hcan.Init.Mode = CAN_MODE_LOOPBACK;

	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		/* Initiliazation Error */
		Error_Handler();
	}

	/*##-2- Configure the CAN Filter ###########################################*/
	/* 设置CAN滤波器0 */
	sFilterConfig.FilterNumber = 0;							// 滤波器序号，0-13，共14个滤波器
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;	// 滤波器模式，设置ID掩码模式
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	// 32位滤波
	sFilterConfig.FilterIdHigh = 0x0000;					// 掩码后ID的高16bit
	sFilterConfig.FilterIdLow = 0x0000;					// 掩码后ID的低16bit
	sFilterConfig.FilterMaskIdHigh = 0x0000;				// ID掩码值高16bit
	sFilterConfig.FilterMaskIdLow = 0x0000;				// ID掩码值低16bit
	sFilterConfig.FilterFIFOAssignment = 0;				// 滤波器绑定FIFO 0
	sFilterConfig.FilterActivation = ENABLE;				// 使能滤波器
	sFilterConfig.BankNumber = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	/*##-3- Configure Transmission process #####################################*/
	hcan.pTxMsg->StdId = 0x101;
	hcan.pTxMsg->ExtId = 0x00;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->DLC = 1;
}
/**
 * @brief  Transmission  complete callback in non blocking mode
 * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *HCan) {
	if (HCan->pRxMsg->IDE == CAN_ID_STD) {
		MsgFlag = 1;
	}

	/* Receive */
	if (HAL_CAN_Receive_IT(HCan, CAN_FIFO0) != HAL_OK) {
		/* Reception Error */
		Error_Handler();
	}
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	sprintf(txBuf, "HAL_CAN_ErrorCallback, GetState=%x,GetError=%x\n", HAL_CAN_GetState(hcan), HAL_CAN_GetError(hcan));
	HAL_UART_Transmit_IT(&huart2, txBuf, (uint16_t) strlen(txBuf));
}
/* USER CODE END 0 */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

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
  MX_DMA_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();

  /* USER CODE BEGIN 2 */
	LED_R(SET);
	LED_G(SET);
	LED_B(SET);
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET); //ON
	HAL_Delay(300);
	LED_R(RESET);
	LED_G(RESET);
	LED_B(RESET); //OFF
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET); //OFF

	/*##-1- Configure the CAN peripheral #######################################*/
	CAN_Config();
	/*##-2- Start the Reception process and enable reception interrupt #########*/
	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK) {
		/* Reception Error */
		Error_Handler();
	}

	uint32_t seconds = 0;
	uint8_t ubKeyNumber = 0;
	memset(txBuf, 0, 256l);
	uint32_t Ticks = HAL_GetTick() + 500;
	uint8_t isOn = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		// Count seconds on the trace device.
		if (Ticks <= HAL_GetTick()) {
			Ticks = HAL_GetTick() + 500; //1ms interrupt
			if (isOn) {
				isOn = 0;
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
				sprintf(txBuf, "Second %u\n", seconds);
				HAL_UART_Transmit_IT(&huart2, txBuf, (uint16_t) strlen(txBuf));
				trace_printf(txBuf);
			} else {
				isOn = 1;
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
				seconds++;
				/* Set the data to be tranmitted */
				ubKeyNumber = ++ubKeyNumber % 8;
				hcan.pTxMsg->Data[0] = ubKeyNumber;
				hcan.pTxMsg->Data[1] = 0xAD;

				/*##-3- Start the Transmission process ###############################*/
				if (HAL_CAN_Transmit(&hcan, 10) != HAL_OK) {
					/* Transmition Error */
					Error_Handler();
				}
			}
		}
		if (MsgFlag) {
			MsgFlag = 0;
			sprintf(txBuf, "StdId=0x%08X, DLC=0x%08X, Data[0]=0x%02X\n", RxMessage.StdId, RxMessage.DLC,
					RxMessage.Data[0]);
			HAL_UART_Transmit_IT(&huart2, txBuf, (uint16_t) strlen(txBuf));
			uint8_t ledCtrl = RxMessage.Data[0];
			if (ledCtrl & 0x1) {
				LED_B(SET);
			} else {
				LED_B(RESET);
			}
			if (ledCtrl & 0x2) {
				LED_G(SET);
			} else {
				LED_G(RESET);
			}
			if (ledCtrl & 0x4) {
				LED_R(SET);
			} else {
				LED_R(RESET);
			}
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

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

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	trace_printf("Error_Handler in File: %s, Line: %d\n", file, line);
	HAL_Delay(300);
//  while(1)
//  {
//  }
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
	trace_printf("Wrong parameters value: file %s on line %d\r\n", file, line);
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
