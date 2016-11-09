/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "Buffer.h"
#include "define.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char Rx_data1;
char Rx_Buffer1[BUFFER_SIZE];
struct ring_buffer rx_rb1;
int Rx_start_flag1;
int Rx_Cplt1;

char Tx_data1;
char Tx_Buffer1[BUFFER_SIZE];
int Tx_index1, Tx_Cplt1;

char Rx_data2;
char Rx_Buffer2[BUFFER_SIZE];
struct ring_buffer rx_rb2;
int Rx_start_flag2;

char Tx_data2;
char Tx_Buffer2[BUFFER_SIZE];
int Tx_index2, Tx_Cplt2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void PC_data_check(void);
void EPS_data_check(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart1, (uint8_t *) &Rx_data1, 1); // Rx1 interrupt enable
    HAL_UART_Receive_IT(&huart2, (uint8_t *) &Rx_data2, 1); // Rx2 interrupt enable
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USART2_DE_Pin */
  GPIO_InitStruct.Pin = USART2_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USART2_DE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // ETX를 만나면 이후 Receive 중단.(ETX 까지는 저장함.)
{
    if(huart->Instance == USART1)
    {        
        Ring_buffer_put(&rx_rb1, Rx_data1);
        
        if(Rx_data1 == STX && Rx_start_flag1 == 0)
        {
            Ring_buffer_flush(&rx_rb1);
            Ring_buffer_put(&rx_rb1, STX);
            Rx_start_flag1 = 1;
        }          
        else if(Rx_Cplt1 == 1)
        {
            PC_data_check();
            Rx_Cplt1 = 0;            
            Rx_start_flag1 = 0;
        }
        else if(Rx_data1 == ETX)
        {
            Rx_Cplt1 = 1;
        }
        /*
        else if(Rx_data1 == ETX && Rx_start_flag1 == 1)
        {
            PC_data_check();
            Rx_start_flag1 = 0;         
        }  */          
        
        HAL_UART_Receive_IT(&huart1, (uint8_t *) &Rx_data1, 1);   
    }    
    else if(huart->Instance == USART2)
    {        
        Ring_buffer_put(&rx_rb2, Rx_data2);
     //   Tx_Buffer1[Tx_index1++] = Rx_data2;
        
        if(Rx_data2 == STX && Rx_start_flag2 == 0)
        {
            Ring_buffer_flush(&rx_rb2);
            Ring_buffer_put(&rx_rb2, STX);
            Rx_start_flag2 = 1;
        }          
        else if(Rx_data2 == ETX && Rx_start_flag2 == 1)
        {
			EPS_data_check();
            Rx_start_flag2 = 0;
//            Tx_index1 = 0;
//            HAL_UART_Transmit_IT(&huart1, (uint8_t *)Tx_Buffer1, 1); // Echo // EPS에서 온 data를 PC로 전송.
        }            
        
        HAL_UART_Receive_IT(&huart2, (uint8_t *) &Rx_data2, 1);   
    }    
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{       
    int i = 0;
    
    if(huart->Instance == USART1)
    {
//		if(!Tx_Cplt1)
//		{
			if(Tx_Buffer1[Tx_index1] != ETX)
			{
				HAL_UART_Transmit_IT(&huart1, (uint8_t *) &(Tx_Buffer1[++Tx_index1]), 1);
			}
			else
			{         
			//	HAL_UART_Transmit_IT(&huart1, (uint8_t *) &(Tx_Buffer1[++Tx_index1]), 1); // If last packet is checksum,
				for(i = 0; i < BUFFER_SIZE; i++)
				{
					Tx_Buffer1[i] = 0;
				}
				Tx_index1 = 0;	
				Tx_Cplt1 = 1;
			}
	/*	}
		else
		{
			for(i = 0; i < BUFFER_SIZE; i++)
			{
				Tx_Buffer1[i] = 0;
			}
			Tx_index1 = 0;
			Tx_Cplt1 = 0;
		}*/
	}
    else if(huart->Instance == USART2)
    {
        if(!Tx_Cplt2)
        {
            if(Tx_Buffer2[Tx_index2] != ETX)
            {
                HAL_UART_Transmit_IT(&huart2, (uint8_t *) &(Tx_Buffer2[++Tx_index2]), 1);
            }
            else
            {            
                HAL_UART_Transmit_IT(&huart2, (uint8_t *) &(Tx_Buffer2[++Tx_index2]), 1); // If last packet is checksum,
                Tx_Cplt2 = 1;
            }
        }
        else
		{
			for(i = 0; i < BUFFER_SIZE; i++)
			{
				Tx_Buffer2[i] = 0;
			}
			Tx_index2 = 0;
            Tx_Cplt2 = 0;
		}
    }     
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{  
	if(huart->Instance == USART1)
		HAL_UART_Receive_IT(&huart1, (uint8_t *) &Rx_data1, 1);
 	if(huart->Instance == USART2)
		HAL_UART_Receive_IT(&huart2, (uint8_t *) &Rx_data2, 1);         
}

void PC_data_check(void)
{
    // 예시 // PC에서 넘어온 명령을 그대로 넘김.
    uint8_t stx, id, command, data, etx, r_checksum;
    uint8_t checksum;
    uint8_t t_checksum;
    
    Ring_buffer_get(&rx_rb1, &stx);
    Ring_buffer_get(&rx_rb1, &id);
    Ring_buffer_get(&rx_rb1, &command);
	
	if(command == CMD_CTRL)
	{
		Ring_buffer_get(&rx_rb1, &data);
		Ring_buffer_get(&rx_rb1, &etx);
		Ring_buffer_get(&rx_rb1, &r_checksum);
		
		checksum = (stx + id + command + data + etx) % 256;	

	    if(r_checksum == checksum) // if data is valid,
		{
			Tx_Buffer2[0] = STX;
			Tx_Buffer2[1] = id;

			Tx_Buffer2[2] = CMD_CTRL;
			Tx_Buffer2[3] = data;
			Tx_Buffer2[4] = ETX;
			t_checksum = (Tx_Buffer2[0] + Tx_Buffer2[1] + Tx_Buffer2[2] + Tx_Buffer2[3] + Tx_Buffer2[4]) % 256; 
			Tx_Buffer2[5] = t_checksum;
		}			
	}
	else if(command == CMD_STATE_REQ)
	{
		Ring_buffer_get(&rx_rb1, &etx);
		Ring_buffer_get(&rx_rb1, &r_checksum);

		checksum = (stx + id + command + etx) % 256;
		
		if(r_checksum == checksum) // if data is valid,
		{
			Tx_Buffer2[0] = STX;
			Tx_Buffer2[1] = id;		
            Tx_Buffer2[2] = CMD_STATE_REQ;
            Tx_Buffer2[3] = ETX;
            t_checksum = (Tx_Buffer2[0] + Tx_Buffer2[1] + Tx_Buffer2[2] + Tx_Buffer2[3]) % 256;
            Tx_Buffer2[4] = t_checksum;
		}			
	}
    
    HAL_UART_Transmit_IT(&huart2, (uint8_t *) Tx_Buffer2, 1);
	/*
	if(command == CMD_CTRL)
	{
		Ring_buffer_get(&rx_rb1, &data);
		Ring_buffer_get(&rx_rb1, &etx);
		Ring_buffer_get(&rx_rb1, &r_checksum);
		
		checksum = (stx + id + command + data + etx) % 256;	

	    if(r_checksum == checksum) // if data is valid,
		{
			Tx_Buffer1[0] = STX;
			Tx_Buffer1[1] = id;

			Tx_Buffer1[2] = CMD_CTRL;
			Tx_Buffer1[3] = data;
			Tx_Buffer1[4] = ETX;
			t_checksum = (Tx_Buffer1[0] + Tx_Buffer1[1] + Tx_Buffer1[2] + Tx_Buffer1[3] + Tx_Buffer1[4]) % 256; 
			Tx_Buffer1[5] = t_checksum;
		}			
	}
	else if(command == CMD_STATE_REQ)
	{
		Ring_buffer_get(&rx_rb1, &etx);
		Ring_buffer_get(&rx_rb1, &r_checksum);

		checksum = (stx + id + command + etx) % 256;
		
		if(r_checksum == checksum) // if data is valid,
		{
			Tx_Buffer1[0] = STX;
			Tx_Buffer1[1] = id;		
            Tx_Buffer1[2] = CMD_STATE_REQ;
            Tx_Buffer1[3] = ETX;
            t_checksum = (Tx_Buffer1[0] + Tx_Buffer1[1] + Tx_Buffer1[2] + Tx_Buffer1[3]) % 256;
            Tx_Buffer1[4] = t_checksum;
		}			
	}
    
    HAL_UART_Transmit_IT(&huart1, (uint8_t *) Tx_Buffer1, 1);*/
}

void EPS_data_check(void)
{
    uint8_t stx, id, command, state, etx, r_checksum;
    uint8_t checksum;
    uint8_t t_checksum;	
    
    Ring_buffer_get(&rx_rb2, &stx);
    Ring_buffer_get(&rx_rb2, &id);
    Ring_buffer_get(&rx_rb2, &command);
    Ring_buffer_get(&rx_rb2, &state);
    Ring_buffer_get(&rx_rb2, &r_checksum);
    Ring_buffer_get(&rx_rb2, &etx);
    
    checksum = (id + command + state) % 256;	// 프로토콜이 중구남방임. 일부러 이렇게 하는건지는 잘 모르겠음.
	t_checksum = checksum; // 테스트 할 때는 똑같은 데이터......지만 PC <-> 컨트롤러 프로토콜에 따라 달라질 수 있으니 주의
	
    if(r_checksum == checksum) // if data is valid,
    {
		Tx_Buffer1[0] = STX;
		Tx_Buffer1[1] = id;
		Tx_Buffer1[2] = CMD_STATE_REQ;
		Tx_Buffer1[3] = state;
		Tx_Buffer1[4] = checksum;
		Tx_Buffer1[5] = ETX;
		
		HAL_UART_Transmit_IT(&huart1, (uint8_t *) Tx_Buffer1, 1);
		
		/*
		Tx_Buffer2[0] = STX;
		Tx_Buffer2[1] = id;
		Tx_Buffer2[2] = CMD_STATE_REQ;
		Tx_Buffer2[3] = state;
		Tx_Buffer2[4] = checksum;
		Tx_Buffer2[5] = ETX;
		
		HAL_UART_Transmit_IT(&huart2, (uint8_t *) Tx_Buffer2, 1);*/
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
