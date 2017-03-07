#include "main.h"
#include "stm32f1xx_hal.h"

#include "sserial.h"

SPI_HandleTypeDef hspi2;

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(YELLOW_GPIO_Port,YELLOW_Pin,GPIO_PIN_SET);

  sserial_init();

  /* Infinite loop */
  while (1)
  {
     sserial_do();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
     // HAL_GPIO_WritePin(REL1_GPIO_Port,REL1_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL2_GPIO_Port,REL2_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL3_GPIO_Port,REL3_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL4_GPIO_Port,REL4_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL5_GPIO_Port,REL5_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL6_GPIO_Port,REL6_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL7_GPIO_Port,REL7_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL8_GPIO_Port,REL8_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL9_GPIO_Port,REL9_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL10_GPIO_Port,REL10_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL11_GPIO_Port,REL11_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(REL12_GPIO_Port,REL12_Pin,GPIO_PIN_SET);
     //
     // HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(YELLOW_GPIO_Port,YELLOW_Pin,GPIO_PIN_RESET);
     //
     // HAL_Delay(100);
     // HAL_GPIO_WritePin(REL1_GPIO_Port,REL1_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL2_GPIO_Port,REL2_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL3_GPIO_Port,REL3_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL4_GPIO_Port,REL4_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL5_GPIO_Port,REL5_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL6_GPIO_Port,REL6_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL7_GPIO_Port,REL7_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL8_GPIO_Port,REL8_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL9_GPIO_Port,REL9_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL10_GPIO_Port,REL10_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL11_GPIO_Port,REL11_Pin,GPIO_PIN_RESET);
     // HAL_GPIO_WritePin(REL12_GPIO_Port,REL12_Pin,GPIO_PIN_RESET);
     //
     // HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET);
     // HAL_GPIO_WritePin(YELLOW_GPIO_Port,YELLOW_Pin,GPIO_PIN_SET);
     // HAL_Delay(100);

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
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;//4 => 9 mbit
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : IN11_Pin IN12_Pin IN13_Pin IN14_Pin 
                           IN15_Pin IN16_Pin IN17_Pin IN1_Pin 
                           IN2_Pin */
  GPIO_InitStruct.Pin = IN11_Pin|IN12_Pin|IN13_Pin|IN14_Pin 
                          |IN15_Pin|IN16_Pin|IN17_Pin|IN1_Pin 
                          |IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN18_Pin IN19_Pin IN20_Pin */
  GPIO_InitStruct.Pin = IN18_Pin|IN19_Pin|IN20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : REL12_Pin REL11_Pin REL10_Pin REL9_Pin 
                           REL8_Pin */
  GPIO_InitStruct.Pin = REL12_Pin|REL11_Pin|REL10_Pin|REL9_Pin 
                          |REL8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : REL7_Pin REL6_Pin */
  GPIO_InitStruct.Pin = REL7_Pin|REL6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : REL5_Pin REL4_Pin REL3_Pin REL2_Pin 
                           REL1_Pin RED_Pin YELLOW_Pin */
  GPIO_InitStruct.Pin = REL5_Pin|REL4_Pin|REL3_Pin|REL2_Pin 
                          |REL1_Pin|RED_Pin|YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE_Pin IN4_Pin IN5_Pin IN6_Pin 
                           IN7_Pin IN8_Pin IN9_Pin IN10_Pin */
  GPIO_InitStruct.Pin = MODE_Pin|IN4_Pin|IN5_Pin|IN6_Pin 
                          |IN7_Pin|IN8_Pin|IN9_Pin|IN10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IN3_Pin */
  GPIO_InitStruct.Pin = IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, REL12_Pin|REL11_Pin|REL10_Pin|REL9_Pin 
                          |REL8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, REL7_Pin|REL6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, REL5_Pin|REL4_Pin|REL3_Pin|REL2_Pin 
                          |REL1_Pin|RED_Pin|YELLOW_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

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
