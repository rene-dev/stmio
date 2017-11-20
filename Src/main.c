#include "main.h"
#include "stm32f1xx_hal.h"

#include "sserial.h"
#include "keymatrix.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
uint8_t tx[100];
uint8_t rx[100];
uint8_t inbuf[10];
uint8_t in[4];
volatile uint8_t out[4];

void SystemClock_Config(void);
void Error_Handler(void);
void spi2_init(void);
void spi13_init(void);

//20kHz
void TIM1_UP_IRQHandler(){
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
  //matrix_update();

  // led[0] = ~matrix_button[0];
  // led[1] = ~matrix_button[1];
  // led[2] = ~matrix_button[2];
  // led[3] = ~matrix_button[3];
  // led[4] = ~matrix_button[4];
  // led[5] = ~matrix_button[5];
  
  // led[0] = 0xff;
  // led[1] = 0xff;
  // led[2] = 0xff;
  // led[3] = 0xff;
  // led[4] = 0xff;
  // led[5] = 0xff;
  
  sserial_do();
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  spi13_init();

  //matrix_init();
  sserial_init();

  GPIO_InitTypeDef GPIO_InitStruct;
  /*
  //out
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  //estop
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //CS debug
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  //spi reset
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
*/
  //LED
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
  
  __HAL_RCC_TIM1_CLK_ENABLE();
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3600; // 72e6 / 3600 = 20kHz
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  // TIM_ClockConfigTypeDef sClockSourceConfig;
  // sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  // HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
  //
  // TIM_MasterConfigTypeDef sMasterConfig;
  // sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  // sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  // HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  tx[0]  = 0x40;//opcode 0x40,0x42,0x44
  tx[1]  = 0x00;//addr
  tx[2]  = 0x00;//dira
  tx[3]  = 0x00;//dirb
  tx[4]  = 0x00;//pola
  tx[5]  = 0x00;//polb
  tx[6]  = 0x00;//gpintena
  tx[7]  = 0x00;//gpintenb
  tx[8]  = 0x00;//defvala
  tx[9]  = 0x00;//defvalb
  tx[10] = 0x00;//intcona
  tx[11] = 0x00;//intconb
  tx[12] = 0x08;//iocon
  tx[13] = 0x08;//iocon
  tx[14] = 0x00;//gppua
  tx[15] = 0x00;//gppub
  tx[16] = 0x00;//intfa
  tx[17] = 0x00;//infbb
  tx[18] = 0x00;//intcapa
  tx[19] = 0x00;//intcapb
  tx[20] = 0xff;//gpioa
  tx[21] = 0xff;//gpiob

  rx[0]  = 0x40;//opcode 0x40,0x42,0x44
  rx[1]  = 0x00;//addr
  rx[2]  = 0xff;//dira
  rx[3]  = 0xff;//dirb
  rx[4]  = 0x00;//pola
  rx[5]  = 0x00;//polb
  rx[6]  = 0x00;//gpintena
  rx[7]  = 0x00;//gpintenb
  rx[8]  = 0x00;//defvala
  rx[9]  = 0x00;//defvalb
  rx[10] = 0x00;//intcona
  rx[11] = 0x00;//intconb
  rx[12] = 0x08;//iocon
  rx[13] = 0x08;//iocon
  rx[14] = 0xff;//gppua
  rx[15] = 0xff;//gppub
  rx[16] = 0x00;//intfa
  rx[17] = 0x00;//infbb
  rx[18] = 0x00;//intcapa
  rx[19] = 0x00;//intcapb
  rx[20] = 0xff;//gpioa
  rx[21] = 0xff;//gpiob

  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  HAL_TIM_Base_Start_IT(&htim1);
  /* Infinite loop */
  while (1){
     
     //estop in test
     // if(HAL_GPIO_ReadPin(GPIOB,  GPIO_PIN_3)){
     //    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
     // }else{
     //    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
     // }
     
     
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi2, tx, 22, 1000);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
     
    // tx[0]  = 0x40;
    // tx[20] = ~led[4];//gpioa
    // tx[21] = ~led[5];//gpiob
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi2, tx, 22, 1);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    // HAL_Delay(1);
    //
    // tx[0]  = 0x42;
    // tx[20] = ~led[2];//gpioa
    // tx[21] = ~led[3];//gpiob
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi2, tx, 22, 1);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    // HAL_Delay(1);
    //
    // tx[0]  = 0x44;
    // tx[20] = ~led[0];//gpioa
    // tx[21] = ~led[1];//gpiob
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi2, tx, 22, 1);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    // HAL_Delay(1);
     
     //out config+write
     tx[0]  = 0x40;//opcode
     tx[20] = out[0];//gpioa
     tx[21] = out[1];//gpiob
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
     HAL_SPI_Transmit(&hspi1, tx, 22, 1);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
     
     tx[0]  = 0x42;//opcode
     tx[20] = out[2];//gpioa
     tx[21] = out[3];//gpiob
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
     HAL_SPI_Transmit(&hspi1, tx, 22, 1);
     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
     
     //in config
     rx[0]  = 0x40;//opcode
     rx[1]  = 0x00;//addr
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
     HAL_SPI_Transmit(&hspi3, rx, 16, 1);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

     rx[0]  = 0x42;//opcode
     rx[1]  = 0x00;//addr
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
     HAL_SPI_Transmit(&hspi3, rx, 16, 1);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
     
     //read
     rx[0]  = 0x41;//opcode
     rx[1]  = 0x12;//addr
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
     HAL_SPI_TransmitReceive(&hspi3, rx, inbuf, 4, 1);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
     in[0] = ~inbuf[2];
     in[1] = ~inbuf[3];
     
     rx[0]  = 0x43;//opcode
     rx[1]  = 0x12;//addr
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
     HAL_SPI_TransmitReceive(&hspi3, rx, inbuf, 4, 1);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
     in[2] = ~inbuf[2];
     in[3] = ~inbuf[3];
     HAL_Delay(1);

     // tx[0]  = 0x40;
     // tx[20] = 0xff;//gpioa
     // tx[21] = 0xff;//gpiob
     // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
     // HAL_SPI_Transmit(&hspi1, tx, 22, 1);
     // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
     //
     // tx[0]  = 0x42;
     // tx[20] = 0xff;//gpioa
     // tx[21] = 0xff;//gpiob
     // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
     // HAL_SPI_Transmit(&hspi1, tx, 22, 1);
     // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
     //
     // HAL_Delay(100);
    
    }


     
     
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

void spi13_init(void){
   GPIO_InitTypeDef GPIO_InitStruct;
   /* Peripheral clock enable */
   __HAL_RCC_SPI1_CLK_ENABLE();
  
   /**SPI1 GPIO Configuration
   PA4     ------> SPI1_CS
   PA5     ------> SPI1_SCK
   PA6     ------> SPI1_MISO
   PA7     ------> SPI1_MOSI 
   */
   
   GPIO_InitStruct.Pin = GPIO_PIN_4;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   
   GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_6;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /* Peripheral clock enable */
   __HAL_RCC_SPI3_CLK_ENABLE();
  
   /**SPI3 GPIO Configuration
   PC9      ------> SPI3_CS
   PC10     ------> SPI3_SCK
   PC11     ------> SPI3_MISO
   PC12     ------> SPI3_MOSI 
   */
   
   GPIO_InitStruct.Pin = GPIO_PIN_9;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
   
   GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_11;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   __HAL_AFIO_REMAP_SPI3_ENABLE();
   
   hspi1.Instance = SPI1;
   hspi1.Init.Mode = SPI_MODE_MASTER;
   hspi1.Init.Direction = SPI_DIRECTION_2LINES;
   hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
   hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
   hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
   hspi1.Init.NSS = SPI_NSS_SOFT;
   hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
   hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
   hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
   hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
   hspi1.Init.CRCPolynomial = 10;
   HAL_SPI_Init(&hspi1);
   
   hspi3.Instance = SPI3;
   hspi3.Init.Mode = SPI_MODE_MASTER;
   hspi3.Init.Direction = SPI_DIRECTION_2LINES;
   hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
   hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
   hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
   hspi3.Init.NSS = SPI_NSS_SOFT;
   hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
   hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
   hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
   hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
   hspi3.Init.CRCPolynomial = 10;
   HAL_SPI_Init(&hspi3);
}

void spi2_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* Peripheral clock enable */
  __HAL_RCC_SPI2_CLK_ENABLE();

  /**SPI2 GPIO Configuration    
  PB13     ------> SPI2_SCK
  PB14     ------> SPI2_MISO
  PB15     ------> SPI2_MOSI
  PB12     ------> SPI2_CS
  */

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;//4 => 9 mbit
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);
}

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
