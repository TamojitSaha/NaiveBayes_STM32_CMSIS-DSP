#include "main.h"
#include <string.h>
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>
#include "arm_math.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void sendData(char * data, UART_HandleTypeDef * huart);
void NaiveBayes(void);

/* 
These parameters can be generated with the python library scikit-learn.
*/
arm_gaussian_naive_bayes_instance_f32 S;

#define NB_OF_CLASSES 3
#define VECTOR_DIMENSION 2

const float32_t theta[NB_OF_CLASSES*VECTOR_DIMENSION] = {
  1.4539529436590528f, 0.8722776016801852f, 
  -1.5267934452462473f, 0.903204577814203f, 
  -0.15338006360932258f, -2.9997913665803964f
}; /**< Mean values for the Gaussians */


const float32_t sigma[NB_OF_CLASSES*VECTOR_DIMENSION] = {
  1.0063470889514925f, 0.9038018246524426f, 
  1.0224479953244736f, 0.7768764290432544f, 
  1.1217662403241206f, 1.2303890106020325f
}; /**< Variances for the Gaussians */

const float32_t classPriors[NB_OF_CLASSES] = {
  0.3333333333333333f, 0.3333333333333333f, 0.3333333333333333f
}; /**< Class prior probabilities */


int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
	HAL_Delay(2000);
	char buff[50] = "Serial Data\n";
	HAL_UART_Transmit(&huart2,buff,strlen(buff),500);
	
	
	
	NaiveBayes();
  while (1)
  {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_Delay(500);
  }

}

void NaiveBayes(void){

	float32_t in[2];
	char buff[80];

  /* Result of the classifier */
  float32_t result[NB_OF_CLASSES];
  float32_t maxProba;
  uint32_t index;
  
  S.vectorDimension = VECTOR_DIMENSION; 
  S.numberOfClasses = NB_OF_CLASSES; 
  S.theta = theta;          
  S.sigma = sigma;         
  S.classPriors = classPriors;    
  S.epsilon=4.328939296523643e-09f; 

  in[0] = 1.5f;
  in[1] = 1.0f;

  index = arm_gaussian_naive_bayes_predict_f32(&S, in, result);
	sprintf(buff,"1.5f, 1.0f in Class: %zu\n",index);
	HAL_UART_Transmit(&huart2,buff,strlen(buff),500);

  maxProba = result[index];
	
//	sendData((char *)"\nData: 1.5f,1.0f; Class:",&huart2);
//	sendData((char *)&index,&huart2);
	

	#if defined(SEMIHOSTING)
		printf("Class = %d\n", index);
	#endif

		in[0] = -1.5f;
		in[1] = 1.0f;

		index = arm_gaussian_naive_bayes_predict_f32(&S, in, result);
		sprintf(buff,"-1.5f, 1.0f in Class: %zu\n",index);
		HAL_UART_Transmit(&huart2,buff,strlen(buff),500);
		
		maxProba = result[index];
	//	sendData((char *)"\nData: -1.5f,1.0f; Class:",&huart2);
	//	sendData((char *)&index,&huart2);

	#if defined(SEMIHOSTING)
		printf("Class = %d\n", index);
	#endif

		in[0] = 0.0f;
		in[1] = -3.0f;

		index = arm_gaussian_naive_bayes_predict_f32(&S, in, result);
		sprintf(buff,"0.1f, -3.0f in Class: %zu\n",index);
		HAL_UART_Transmit(&huart2,buff,strlen(buff),500);

		maxProba = result[index];

}

void sendData(char * data, UART_HandleTypeDef * huart){
  // UART can only send unsigned int
  // Thus we need to convert our struct data
  char buffer[sizeof(data)]; // Create a char buffer of right size
  
  // Copy the data to buffer
  memcpy(buffer, &data, sizeof(data)); // Copy and convert the data
  // Ideally buffer will be 12B long, 4B for each axes data  // Now we can finally send this data
  HAL_UART_Transmit(huart, (uint8_t *)buffer, sizeof(buffer), 50);
  // The last param is timeout duration in ms
}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA6 PA7 PA8 PA9 
                           PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB13 PB14 PB15 
                           PB3 PB4 PB5 PB6 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
