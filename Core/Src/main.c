/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx.h"
#define ARM_MATH_CM4
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
int u;

volatile uint8_t adc_conv_complete_flag = 0;
volatile uint16_t adc_dma_result[5];
int adc_channel_count = sizeof(adc_dma_result)/sizeof(adc_dma_result[0]);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void GPIO_Init(void);
void ADC1_Init(void);
void TIM2_Init(void);
void TIM4_ms_Delay(uint32_t delay);
int map(int, int, int, int, int);
void servo_write(int);
void servo_sweep();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
	UNUSED(hadc);
	adc_conv_complete_flag = 1;
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
}

void GPIO_Init(){
    //PC0 is connected to ADC1 IN10
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;// Enable GPIOC clock
    GPIOC->MODER |= GPIO_MODER_MODER0; // Set PC0 to Analog mode
    GPIOC->MODER |= GPIO_MODER_MODER1; // Set PC1 to Analog mode
    GPIOC->MODER |= GPIO_MODER_MODER2; // Set PC2 to Analog mode
    GPIOC->MODER |= GPIO_MODER_MODER3; // Set PC3 to Analog mode


    RCC->AHB1ENR |= 1; //Enable GPIOA clock
    GPIOA->AFR[0] |= 0x00100000; // Select the PA5 pin in alternate function mode
    GPIOA->MODER |= 0x00000800; //Set the PA5 pin alternate function

    GPIOA->AFR[0] |= 0x10000000;// Select the PA1 pin in alternate function mode
    GPIOA->MODER |= GPIO_MODER_MODER1_1;
    //GPIOA->AFR[1] |= 0x00100000; //
    //GPIOA->MODER |= 0x00000800;
    GPIOA->AFR[0] |= 0x01000000; // Select the PA2 pin in alternate function mode
    GPIOA->MODER |= GPIO_MODER_MODER2_1;

    GPIOA->AFR[0] |= 0x11000000; // Select the PA3 pin in alternate function mode
    GPIOA->MODER |= GPIO_MODER_MODER3_1;

    GPIOA->AFR[1] |= 0x01000000; // Select AF1 (TIM1) for pin A8
    GPIOA->MODER |= GPIO_MODER_MODER8_1; // Set A8 to Alternate Function mode
}
void ADC1_Init(){
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;//Enable ADC clock
    //APB2 clock is not divided so it runs at 16MHz
    ADC->CCR |= 0<<16; // PCLK2 divide by 2, so the clock frequency is 8MHz
    ADC1->CR2 |= ADC_CR2_CONT; // Setting the Continuous Conversion Mode
    ADC1->CR2 |= ADC_CR2_ADON; // Enabling the ADC
    ADC1->CR2 &= ~ADC_CR2_ALIGN; //setting the alignment to the right
    ADC1->CR1 &= ~ADC_CR1_RES; // Selecting 12-bit resolution
    ADC1->SMPR1 |= (ADC_SMPR1_SMP10_0 | ADC_SMPR1_SMP10_2); // 112 Sampling cycle selection
    ADC1->SMPR1 |= (ADC_SMPR1_SMP11_0 | ADC_SMPR1_SMP11_2); // 112 Sampling cycle selection
    ADC1->SMPR1 |= (ADC_SMPR1_SMP12_0 | ADC_SMPR1_SMP12_2); // 112 Sampling cycle selection
    ADC1->SMPR1 |= (ADC_SMPR1_SMP13_0 | ADC_SMPR1_SMP13_2); // 112 Sampling cycle selection
    ADC1->SMPR1 |= (ADC_SMPR1_SMP14_0 | ADC_SMPR1_SMP14_2); // 112 Sampling cycle selection
    ADC1->SQR3 |= (10<<0)|(11<<5)|(12<<10)|(13<<15)|(14<<20);

}
void TIM2_Init(){
    RCC->APB1ENR |=1;
    TIM2->PSC = 16-1; //Setting the clock frequency to 1MHz.
    TIM2->ARR = 20000; // Total period of the timer
    TIM2->CNT = 0;
    //TIM2->CCMR1 = 0x0060; //PWM mode for the timer
    // Configure TIM2 CH1 for PWM mode
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // PWM mode for CH1
    TIM2->CCER |= TIM_CCER_CC1E; // Enable channel 1 as output

    TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // PWM mode for CH2
    TIM2->CCER |= TIM_CCER_CC2E; // Enable channel 2 as output
    //TIM2->CCR1 = 500; // Pulse width for PWM
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; // PWM mode for CH2
    TIM2->CCER |= TIM_CCER_CC3E; // Enable channel 3 as output

    TIM2->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; // PWM mode for CH2
    TIM2->CCER |= TIM_CCER_CC4E; // Enable channel 4 as output

    TIM2->CCR1 = 500; // Pulse width for PWM
    TIM2->CCR2 = 500;
    TIM2->CCR3 = 500;
    TIM2->CCR4 = 500;
}
void TIM1_Init() {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // Enable TIM1 clock
    TIM1->PSC = 16 - 1; // Setting the clock frequency to 1MHz.
    TIM1->ARR = 20000; // Total period of the timer
    TIM1->CNT = 0;

    // Configure TIM1 CH1 for PWM mode
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // PWM mode for CH1
    TIM1->CCER |= TIM_CCER_CC1E; // Enable channel 1 as output

    // Configure the corresponding GPIO pin (A8) in alternate function mode
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
    GPIOA->MODER |= GPIO_MODER_MODER8_1; // Set A8 to Alternate Function mode
    GPIOA->AFR[1] |= 0x01000000; // Select AF1 (TIM1) for pin A8

    TIM1->CCR1 = 500;
}

//int map(int st1, int fn1, int st2, int fn2, int value)
//{
//    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
//}

//void servo_write(int angle)
//{
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, map(0,180,50,250,angle));
//}

//void servo_sweep(void)
//{
//		for(int i = 0; i <= 180; i++)
//		{
//			servo_write(i);
//			HAL_Delay(10);
//		}
//		for(int i = 180; i >= 0; i--)
//		{
//			servo_write(i);
//			HAL_Delay(10);
//		}
//}
//void TIM4_ms_Delay(uint32_t delay){
//    RCC->APB1ENR |= 1<<2; //Start the clock for the timer peripheral
//    TIM4->PSC = 16000-1; //Setting the clock frequency to 1kHz.
//    TIM4->ARR = (delay); // Total period of the timer
//    TIM4->CNT = 0;
//    TIM4->CR1 |= 1; //Start the Timer
//    while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
//    TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  RCC->CFGR |= 0<<10; // set APB1 = 16 MHz
  GPIO_Init();
  ADC1_Init();
  TIM2_Init();
  TIM2->CR1 |= 1;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_result, adc_channel_count);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
      //ADC1->CR2 |= ADC_CR2_SWSTART; //Starting ADC conversion
      //while (!(ADC1->SR & ADC_SR_EOC)){}
      //u = ADC1->DR;
      //TIM2->CCR1 = (int)(((u/4095)*2000)+500);
      //TIM4_ms_Delay(50);
	  if (adc_conv_complete_flag==1){
      uint16_t adc_value_ch1 = adc_dma_result[0];
      uint16_t pwm_value_ch1 = (adc_value_ch1 * 2000) / 4095 + 500; // Scale ADC value to PWM range

      uint16_t adc_value_ch2 = adc_dma_result[1];
      uint16_t pwm_value_ch2 = (adc_value_ch2 * 2000) / 4095 + 500; // Scale ADC value to PWM range

      uint16_t adc_value_ch3 = adc_dma_result[2];
      uint16_t pwm_value_ch3 = (adc_value_ch3 * 2000) / 4095 + 500; // Scale ADC value to PWM range

      uint16_t adc_value_ch4 = adc_dma_result[3];
      uint16_t pwm_value_ch4 = (adc_value_ch4 * 2000) / 4095 + 500; // Scale ADC value to PWM range

      uint16_t adc_value_ch5 = adc_dma_result[4];
      uint16_t pwm_value_ch5 = (adc_value_ch5 * 2000) / 4095 + 500; // Scale ADC value to PWM range

      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)pwm_value_ch1);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)pwm_value_ch2);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint16_t)pwm_value_ch3);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t)pwm_value_ch4);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t)pwm_value_ch5);
      adc_conv_complete_flag=0;
	  }
      HAL_Delay(10);

      //HAL_ADC_Start(&hadc1);

      //if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
          //uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
          //uint16_t pwm_value = (adc_value * 2000)/4095+500; // Scale ADC value to PWM range
          //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_value);
      //}
      //HAL_Delay(100); // Adjust the delay as needed

	  //for (int i = 0; i<= 2500; i++){s
	  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
	  ////  HAL_Delay(1); // Adjust the delay as needed
	  //}
	  //servo_write(0);
      //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2500);
      //servo_write(200);
      //HAL_Delay(100);
	  //servo_sweep();

      // when adc_conv_complete_flag is set to 1,
      // that means DMA conversion is completed
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4065;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 160;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4065;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
