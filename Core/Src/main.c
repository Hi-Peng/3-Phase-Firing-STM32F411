/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
#define SCR_PULSE_WIDTH 1000

// Global alpha phase setting
int16_t sudut_alpha = 2;
uint32_t counter_encoder = 0;
int16_t counter_last_val = 0;

uint8_t first_wave_flag_1 = 0;
uint8_t first_wave_flag_2 = 0;
uint8_t first_wave_flag_3 = 0;

// Button state
uint8_t button_device_select_state = DEVICE_SCR;
uint8_t button_mode_select_state = MODE_UNIDIRECTIONAL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void set_alpha(int alpha);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	// MX_USB_DEVICE_Init();
	// MX_ADC1_Init();
	MX_TIM10_Init();
	/* USER CODE BEGIN 2 */

	__HAL_TIM_CLEAR_IT(&htim2 ,TIM_IT_UPDATE);
	// Initializing Timer Base
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start(&htim5);

	// Initializing One Pulse mode for unidirectional mode
	HAL_TIM_OnePulse_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_OnePulse_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_OnePulse_Start_IT(&htim4, TIM_CHANNEL_2);

	// Intializing Output Compare mode for bidirectional mode
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);

	// Initializing Encoder
	HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);

	// Init sudut_alpha to remove unwanted timer compare error
	// Basede on trial and error, we found out that 2 uS is enough
	set_alpha(sudut_alpha);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */
	TIM_OnePulse_InitTypeDef sConfigOP = { 0 };
	/* USER CODE END TIM2_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 60-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 20000-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_BOTHEDGE;
	sSlaveConfig.TriggerFilter = 12;
	if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */
	sConfigOP.OCMode = TIM_OCMODE_PWM2;
	sConfigOP.Pulse = 1000;
	sConfigOP.OCPolarity = TIM_OCPOLARITY_HIGH;

	sConfigOP.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigOP.ICPolarity = TIM_ICPOLARITY_FALLING;
	sConfigOP.ICFilter = 0;
	if (HAL_TIM_OnePulse_ConfigChannel(&htim2, &sConfigOP, TIM_CHANNEL_2,
			TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */
	TIM_OnePulse_InitTypeDef sConfigOP = { 0 };
	/* USER CODE END TIM3_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 60-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 20000-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_BOTHEDGE;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	sConfigOP.OCMode = TIM_OCMODE_PWM2;
	sConfigOP.Pulse = 1000;
	sConfigOP.OCPolarity = TIM_OCPOLARITY_HIGH;

	sConfigOP.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigOP.ICPolarity = TIM_ICPOLARITY_FALLING;
	sConfigOP.ICFilter = 0;
	if (HAL_TIM_OnePulse_ConfigChannel(&htim3, &sConfigOP, TIM_CHANNEL_2,
			TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */
	TIM_OnePulse_InitTypeDef sConfigOP = { 0 };
	/* USER CODE END TIM4_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 60-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 20000-1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_BOTHEDGE;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */
	sConfigOP.OCMode = TIM_OCMODE_PWM2;
	sConfigOP.Pulse = 1000;
	sConfigOP.OCPolarity = TIM_OCPOLARITY_HIGH;

	sConfigOP.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigOP.ICPolarity = TIM_ICPOLARITY_FALLING;
	sConfigOP.ICFilter = 0;
	if (HAL_TIM_OnePulse_ConfigChannel(&htim4, &sConfigOP, TIM_CHANNEL_2,
			TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void)
{

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 60000;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 100;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_PIN_Pin */
	GPIO_InitStruct.Pin = LED_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_PIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : FIRING_MODE_Pin DEVICE_SEL_Pin */
	GPIO_InitStruct.Pin = FIRING_MODE_Pin|DEVICE_SEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

// TRIAC Mode Init
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	// Zero cross detector Handler
	if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (button_device_select_state != 1){
				TIM2->CCMR2 = (TIM2->CCMR2 & ~TIM_CCMR2_OC3M) | (5<< TIM_CCMR2_OC3M_Pos);
				if (first_wave_flag_1 != 0) {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, sudut_alpha);
					TIM2->CCMR2 = (TIM2->CCMR2 & ~TIM_CCMR2_OC3M) | (4 << TIM_CCMR2_OC3M_Pos);
					first_wave_flag_1 = 0;
				} else {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
							(sudut_alpha+SCR_PULSE_WIDTH));
					TIM2->CCMR2 = (TIM2->CCMR2 & ~TIM_CCMR2_OC3M) | (5 << TIM_CCMR2_OC3M_Pos);
					first_wave_flag_1 = 1;
				}
			}
		}
	}

	// Phase B
	if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (button_device_select_state != 1){
				TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | (5<< TIM_CCMR2_OC3M_Pos);
				if (first_wave_flag_2 != 0) {
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, sudut_alpha);
					TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | (4 << TIM_CCMR2_OC3M_Pos);
					first_wave_flag_2 = 0;
				} else {
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
							(sudut_alpha+SCR_PULSE_WIDTH));
					TIM2->CCMR2 = (TIM2->CCMR2 & ~TIM_CCMR2_OC3M) | (5 << TIM_CCMR2_OC3M_Pos);
					first_wave_flag_2 = 1;
				}
			}
		}
	}

	// Phase C
	if (htim->Instance == TIM4) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (button_device_select_state != 1){
				TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC3M) | (5<< TIM_CCMR2_OC3M_Pos);
				if (first_wave_flag_3 != 0) {
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, sudut_alpha);
					TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC3M) | (4 << TIM_CCMR2_OC3M_Pos);
					first_wave_flag_3 = 0;
				} else {
					__HAL_TIM_SET_COMPARE(&htim4	, TIM_CHANNEL_3,
							(sudut_alpha+SCR_PULSE_WIDTH));
					TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC3M) | (5 << TIM_CCMR2_OC3M_Pos);
					first_wave_flag_3 = 1;
				}
			}
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5) {
		uint16_t current_cnt;
		current_cnt = __HAL_TIM_GET_COUNTER(&htim5);
		uint16_t diff = (uint16_t)(current_cnt - counter_last_val);

		if (diff == 0u)
		{
			// No change
			// sudut_alpha = diff;
		}
		else if (diff & 0x8000u)
		{
			// Counter has decreased, diff is negative
			sudut_alpha += (diff * 100);

		}
		else
		{
			// Counter has increased, diff is positive
			sudut_alpha += (diff * 100);
		}

		counter_last_val = current_cnt;

		if (sudut_alpha <= 5){
			sudut_alpha = 2;
		}
		else if(sudut_alpha >= 10000){
			sudut_alpha = 10000;
		}

		set_alpha(sudut_alpha);
		HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);
		//				counter_encoder = (int16_t) __HAL_TIM_GET_COUNTER(&htim5);
		//				if (!__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5)){
		//					htim
		//				}
		//				sudut_alpha = (int16_t) __HAL_TIM_GET_COUNTER(&htim5) * 100;
		//				if (__HAL_TIM_GET_COUNTER(&htim5) <= 1 && __HAL_TIM_GET_COUNTER(&htim5) >= 4194967295){
		//					sudut_alpha = 50;
		//					__HAL_TIM_SET_COUNTER(&htim5, 1);
		//				}
		//				else if(__HAL_TIM_GET_COUNTER(&htim5) >= 85 && __HAL_TIM_GET_COUNTER(&htim5) <= 100){
		//					sudut_alpha = 8500;
		//					__HAL_TIM_SET_COUNTER(&htim5, 85);
		//				}
	}
}

void set_alpha(int alpha) {
	if (button_device_select_state == DEVICE_SCR){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (alpha + PI_ANGLE_TIM));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (alpha));
		__HAL_TIM_SET_AUTORELOAD(&htim2, (alpha + 10000 + SCR_PULSE_WIDTH));

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (alpha + PI_ANGLE_TIM));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (alpha));
		__HAL_TIM_SET_AUTORELOAD(&htim3, (alpha+10000+SCR_PULSE_WIDTH));

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (alpha + PI_ANGLE_TIM));
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (alpha));
		__HAL_TIM_SET_AUTORELOAD(&htim4, (alpha + 10000 + SCR_PULSE_WIDTH));
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (alpha));
		TIM2->CCMR2 = (TIM2->CCMR2 & ~TIM_CCMR2_OC3M) | (4 << TIM_CCMR2_OC3M_Pos);
		__HAL_TIM_SET_AUTORELOAD(&htim2, (alpha + SCR_PULSE_WIDTH));

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (alpha));
		TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | (4 << TIM_CCMR2_OC3M_Pos);
		__HAL_TIM_SET_AUTORELOAD(&htim3, (alpha + SCR_PULSE_WIDTH));

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (alpha));
		TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC3M) | (4 << TIM_CCMR2_OC3M_Pos);
		__HAL_TIM_SET_AUTORELOAD(&htim4, (alpha + SCR_PULSE_WIDTH));
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == DEVICE_SEL_Pin && button_device_select_state == DEVICE_TRIAC) // If The INT Source Is EXTI Line9 (A9 Pin)
	{
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
		HAL_TIM_Base_Start_IT(&htim10);
		button_device_select_state = DEVICE_SCR;
	}
	else if (GPIO_Pin == DEVICE_SEL_Pin && button_device_select_state == DEVICE_SCR) {
		button_device_select_state = DEVICE_TRIAC;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
	 */
	if(htim->Instance == TIM10){
		if(HAL_GPIO_ReadPin(DEVICE_SEL_GPIO_Port, DEVICE_SEL_Pin) == GPIO_PIN_RESET){
			button_device_select_state = DEVICE_TRIAC;
			HAL_TIM_Base_Stop_IT(&htim10);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_BOTHEDGE);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_ICPOLARITY_BOTHEDGE);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_ICPOLARITY_BOTHEDGE);
		}
	}


	/*
	 * Reconfigure channel 3 OC when the timers end (CNT>ARR).
	 * Set the CCRx to the previous sudut_alpha and also set the OC mode as active on match
	 * This must set the CCMR2 register 4-6 bits to 001b or 4U
	 */
	if (htim->Instance == TIM2) {
		TIM2->CCMR2 = (TIM2->CCMR2 & ~TIM_CCMR2_OC3M) | (4 << TIM_CCMR2_OC3M_Pos);
		TIM2->CCR3 = sudut_alpha;
	}

	if (htim->Instance == TIM3) {
		TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | (4 << TIM_CCMR2_OC3M_Pos);
		TIM3->CCR3 = sudut_alpha;
	}

	if (htim->Instance == TIM4) {
		TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC3M) | (4 << TIM_CCMR2_OC3M_Pos);
		TIM4->CCR3 = sudut_alpha;
	}

}

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
	while (1) {
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
