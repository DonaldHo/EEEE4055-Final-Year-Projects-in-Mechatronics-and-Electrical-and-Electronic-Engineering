/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "llc_converter.h"

#include "ai_platform.h"
#include "ann.h"
#include "ann_data.h"
#include "cnn.h"
#include "cnn_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Function used for ADC.
void bubbleSort(uint32_t *arr, uint32_t size);
void swap(uint32_t *a, uint32_t *b);
double movingAverage(uint16_t *x);

// MPPT algorithm.
void smartGMPPT(double Vpv, double Ipv);

// Check and set PWM.
double checkFreq(double switching_freq);
void setFreq(double switching_freq);

// AI model.
static void InitCNN(void);
static void RunCNN(float *pIn, float *pOut);
static void InitANN(void);
static void RunANN(float *pIn, float *pOut);
void normalizeANN(float I1, float I2, float I3);
void denormalizeANN(float *PredictedOutput);
void normalizeCNN(float I1, float I2, float I3);
void denormalizeCNN(float *PredictedOutput);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// ADC buffer.
#define ADC_BUF_LEN 160 // 16kHz/160=100Hz. Same as oscilloscope.
#define NO_ADC_READ 5

// Maximum and minimum frequency,
#define fminimum 45000
#define fmaximum 250000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Buffers for ADC.
uint16_t Vpv_Buf[ADC_BUF_LEN];
uint16_t Ipv_Buf[ADC_BUF_LEN];
uint16_t buff_count = 0;

// Struct for LLC and converter.
struct LLC_param myLLC = {0};

struct PandO_param myPandO = {0};
struct CPI_param myCPI = {0};

struct Vscanning_param myVscanning = {0};
struct Vcontrol_param myVcontrol = {0};

struct ADC_param myADC = {0};
struct Kalman_param myVpv = {0};
struct Kalman_param myIpv = {0};
/**
 * MODE :
 * 1 - Voltage scanning at Vscan = [0.4*Voc, 0.4+1Voc, 0.4+2Voc]
 * 2 - NN predicted using Inn[3] -> VGMPPT, PGMPPT -> finit.
 * 3 - P&O.
 * 4 - CPI.
 * 5 - Voltage tracking controller.
 * 6 - Potential meter to change frequency.
 * 7 - Test voltage tracking using linear power supply.
 * */
uint16_t MODE = 6;


float Voc = 0.00;
float temperature = 0.00;
float Isc = 0.00;
float Pmax = 0.00;


// Global variable to select topology .
uint16_t HB = 0;
uint16_t Quadrupler = 0;


// AI model initialization.
// ANN.
ai_handle ANNModel = AI_HANDLE_NULL;
ai_u8 activationsANN[AI_ANN_DATA_ACTIVATIONS_SIZE];
float aiInDataANN[AI_ANN_IN_1_SIZE];
float aiOutDataANN[AI_ANN_OUT_1_SIZE];
ai_buffer* ai_input_ann;
ai_buffer* ai_output_ann;

// CNN.
ai_handle CNNModel = AI_HANDLE_NULL;
ai_u8 activationsCNN[AI_CNN_DATA_ACTIVATIONS_SIZE];
float aiInDataCNN[AI_CNN_IN_1_SIZE];
float aiOutDataCNN[AI_CNN_OUT_1_SIZE];
ai_buffer* ai_input_cnn;
ai_buffer* ai_output_cnn;

// Flag used inside main loop.
int flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC4_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  // Greetings.
  int uart_buffer_len = 0;
  char uart_buffer[50] = "\0";
  uart_buffer_len = sprintf(uart_buffer, "\r\nSmart GMPPT for LLC converter!\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);


  // Start timer 16 to fill buffer.
  HAL_TIM_Base_Start_IT(&htim16);


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Wait for 2.0 second to get Voc.
  HAL_Delay(2000);
  double Vpv_tmp = movingAverage(Vpv_Buf) / 4095 * 3.3;
  // Extrapolate Vpv.
  Voc = 20.019 * Vpv_tmp - 0.6189;


  // Format and transmit over UART.
  uart_buffer_len = sprintf(uart_buffer, "\n%Voc=%.2f\r\n", Voc);
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

  Voc = 35.3551; // Comment this in real when doing whole system.

  Voc = Voc + 2.54; // Add offset.

  temperature = 25 + (Voc - 12.64*3) / (-0.36099/100 * 12.64 * 3);
  Isc = 8.62 + (temperature-25)*(0.06371/100*8.62);
  Pmax = 249.8472 + (temperature-25)*(-0.4417/100*249.8472);
  uart_buffer_len = sprintf(uart_buffer, "\nInit param=%.2f %.2f %.2f %.2f\r\n\n", temperature, Voc, Isc, Pmax);
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // PWM generation using timer 1.
  TIM1->CCR1 = htim1.Init.Period * 0.5;			// Initially set duty cycle of 50% at 100kHz (Resonance frequency).

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);		// Start CH1.
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // Start CH1N.

  TIM1->BDTR &= ~TIM_BDTR_DTG;  				// Clear previous dead-time.
  TIM1->BDTR |= (16 & TIM_BDTR_DTG);    		// Set dead-time of 100ns (16/160MHz).


  // Wait for 0.5 second for stabilization after starting PWM.
  HAL_Delay(500);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Init Vpv and Ipv accordingly.
  // Initialize voltage scanning mode.
  if (MODE == 1) initVscanning(&myVscanning, 0.4*12.64, 0.4+12.64, 0.4+2*12.64);
  // Initialize P&O mode and start with 100kHz.
  else if (MODE == 3) initPandO(&myLLC, &myPandO, 0, 0, 100000, 500, 0.5);
  // Initialize CPI mode.
  else if (MODE == 4) initCPI(&myLLC, &myCPI, fminimum, fmaximum, 0.5);
  // Initialize voltage tracking mode.
  else if (MODE == 5) {
	  // Need to solve FHA to get finit.
	  initVcontrol(&myLLC, &myVcontrol, 11.00, 1);
	  uart_buffer_len = sprintf(uart_buffer, "fsw_vcontrol  %.2f\r\n", myLLC.fold);
	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // Initialize AI model.
  InitANN();
  InitCNN();


  // Initialize KALMAN Filter according to paper.
  initKalmanFilter(&myVpv, 1, 0.1);
  initKalmanFilter(&myIpv, 1, 0.1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (flag==1) {
		// Toggle to see 100Hz of sampling frequency.
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);

		// Here to set HB and Quadrupler.
		// Based on the GAIN value from voltage tracking algo ? FHA ? or anything.


		// Enable Quadrupler.
		if (Quadrupler == 1) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); 	// D1.
		}
		else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);   // D1.
		}

		// Enable half bridge.
		if (HB == 1) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); 	// D2.
		}
		else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);  // D2.
		}


		double Vpv = movingAverage(Vpv_Buf) / 4095 * 3.3;
		// Extrapolate Vpv.
		Vpv = 20.019 * Vpv - 0.6189;
		double Ipv = movingAverage(Ipv_Buf) / 4095 * 3.3;
		// Extrapolate Ipv.
		Ipv = 3.3455 * Ipv + 0.0392;
		// Run algorithm.
		smartGMPPT(Vpv, Ipv);

		// Reset flag.
		flag = 0;
	}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_5CYCLE;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoPowerOff = ADC_LOW_POWER_NONE;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.SamplingTimeCommon1 = ADC4_SAMPLETIME_1CYCLE_5;
  hadc4.Init.SamplingTimeCommon2 = ADC4_SAMPLETIME_1CYCLE_5;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC4_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */
  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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
  sBreakDeadTimeConfig.DeadTime = 16;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UCPD_PWR_GPIO_Port, UCPD_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_RED_Pin|LED_GREEN_Pin|Mems_VL53_xshut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WRLS_WKUP_B_GPIO_Port, WRLS_WKUP_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Mems_STSAFE_RESET_Pin|WRLS_WKUP_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WRLS_FLOW_Pin Mems_VLX_GPIO_Pin Mems_INT_LPS22HH_Pin */
  GPIO_InitStruct.Pin = WRLS_FLOW_Pin|Mems_VLX_GPIO_Pin|Mems_INT_LPS22HH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : WRLS_UART4_RX_Pin WRLS_UART4_TX_Pin */
  GPIO_InitStruct.Pin = WRLS_UART4_RX_Pin|WRLS_UART4_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_UCPD_CC1_Pin */
  GPIO_InitStruct.Pin = USB_UCPD_CC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_UCPD_CC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_F_NCS_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_F_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
  HAL_GPIO_Init(OCTOSPI_F_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_IO5_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_IO5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTOSPI_F_IO7_Pin OCTOSPI_F_IO5_Pin OCTOSPI_F_IO6_Pin OCTOSPI_F_IO4_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_F_IO7_Pin|OCTOSPI_F_IO5_Pin|OCTOSPI_F_IO6_Pin|OCTOSPI_F_IO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3_BOOT0_Pin */
  GPIO_InitStruct.Pin = PH3_BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PH3_BOOT0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_PWR_Pin */
  GPIO_InitStruct.Pin = UCPD_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UCPD_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WRLS_SPI2_MOSI_Pin WRLS_SPI2_MISO_Pin WRLS_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = WRLS_SPI2_MOSI_Pin|WRLS_SPI2_MISO_Pin|WRLS_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_DQS_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_DQS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_DQS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB9 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_IO7_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_IO7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTOSPI_F_IO0_Pin OCTOSPI_F_IO1_Pin OCTOSPI_F_IO2_Pin OCTOSPI_F_IO3_Pin
                           OCTOSPI_F_CLK_P_Pin OCTOSPI_F_DQS_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_F_IO0_Pin|OCTOSPI_F_IO1_Pin|OCTOSPI_F_IO2_Pin|OCTOSPI_F_IO3_Pin
                          |OCTOSPI_F_CLK_P_Pin|OCTOSPI_F_DQS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Button_Pin */
  GPIO_InitStruct.Pin = USER_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PH4 PH5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin Mems_VL53_xshut_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|Mems_VL53_xshut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTOSPI_R_IO0_Pin OCTOSPI_R_IO2_Pin OCTOSPI_R_IO1_Pin OCTOSPI_R_IO3_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO0_Pin|OCTOSPI_R_IO2_Pin|OCTOSPI_R_IO1_Pin|OCTOSPI_R_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_IO4_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_IO4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_C_P_Pin USB_C_PA11_Pin */
  GPIO_InitStruct.Pin = USB_C_P_Pin|USB_C_PA11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_CCK1_Pin */
  GPIO_InitStruct.Pin = MIC_CCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
  HAL_GPIO_Init(MIC_CCK1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MIC_SDINx_Pin MIC_CCK0_Pin */
  GPIO_InitStruct.Pin = MIC_SDINx_Pin|MIC_CCK0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_ADF1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : WRLS_WKUP_B_Pin */
  GPIO_InitStruct.Pin = WRLS_WKUP_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WRLS_WKUP_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WRLS_NOTIFY_Pin Mems_INT_IIS2MDC_Pin USB_IANA_Pin */
  GPIO_InitStruct.Pin = WRLS_NOTIFY_Pin|Mems_INT_IIS2MDC_Pin|USB_IANA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD15 PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OCTOSPI_R_IO6_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_IO6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(OCTOSPI_R_IO6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_UCPD_FLT_Pin Mems_ISM330DLC_INT1_Pin */
  GPIO_InitStruct.Pin = USB_UCPD_FLT_Pin|Mems_ISM330DLC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OCTOSPI_R_CLK_P_Pin OCTOSPI_R_NCS_Pin */
  GPIO_InitStruct.Pin = OCTOSPI_R_CLK_P_Pin|OCTOSPI_R_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WRLS_SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = WRLS_SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(WRLS_SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_UCPD_CC2_Pin */
  GPIO_InitStruct.Pin = USB_UCPD_CC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_UCPD_CC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Mems_STSAFE_RESET_Pin WRLS_WKUP_W_Pin */
  GPIO_InitStruct.Pin = Mems_STSAFE_RESET_Pin|WRLS_WKUP_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_SDIN0_Pin */
  GPIO_InitStruct.Pin = MIC_SDIN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
  HAL_GPIO_Init(MIC_SDIN0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Timer 16 overflow.
	if (htim == &htim16) {
		// Toggle GPIO pin here.
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);

		// Buffer for UART.
//		int uart_buffer_len = 0;
//		char uart_buffer[50] = "\0";

		uint32_t adc4[NO_ADC_READ];
		uint32_t adc1[NO_ADC_READ];

		for (uint8_t i = 0; i < NO_ADC_READ; i++) {
			// Voltage.
			HAL_ADC_Start(&hadc4);
			HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
			adc4[i] = HAL_ADC_GetValue(&hadc4);

			// Current.
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			adc1[i] = HAL_ADC_GetValue(&hadc1);
		}

		bubbleSort(adc4, NO_ADC_READ);
		bubbleSort(adc1, NO_ADC_READ);

		Vpv_Buf[buff_count] = adc4[NO_ADC_READ/2];
		Ipv_Buf[buff_count] = adc1[NO_ADC_READ/2];

		// Display result.
//		uart_buffer_len = sprintf(uart_buffer, "%d, %d, %d\n", buff_count, Vpv_Buf[buff_count], Ipv_Buf[buff_count]);
//		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

		buff_count++;

		// Trigger every 100Hz.
		if (buff_count >= ADC_BUF_LEN) {
			// Flag used inside main loop.
			flag = 1;
			buff_count = 0;
		}
	}
}


/**
 * Function to swap two values in an array.
 */
void swap(uint32_t *a, uint32_t *b) {
	uint32_t temp = *a;
    *a = *b;
    *b = temp;
}


/**
 * Function to sort a float array using Bubble Sort.
 *
 * @param arr Pointer to the input array.
 * @param size Length of the input array.
 */
void bubbleSort(uint32_t *arr, uint32_t size) {
    int swapped;

    do {
        swapped = 0;
        for (int i = 0; i < size - 1; i++) {
            if (arr[i] > arr[i + 1]) {
                swap(&arr[i], &arr[i + 1]);
                swapped = 1;
            }
        }
    } while (swapped);
}


/**
 * Function to compute the moving average of an array.
 *
 * @param x: Pointer to an array of uint16_t values.
 * @return The computed moving average as a double.
 */
double movingAverage(uint16_t *x) {
    double sum = 0;

    // Iterate through the array and sum all elements
    for (uint16_t i = 0; i < ADC_BUF_LEN; i++) {
        sum = sum + x[i];
    }

    // Return the average by dividing the sum by the array length
    return sum / ADC_BUF_LEN;
}


/**
 * Function to compute GMPPT algorithm at frequency of 250Hz.
 *
 * @param Vpv: Input voltage.
 * @param Ipv: Input current.
 */
void smartGMPPT(double Vpv, double Ipv){
	// Buffer for UART.
	int uart_buffer_len = 0;
	char uart_buffer[50] = "\0";

	// Kalman filter (Need to comment this when doing CPI).
//	Vpv = KalmanFilter(&myVpv, Vpv);
//	Ipv = KalmanFilter(&myIpv, Ipv);

	uart_buffer_len = sprintf(uart_buffer, "%.2f, %.2f %.2f %d %d\t", Vpv, Ipv, Vpv*Ipv, HB, Quadrupler);
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);


	// Force off everything.
//	if (Vpv > 50 || Ipv > 10) {
//		TIM1->CR1 &= ~TIM_CR1_CEN;  // Disable TIM1 (Timer stops counting)
//		TIM1->CCER &= ~TIM_CCER_CC1E;  // Disable the PWM output on Channel 1.
//		MODE = 10;
//	}


	int done = 0;

	if (MODE == 1) {
		// VCPI.
		if (myVcontrol.flag == 0 && myVscanning.count < 3) {
			if (myVscanning.count==0) {        //5V
				Quadrupler = 1;
				HB = 0;
			}
			else if (myVscanning.count==1) {   //13V
				Quadrupler = 0;
				HB = 0;
			}
			else if (myVscanning.count==2) {   //25V
				Quadrupler = 0;
				HB = 1;
			}
			else {
				Quadrupler = 0;
				HB = 0;
			}


			// Function declaration.
			// Initiate starting finit for voltage scanning.
			initVcontrol(&myLLC, &myVcontrol, myVscanning.Vref[myVscanning.count], 1.1);
			// 0 -> 1
			// 1 -> 2
			// 2 -> 3
			myVscanning.count = myVscanning.count + 1;
			// This will make flag == 1

			uart_buffer_len = sprintf(uart_buffer, "Set: HB=%d, Q=%d, flag=%d, C=%d, Vref=%.2f, finit=%.2f\t", HB, Quadrupler, myVcontrol.flag, myVscanning.count-1, myVscanning.Vref[myVscanning.count-1], myLLC.fold);
			HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
		}
		else{
			done = Vcontrol(&myLLC, &myVcontrol, Vpv);
		}


		if (done == 1) {
			myVscanning.Inn[myVscanning.count-1] = Ipv;
			// Format and transmit over UART.
			uart_buffer_len = sprintf(uart_buffer, "Inn=%.2f, %.2f, %.2f\r\t", myVscanning.Inn[0], myVscanning.Inn[1], myVscanning.Inn[2]);
			HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

			if (myVscanning.count == 3) {
//				MODE = 2;
				// Force to go other state.
				MODE = 10;

				// Format and transmit over UART.
				uart_buffer_len = sprintf(uart_buffer, "Done voltage scanning!\r\t");
				HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
			}
		}

		// Check and set switching frequency.
		myLLC.fold = checkFreq(myLLC.fold);
		setFreq(myLLC.fold);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "fscan=%.2f\r\n", myLLC.fold);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	else if (MODE == 2) {
		// Input currents sampled from voltage scanning. can use this directly.
		//myVscanning.Inn[0]
		float I1 = 7.4314;
		float I2 = 1.7318;
		float I3 = 1.4145;

		float denormalizedOutput[6];

		/* Run AI prediction */
		normalizeANN(I1, I2, I3);
		RunANN(aiInDataANN, aiOutDataANN);
		denormalizeANN(denormalizedOutput);

//		normalizeCNN(I1, I2, I3);
//		RunCNN(aiInDataCNN, aiOutDataCNN);
//		denormalizeCNN(denormalizedOutput);

		uart_buffer_len = sprintf(uart_buffer, "\r\nOutputs: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\r\n", denormalizedOutput[0], denormalizedOutput[1], denormalizedOutput[2],denormalizedOutput[3], denormalizedOutput[4], denormalizedOutput[5]);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

		// Based on this find the maximum power and voltage and solve FHA below.
		double Pmax = fmax(fmax(denormalizedOutput[0], denormalizedOutput[1]), denormalizedOutput[2]);
		int idx = (fabs(denormalizedOutput[0] - Pmax) < 1e-3) ? 3 :(fabs(denormalizedOutput[1] - Pmax) < 1e-3) ? 4 : 5;
		double Vmax = denormalizedOutput[idx];

		uart_buffer_len = sprintf(uart_buffer, "%.2f, %.2f\t", Pmax, Vmax);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

		// GMPPT prediction.
		double PGMPPT = Pmax;
		double VGMPPT = Vmax;

		// Solving FHA equation.
		solve_FHA_equation(VGMPPT, PGMPPT, &myLLC.fold);

		// See which mode to on.
		if (VGMPPT > TH1) {
			Quadrupler = 0;
			HB = 1;

		}
		else if (VGMPPT > TH2) {
			Quadrupler = 0;
			HB = 0;
		}
		else {
			Quadrupler = 1;
			HB = 0;
		}


		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "finit=%.2f\r\n", myLLC.fold);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

		// Check and set switching frequency.
		myLLC.fold = checkFreq(myLLC.fold);
		setFreq(myLLC.fold);

		// Initialize P&O algorithm.
		initPandO(&myLLC, &myPandO, Vpv, Ipv, myLLC.fold, 500, 0.5);

		// Go to P&O with initial frequency using FHA.
		MODE = 3;
	}
	else if (MODE == 3) {
		// P&O algorithm.
		PandO(&myLLC, &myPandO, Vpv, Ipv);

		// Check and set switching frequency.
		myLLC.fold = checkFreq(myLLC.fold);
		setFreq(myLLC.fold);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "fp&o=%.2f\r\n", myLLC.fold);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	else if (MODE == 4) {
		// CPI algorithm.
		CPI(&myLLC, &myCPI, Vpv, Ipv);

		// Check and set switching frequency.
		myLLC.fold = checkFreq(myLLC.fold);
		setFreq(myLLC.fold);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "fcpi=%.2f\r\n", myLLC.fold);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	else if (MODE == 5) {
		// See which mode to on.
		if (myVcontrol.Vref > 14) {
			Quadrupler = 0;
			HB = 1;
		}
		else if (myVcontrol.Vref > 9) {
			Quadrupler = 0;
			HB = 0;
		}
		else {
			Quadrupler = 1;
			HB = 0;
		}

		// Always run.
		Vcontrol(&myLLC, &myVcontrol, Vpv);

		// Check and set switching frequency.
		myLLC.fold = checkFreq(myLLC.fold);
		setFreq(myLLC.fold);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "fv=%.2f\r\n", myLLC.fold);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	else if (MODE == 6) {
		// Map ADC voltage (0–3.3V) to switching frequency).
		float switch_freq = fminimum + (Vpv / 60) * (fmaximum - fminimum);
		myLLC.fold = switch_freq;
		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "fsw=%.2fHz\r\n", myLLC.fold);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

		// Check and set switching frequency.
		myLLC.fold = checkFreq(myLLC.fold);
		setFreq(myLLC.fold);
	}
	// Just for testing.
	else if (MODE == 7) {
		double Ppv = Vpv*Ipv;

		if (fabs(Ppv - 15) < 0.50) {}
		else if ((Ppv - 15) > 0 ) {
			myLLC.fold = myLLC.fold + 100;
		}
		else {
			// Decrease frequency to increase power.
			myLLC.fold = myLLC.fold - 100;
		}

		// Check and set switching frequency.
		myLLC.fold = checkFreq(myLLC.fold);
		setFreq(myLLC.fold);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "ftest=%.2f\r\n", myLLC.fold);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	else {
		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
}


/**
 * Function to check minimum and maximum threshold switching frequency.
 *
 * @param1 fsw: Switching frequency.
 */
double checkFreq(double switching_freq) {
	if (switching_freq < fminimum) {
		switching_freq = fminimum;
	} else if (switching_freq > fmaximum) {
		switching_freq = fmaximum;
	}
	return switching_freq;
}


/**
 * Function to set switching frequency.
 *
 * @param1 fsw: Switching frequency.
 */
void setFreq(double switching_freq){
	if (switching_freq>=fminimum && switching_freq<=fmaximum) {
		// Compute ARR value for TIM1 based on switch_freq.
		uint32_t TIM1_CLK = 160000000;
		uint32_t ARR_Value = (TIM1_CLK / switching_freq) - 1;

		// Ensure update happens only at next overflow.
		TIM1->SR &= ~TIM_SR_UIF;            // Clear any previous update flag.
		TIM1->ARR = ARR_Value;              // Set new period.
		TIM1->CCR1 = ARR_Value / 2;         // Maintain 50% duty cycle.
		TIM1->EGR |= TIM_EGR_UG;            // Apply update safely at next overflow.

		// Update Dead Time Safely.
		TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | (16 & TIM_BDTR_DTG);  // Set 100ns dead time.
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for AI model.
void normalizeANN(float I1, float I2, float I3) {
	// Normalize input current.
	aiInDataANN[0] = I1 / Isc;
	aiInDataANN[1] = I2 / Isc;
	aiInDataANN[2] = I3 / Isc;
}

void denormalizeANN(float *PredictedOutput) {
	// Denormalize power.
	PredictedOutput[0] = aiOutDataANN[0] * Pmax;
	PredictedOutput[1] = aiOutDataANN[1] * Pmax;
	PredictedOutput[2] = aiOutDataANN[2] * Pmax;

	// Denormalize voltage.
	PredictedOutput[3] = aiOutDataANN[3] * Voc;
	PredictedOutput[4] = aiOutDataANN[4] * Voc;
	PredictedOutput[5] = aiOutDataANN[5] * Voc;
}

void normalizeCNN(float I1, float I2, float I3) {
	// Normalize input current.
	aiInDataCNN[0] = I1 / Isc;
	aiInDataCNN[1] = I2 / Isc;
	aiInDataCNN[2] = I3 / Isc;
}

void denormalizeCNN(float *PredictedOutput) {
	// Denormalize power.
	PredictedOutput[0] = aiOutDataCNN[0] * Pmax;
	PredictedOutput[1] = aiOutDataCNN[1] * Pmax;
	PredictedOutput[2] = aiOutDataCNN[2] * Pmax;

	// Denormalize voltage.
	PredictedOutput[3] = aiOutDataCNN[3] * Voc;
	PredictedOutput[4] = aiOutDataCNN[4] * Voc;
	PredictedOutput[5] = aiOutDataCNN[5] * Voc;
}

static void InitANN(void) {
	ai_error err;

	char buf[50];
	int buf_len = 0;

	/* Create a local array with the addresses of the activations buffers */
	const ai_handle act_addr[] = { activationsANN };
	/* Create an instance of the model */
	err = ai_ann_create_and_init(&ANNModel, act_addr, NULL);
	if (err.type != AI_ERROR_NONE) {
		buf_len = sprintf(buf, "ANNModel_create error - type=%d code=%d\r\n", err.type, err.code);
		HAL_UART_Transmit(&huart1, (uint8_t *)buf, buf_len, 100);
		Error_Handler();
	}
	ai_input_ann = ai_cnn_inputs_get(ANNModel, NULL);
	ai_output_ann = ai_cnn_outputs_get(ANNModel, NULL);
}

static void RunANN(float *pIn, float *pOut) {
  ai_i32 batch;
  ai_error err;

  char buf[50]; // buffer for serial output string
  int buf_len = 0;

  /* Update IO handlers with the data payload */
  ai_input_ann[0].data = AI_HANDLE_PTR(pIn);
  ai_output_ann[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_cnn_run(ANNModel, ai_input_ann, ai_output_ann);
  if (batch != 1) {
    err = ai_cnn_get_error(ANNModel);
    buf_len = sprintf(buf, "AI model_run error - type=%d code=%d\r\n", err.type, err.code);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, buf_len, 100);
    Error_Handler();
  }
}

static void InitCNN(void) {
	ai_error err;

	char buf[50];
	int buf_len = 0;

	/* Create a local array with the addresses of the activations buffers */
	const ai_handle act_addr[] = { activationsCNN };
	/* Create an instance of the model */
	err = ai_cnn_create_and_init(&CNNModel, act_addr, NULL);
	if (err.type != AI_ERROR_NONE) {
		buf_len = sprintf(buf, "CNNModel_create error - type=%d code=%d\r\n", err.type, err.code);
		HAL_UART_Transmit(&huart1, (uint8_t *)buf, buf_len, 100);
		Error_Handler();
	}
	ai_input_cnn = ai_cnn_inputs_get(CNNModel, NULL);
	ai_output_cnn = ai_cnn_outputs_get(CNNModel, NULL);
}

static void RunCNN(float *pIn, float *pOut) {

  ai_i32 batch;
  ai_error err;

  char buf[50]; // buffer for serial output string
  int buf_len = 0;

  /* Update IO handlers with the data payload */
  ai_input_cnn[0].data = AI_HANDLE_PTR(pIn);
  ai_output_cnn[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_cnn_run(CNNModel, ai_input_cnn, ai_output_cnn);
  if (batch != 1) {
    err = ai_cnn_get_error(CNNModel);
    buf_len = sprintf(buf, "AI model_run error - type=%d code=%d\r\n", err.type, err.code);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, buf_len, 100);
    Error_Handler();
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
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
