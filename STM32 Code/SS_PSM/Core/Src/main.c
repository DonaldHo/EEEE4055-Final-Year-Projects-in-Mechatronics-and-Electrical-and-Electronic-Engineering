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

// Check and set PWM with different phase shift.
double checkPhaseShift(double phaseShift);
void setPhaseShift(double phaseShift);

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
#define ADC_BUF_LEN 160  // 16kHz/160=100Hz. Same as oscilloscope.
#define NO_ADC_READ 5
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc4;

MDF_HandleTypeDef AdfHandle0;
MDF_FilterConfigTypeDef AdfFilterConfig0;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;
OSPI_HandleTypeDef hospi2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
// Buffers for ADC.
uint16_t Vpv_Buf[ADC_BUF_LEN];
uint16_t Ipv_Buf[ADC_BUF_LEN];
uint16_t buff_count = 0;

// Struct for LLC and converter.
struct LLC_param myLLC = {0};

struct PandO_param myPandO = {0};

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
 * 5 - Voltage tracking controller.
 * 6 - Potential meter to change frequency.
 * 7 - Test voltage tracking using linear power supply.
 * */
uint16_t MODE = 6;


float Voc = 0.00;
float temperature = 0.00;
float Isc = 0.00;
float Pmax = 0.00;


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
static void MX_ADF1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ICACHE_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_OCTOSPI2_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UCPD1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC4_Init(void);
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
  int uart_buffer_len = 0;
  char uart_buffer[50] = "\0";
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
  MX_ADF1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ICACHE_Init();
  MX_OCTOSPI1_Init();
  MX_OCTOSPI2_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_UCPD1_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_ADC4_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  // Greetings.
  uart_buffer_len = sprintf(uart_buffer, "\r\nSmart GMPPT for LLC converter!\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

  // Start timer 16 for algorithm execution.
  HAL_TIM_Base_Start_IT(&htim16);

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Wait for 2.0 second to get Voc.
  HAL_Delay(2000);
  double Vpv_tmp = movingAverage(Vpv_Buf) / 4095 * 3.3;
  // Extrapolate Vpv.
  Voc = 20.019 * Vpv_tmp - 0.6189;

  // Format and transmit over UART.
  uart_buffer_len = sprintf(uart_buffer, "\nVoc=%.2f\r\n", Voc);
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

  Voc = 35.3551;    // Comment this in real when doing whole system.
  Voc = Voc + 2.54; // Add offset.

  temperature = 25 + (Voc - 12.64*3) / (-0.36099/100 * 12.64 * 3);
  Isc = 8.62 + (temperature-25)*(0.06371/100*8.62);
  Pmax = 249.8472 + (temperature-25)*(-0.4417/100*249.8472);
  uart_buffer_len = sprintf(uart_buffer, "\nInit param=%.2f %.2f %.2f %.2f\r\n\n", temperature, Voc, Isc, Pmax);
  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
  ///////////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // PWM generation using timer 1 (PFM).
  TIM1->CCR1 = htim1.Init.Period * 0.5;			// Initially set duty cycle of 50% at 100kHz (Resonance frequency).
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);		// Start CH1.
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // Start CH1N.
  TIM1->BDTR &= ~TIM_BDTR_DTG;  				// Clear previous dead-time.
  TIM1->BDTR |= (16 & TIM_BDTR_DTG);    		// Set dead-time of 100ns (16/160MHz).

  // Phase shifted PWM signal using Master Slave mode.
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);

  // PWM generation using timer 8 (Secondary -> PSM).
  TIM8->CCR1 = htim1.Init.Period * 0.5;			// Initially set duty cycle of 50% at 100kHz (Resonance frequency).
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);		// Start CH4.
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_4);  // Start CH4N.
  TIM8->BDTR &= ~TIM_BDTR_DTG;  				// Clear previous dead-time.
  TIM8->BDTR |= (16 & TIM_BDTR_DTG);    		// Set dead-time of 100ns (16/160MHz).

  TIM1->CCR2 = 0;
  TIM1->EGR |= TIM_EGR_UG;
  ///////////////////////////////////////////////////////////////////////////////////////////////

  // Wait for 0.5 second for stabilization.
  HAL_Delay(500);

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Init Vpv and Ipv accordingly.
  // Initialize voltage scanning mode.
  if (MODE == 1) initVscanning(&myVscanning, 0.4*12.64, 0.4+12.64, 0.4+2*12.64);
  // Initialize P&O mode and start with 100kHz.
  else if (MODE == 3) initPandO(&myLLC, &myPandO, 0, 0, 0, 1, 0.5);
  // Initialize voltage tracking mode.
  else if (MODE == 5) {
	  // Need to solve SS-PSM Equation to get PhiInit.
	  initVcontrol(&myLLC, &myVcontrol, 11.00, 2.1);
	  uart_buffer_len = sprintf(uart_buffer, "fsw_vcontrol  %.2f\r\n", myLLC.PhiOld);
	  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////


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
	  if (flag == 1) {
		  // Toggle to see 100Hz of sampling frequency.
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
  * @brief ADF1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADF1_Init(void)
{

  /* USER CODE BEGIN ADF1_Init 0 */

  /* USER CODE END ADF1_Init 0 */

  /* USER CODE BEGIN ADF1_Init 1 */

  /* USER CODE END ADF1_Init 1 */

  /**
    AdfHandle0 structure initialization and HAL_MDF_Init function call
  */
  AdfHandle0.Instance = ADF1_Filter0;
  AdfHandle0.Init.CommonParam.ProcClockDivider = 1;
  AdfHandle0.Init.CommonParam.OutputClock.Activation = DISABLE;
  AdfHandle0.Init.SerialInterface.Activation = ENABLE;
  AdfHandle0.Init.SerialInterface.Mode = MDF_SITF_LF_MASTER_SPI_MODE;
  AdfHandle0.Init.SerialInterface.ClockSource = MDF_SITF_CCK0_SOURCE;
  AdfHandle0.Init.SerialInterface.Threshold = 4;
  AdfHandle0.Init.FilterBistream = MDF_BITSTREAM0_FALLING;
  if (HAL_MDF_Init(&AdfHandle0) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    AdfFilterConfig0 structure initialization

    WARNING : only structure is filled, no specific init function call for filter
  */
  AdfFilterConfig0.DataSource = MDF_DATA_SOURCE_BSMX;
  AdfFilterConfig0.Delay = 0;
  AdfFilterConfig0.CicMode = MDF_ONE_FILTER_SINC4;
  AdfFilterConfig0.DecimationRatio = 2;
  AdfFilterConfig0.Gain = 0;
  AdfFilterConfig0.ReshapeFilter.Activation = DISABLE;
  AdfFilterConfig0.HighPassFilter.Activation = DISABLE;
  AdfFilterConfig0.SoundActivity.Activation = DISABLE;
  AdfFilterConfig0.AcquisitionMode = MDF_MODE_ASYNC_CONT;
  AdfFilterConfig0.FifoThreshold = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  AdfFilterConfig0.DiscardSamples = 0;
  /* USER CODE BEGIN ADF1_Init 2 */

  /* USER CODE END ADF1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30909DEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30909DEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
  HAL_OSPI_DLYB_CfgTypeDef HAL_OSPI_DLYB_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_APMEMORY;
  hospi1.Init.DeviceSize = 23;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 2;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi1.Init.ChipSelectBoundary = 10;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 100;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.DQSPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_OSPI_DLYB_Cfg_Struct.Units = 0;
  HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = 0;
  if (HAL_OSPI_DLYB_SetConfig(&hospi1, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief OCTOSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI2_Init(void)
{

  /* USER CODE BEGIN OCTOSPI2_Init 0 */

  /* USER CODE END OCTOSPI2_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
  HAL_OSPI_DLYB_CfgTypeDef HAL_OSPI_DLYB_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI2_Init 1 */

  /* USER CODE END OCTOSPI2_Init 1 */
  /* OCTOSPI2 parameter configuration*/
  hospi2.Instance = OCTOSPI2;
  hospi2.Init.FifoThreshold = 4;
  hospi2.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi2.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi2.Init.DeviceSize = 26;
  hospi2.Init.ChipSelectHighTime = 2;
  hospi2.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi2.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi2.Init.ClockPrescaler = 4;
  hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi2.Init.ChipSelectBoundary = 0;
  hospi2.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi2.Init.MaxTran = 0;
  hospi2.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi2) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 2;
  sOspiManagerCfg.DQSPort = 2;
  sOspiManagerCfg.NCSPort = 2;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_2_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_2_HIGH;
  if (HAL_OSPIM_Config(&hospi2, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_OSPI_DLYB_Cfg_Struct.Units = 0;
  HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = 0;
  if (HAL_OSPI_DLYB_SetConfig(&hospi2, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI2_Init 2 */

  /* USER CODE END OCTOSPI2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi2, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 799;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1599;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 799;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim8, TIM_CHANNEL_4);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**UCPD1 GPIO Configuration
  PA15 (JTDI)   ------> UCPD1_CC1
  PB15   ------> UCPD1_CC2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Mems_STSAFE_RESET_Pin|WRLS_WKUP_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WRLS_FLOW_Pin Mems_VLX_GPIO_Pin Mems_INT_LPS22HH_Pin */
  GPIO_InitStruct.Pin = WRLS_FLOW_Pin|Mems_VLX_GPIO_Pin|Mems_INT_LPS22HH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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

  /*Configure GPIO pin : USER_Button_Pin */
  GPIO_InitStruct.Pin = USER_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin Mems_VL53_xshut_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|Mems_VL53_xshut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_CCK1_Pin */
  GPIO_InitStruct.Pin = MIC_CCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
  HAL_GPIO_Init(MIC_CCK1_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : USB_UCPD_FLT_Pin Mems_ISM330DLC_INT1_Pin */
  GPIO_InitStruct.Pin = USB_UCPD_FLT_Pin|Mems_ISM330DLC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Function to set phase shift.
 */
void setPhaseShift(double phaseShift){
	// Set phase relationship between timers
	// Wait for update to complete to ensure safe counter value changes
	while (!(TIM1->SR & TIM_SR_UIF));
	TIM1->SR &= ~TIM_SR_UIF;  // Clear update flag

	TIM1->CNT = phaseShift;
	TIM8->CNT = 0;
}


/**
 * Function to check phase shift.
 */
double checkPhaseShift(double phaseShift) {
	if (phaseShift < PhiMinimum) {
		phaseShift = PhiMinimum;
	} else if (phaseShift > PhiMaximum) {
		phaseShift = PhiMaximum;
	}
	return phaseShift;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Function to compute GMPPT algorithm at frequency of 100Hz.
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

	uart_buffer_len = sprintf(uart_buffer, "%.2f, %.2f %.2f\t", Vpv, Ipv, Vpv*Ipv);
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
			// Function declaration.
			// Initiate starting finit for voltage scanning.
			initVcontrol(&myLLC, &myVcontrol, myVscanning.Vref[myVscanning.count], 2.1);
			// 0 -> 1
			// 1 -> 2
			// 2 -> 3
			myVscanning.count = myVscanning.count + 1;
			// This will make flag == 1

			uart_buffer_len = sprintf(uart_buffer, "Set: flag=%d, C=%d, Vref=%.2f, finit=%.2f\t", myVcontrol.flag, myVscanning.count-1, myVscanning.Vref[myVscanning.count-1], myLLC.PhiOld);
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

		// Check and set phase shift.
		myLLC.PhiOld = checkPhaseShift(myLLC.PhiOld);
		setPhaseShift(myLLC.PhiOld/360*1600);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "Phi1=%.2fdeg\r\n", myLLC.PhiOld);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	else if (MODE == 2) {
		// Input currents sampled from voltage scanning. can use this directly.
//		myVscanning.Inn[0]
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
		solveSSPSM(VGMPPT, PGMPPT, &myLLC.PhiOld);

		// Check and set phase shift.
		myLLC.PhiOld = checkPhaseShift(myLLC.PhiOld);
		setPhaseShift(myLLC.PhiOld/360*1600);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "Phi2=%.2fdeg\r\n", myLLC.PhiOld);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);

		// Initialize P&O algorithm.
		initPandO(&myLLC, &myPandO, Vpv, Ipv, myLLC.PhiOld, 1, 0.5);

		// Go to P&O with initial frequency using FHA.
		MODE = 3;
	}
	else if (MODE == 3) {
		// P&O algorithm.
		PandO(&myLLC, &myPandO, Vpv, Ipv);

		// Check and set phase shift.
		myLLC.PhiOld = checkPhaseShift(myLLC.PhiOld);
		setPhaseShift(myLLC.PhiOld/360*1600);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "Phi3=%.2fdeg\r\n", myLLC.PhiOld);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	else if (MODE == 5) {
		// Always run.
		Vcontrol(&myLLC, &myVcontrol, Vpv);

		// Check and set phase shift.
		myLLC.PhiOld = checkPhaseShift(myLLC.PhiOld);
		setPhaseShift(myLLC.PhiOld/360*1600);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "Phi5=%.2fdeg\r\n", myLLC.PhiOld);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	else if (MODE == 6) {
		// Map ADC voltage phase shift.
		float Phi = (Vpv / 60) * 360;   // Max is 72/360*1600.
		myLLC.PhiOld = Phi;
		// Check and set phase shift.
		myLLC.PhiOld = checkPhaseShift(myLLC.PhiOld);
		setPhaseShift(myLLC.PhiOld/360*1600);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "Phi6=%.2fdeg\r\n", myLLC.PhiOld);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	// Just for testing.
	else if (MODE == 7) {
		double Ppv = Vpv*Ipv;

		if (fabs(Ppv - 15) < 0.50) {}
		else if ((Ppv - 15) > 0 ) {
			myLLC.PhiOld = myLLC.PhiOld + 1;
		}
		else {
			// Decrease frequency to increase power.
			myLLC.PhiOld = myLLC.PhiOld - 1;
		}

		// Check and set phase shift.
		myLLC.PhiOld = checkPhaseShift(myLLC.PhiOld);
		setPhaseShift(myLLC.PhiOld/360*1600);

		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "Phi7=%.2fdeg\r\n", myLLC.PhiOld);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
	else {
		// Format and transmit over UART.
		uart_buffer_len = sprintf(uart_buffer, "\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, uart_buffer_len, HAL_MAX_DELAY);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////


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
