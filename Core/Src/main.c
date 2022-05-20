/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32F413h_discovery_lcd.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
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
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm2_channel7;

I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s2;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;

SRAM_HandleTypeDef hsram1;

/* Definitions for MetricChange */
osThreadId_t MetricChangeHandle;
const osThreadAttr_t MetricChange_attributes = { .name = "MetricChange",
		.priority = (osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* Definitions for Motor */
osThreadId_t MotorHandle;
const osThreadAttr_t Motor_attributes = { .name = "Motor", .priority =
		(osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* Definitions for Lidar */
osThreadId_t LidarHandle;
const osThreadAttr_t Lidar_attributes = { .name = "Lidar", .priority =
		(osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* Definitions for LCD */
osThreadId_t LCDHandle;
const osThreadAttr_t LCD_attributes = { .name = "LCD", .priority =
		(osPriority_t) osPriorityNormal, .stack_size = 128 * 4 };
/* USER CODE BEGIN PV */
#define MOTOR_TASK_RATE 75
#define LIDAR_TASK_RATE 75
#define LCD_TASK_RATE 200
#define METRIC_TASK_RATE 300
#define CM_TO_INCH 0.393701
#define CM_TO_FEET 0.0328084
#define CM_TO_METER 0.01
uint8_t lidar_rx_buffer[9];
//5A 06 03 00 00 00
uint8_t lidar_tx_buffer_init[] = { 0x5A, 0x06, 0x03, 0x01, 0x00, 0x00 };
//5A 05 05 02 00
uint8_t lidar_tx_buffer[] = { 0x5A, 0x04, 0x04, 0x00 };
extern uint8_t metric_change = 0;
uint8_t curr_metric = 0;
uint8_t distance_high;
uint8_t distance_low;
double metric_multiplier = 1;
char metric[10] = "cm";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DFSDM2_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2S2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_UART7_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
void MetricChangeTask(void *argument);
void MotorTask(void *argument);
void LidarTask(void *argument);
void LCDTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DFSDM1_Init();
	MX_DFSDM2_Init();
	MX_FSMC_Init();
	MX_I2S2_Init();
	MX_SDIO_SD_Init();
	MX_UART7_Init();
	MX_I2C2_Init();
	MX_QUADSPI_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */
	BSP_LCD_Init();
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

	HAL_UART_Transmit(&huart7, lidar_tx_buffer_init, 6, 50);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */

	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of MetricChange */
	MetricChangeHandle = osThreadNew(MetricChangeTask, NULL,
			&MetricChange_attributes);

	/* creation of Motor */
	MotorHandle = osThreadNew(MotorTask, NULL, &Motor_attributes);

	/* creation of Lidar */
	LidarHandle = osThreadNew(LidarTask, NULL, &Lidar_attributes);

	/* creation of LCD */
	LCDHandle = osThreadNew(LCDTask, NULL, &LCD_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1
			| RCC_PERIPHCLK_DFSDM1 | RCC_PERIPHCLK_SDIO | RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
	PeriphClkInitStruct.PLLI2S.PLLI2SM = 12;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
	PeriphClkInitStruct.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_APB2;
	PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_SYSCLK;
	PeriphClkInitStruct.PLLI2SSelection = RCC_PLLI2SCLKSOURCE_PLLSRC;
	PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief DFSDM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DFSDM1_Init(void) {

	/* USER CODE BEGIN DFSDM1_Init 0 */

	/* USER CODE END DFSDM1_Init 0 */

	/* USER CODE BEGIN DFSDM1_Init 1 */

	/* USER CODE END DFSDM1_Init 1 */
	hdfsdm1_channel1.Instance = DFSDM1_Channel1;
	hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
	hdfsdm1_channel1.Init.OutputClock.Selection =
	DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
	hdfsdm1_channel1.Init.OutputClock.Divider = 2;
	hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
	hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
	hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
	hdfsdm1_channel1.Init.SerialInterface.Type =
	DFSDM_CHANNEL_MANCHESTER_RISING;
	hdfsdm1_channel1.Init.SerialInterface.SpiClock =
	DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
	hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
	hdfsdm1_channel1.Init.Awd.Oversampling = 1;
	hdfsdm1_channel1.Init.Offset = 0;
	hdfsdm1_channel1.Init.RightBitShift = 0x00;
	if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DFSDM1_Init 2 */

	/* USER CODE END DFSDM1_Init 2 */

}

/**
 * @brief DFSDM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DFSDM2_Init(void) {

	/* USER CODE BEGIN DFSDM2_Init 0 */

	/* USER CODE END DFSDM2_Init 0 */

	/* USER CODE BEGIN DFSDM2_Init 1 */

	/* USER CODE END DFSDM2_Init 1 */
	hdfsdm2_channel1.Instance = DFSDM2_Channel1;
	hdfsdm2_channel1.Init.OutputClock.Activation = ENABLE;
	hdfsdm2_channel1.Init.OutputClock.Selection =
	DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
	hdfsdm2_channel1.Init.OutputClock.Divider = 2;
	hdfsdm2_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
	hdfsdm2_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
	hdfsdm2_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
	hdfsdm2_channel1.Init.SerialInterface.Type =
	DFSDM_CHANNEL_MANCHESTER_RISING;
	hdfsdm2_channel1.Init.SerialInterface.SpiClock =
	DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
	hdfsdm2_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
	hdfsdm2_channel1.Init.Awd.Oversampling = 1;
	hdfsdm2_channel1.Init.Offset = 0;
	hdfsdm2_channel1.Init.RightBitShift = 0x00;
	if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel1) != HAL_OK) {
		Error_Handler();
	}
	hdfsdm2_channel7.Instance = DFSDM2_Channel7;
	hdfsdm2_channel7.Init.OutputClock.Activation = ENABLE;
	hdfsdm2_channel7.Init.OutputClock.Selection =
	DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
	hdfsdm2_channel7.Init.OutputClock.Divider = 2;
	hdfsdm2_channel7.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
	hdfsdm2_channel7.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
	hdfsdm2_channel7.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
	hdfsdm2_channel7.Init.SerialInterface.Type =
	DFSDM_CHANNEL_MANCHESTER_RISING;
	hdfsdm2_channel7.Init.SerialInterface.SpiClock =
	DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
	hdfsdm2_channel7.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
	hdfsdm2_channel7.Init.Awd.Oversampling = 1;
	hdfsdm2_channel7.Init.Offset = 0;
	hdfsdm2_channel7.Init.RightBitShift = 0x00;
	if (HAL_DFSDM_ChannelInit(&hdfsdm2_channel7) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DFSDM2_Init 2 */

	/* USER CODE END DFSDM2_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void) {

	/* USER CODE BEGIN I2S2_Init 0 */

	/* USER CODE END I2S2_Init 0 */

	/* USER CODE BEGIN I2S2_Init 1 */

	/* USER CODE END I2S2_Init 1 */
	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_8K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2S2_Init 2 */

	/* USER CODE END I2S2_Init 2 */

}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void) {

	/* USER CODE BEGIN QUADSPI_Init 0 */

	/* USER CODE END QUADSPI_Init 0 */

	/* USER CODE BEGIN QUADSPI_Init 1 */

	/* USER CODE END QUADSPI_Init 1 */
	/* QUADSPI parameter configuration*/
	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 255;
	hqspi.Init.FifoThreshold = 1;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
	hqspi.Init.FlashSize = 1;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	hqspi.Init.FlashID = QSPI_FLASH_ID_1;
	hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN QUADSPI_Init 2 */

	/* USER CODE END QUADSPI_Init 2 */

}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void) {

	/* USER CODE BEGIN SDIO_Init 0 */

	/* USER CODE END SDIO_Init 0 */

	/* USER CODE BEGIN SDIO_Init 1 */

	/* USER CODE END SDIO_Init 1 */
	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 0;
	if (HAL_SD_Init(&hsd) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SDIO_Init 2 */

	/* USER CODE END SDIO_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 16;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 1000;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART7_Init(void) {

	/* USER CODE BEGIN UART7_Init 0 */

	/* USER CODE END UART7_Init 0 */

	/* USER CODE BEGIN UART7_Init 1 */

	/* USER CODE END UART7_Init 1 */
	huart7.Instance = UART7;
	huart7.Init.BaudRate = 115200;
	huart7.Init.WordLength = UART_WORDLENGTH_8B;
	huart7.Init.StopBits = UART_STOPBITS_1;
	huart7.Init.Parity = UART_PARITY_NONE;
	huart7.Init.Mode = UART_MODE_TX_RX;
	huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart7.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart7) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART7_Init 2 */

	/* USER CODE END UART7_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
	LED1_RED_Pin | MEMS_LED_Pin | LCD_BL_CTRL_Pin | LEFT_MOTOR_2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, RIGHT_MOTOR_I_Pin | RIGHT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(RIGHT_MOTOR_IA4_GPIO_Port, RIGHT_MOTOR_IA4_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_13 | LCD_TE_Pin | WIFI_WKUP_Pin | LEFT_MOTOR_I_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, USB_OTG_FS_PWR_EN_Pin | GPIO_PIN_13,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : LED1_RED_Pin MEMS_LED_Pin LCD_BL_CTRL_Pin LEFT_MOTOR_2_Pin */
	GPIO_InitStruct.Pin = LED1_RED_Pin | MEMS_LED_Pin | LCD_BL_CTRL_Pin
			| LEFT_MOTOR_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : RIGHT_MOTOR_I_Pin RIGHT_Pin */
	GPIO_InitStruct.Pin = RIGHT_MOTOR_I_Pin | RIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_A0_Pin ARD_A5_Pin */
	GPIO_InitStruct.Pin = ARD_A0_Pin | ARD_A5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : CTP_INT_Pin */
	GPIO_InitStruct.Pin = CTP_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CTP_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : METRIC_CHANGE_Pin */
	GPIO_InitStruct.Pin = METRIC_CHANGE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(METRIC_CHANGE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RIGHT_MOTOR_IA4_Pin */
	GPIO_InitStruct.Pin = RIGHT_MOTOR_IA4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RIGHT_MOTOR_IA4_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_A3_Pin */
	GPIO_InitStruct.Pin = ARD_A3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_A3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_A4_Pin */
	GPIO_InitStruct.Pin = ARD_A4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_A4_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_Detect_Pin */
	GPIO_InitStruct.Pin = SD_Detect_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SD_Detect_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D13_Pin */
	GPIO_InitStruct.Pin = ARD_D13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
	HAL_GPIO_Init(ARD_D13_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB13 LCD_TE_Pin WIFI_WKUP_Pin LEFT_MOTOR_I_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | LCD_TE_Pin | WIFI_WKUP_Pin
			| LEFT_MOTOR_I_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_OVRCR_Pin CODEC_INT_Pin */
	GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_Pin | CODEC_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin PG13 */
	GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : I2CFMP1_SCL_Pin I2CFMP_SDA_Pin */
	GPIO_InitStruct.Pin = I2CFMP1_SCL_Pin | I2CFMP_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_FMPI2C1;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D10_Pin */
	GPIO_InitStruct.Pin = ARD_D10_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(ARD_D10_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PG9 PG14 */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : ARD_D12_Pin ARD_D11_Pin */
	GPIO_InitStruct.Pin = ARD_D12_Pin | ARD_D11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : ARD_D9_Pin */
	GPIO_InitStruct.Pin = ARD_D9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(ARD_D9_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void) {

	/* USER CODE BEGIN FSMC_Init 0 */

	/* USER CODE END FSMC_Init 0 */

	FSMC_NORSRAM_TimingTypeDef Timing = { 0 };

	/* USER CODE BEGIN FSMC_Init 1 */

	/* USER CODE END FSMC_Init 1 */

	/** Perform the SRAM1 memory initialization sequence
	 */
	hsram1.Instance = FSMC_NORSRAM_DEVICE;
	hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	/* hsram1.Init */
	hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
	hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
	hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	hsram1.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;
	hsram1.Init.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
	hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
	/* Timing */
	Timing.AddressSetupTime = 15;
	Timing.AddressHoldTime = 15;
	Timing.DataSetupTime = 255;
	Timing.BusTurnAroundDuration = 15;
	Timing.CLKDivision = 16;
	Timing.DataLatency = 17;
	Timing.AccessMode = FSMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN FSMC_Init 2 */

	/* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_MetricChangeTask */
/**
 * @brief  Function implementing the MetricChange thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_MetricChangeTask */
void MetricChangeTask(void *argument) {
	/* init code for USB_HOST */
	MX_USB_HOST_Init();
	/* USER CODE BEGIN 5 */
	const TickType_t xFrequency = METRIC_TASK_RATE;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		long start = xTaskGetTickCount();
		if (metric_change != curr_metric) {
			if (metric_change == 0) {
				//metric is cm
				metric_multiplier = 1;
			} else if (metric_change == 1) {
				//metric is inch
				metric_multiplier = CM_TO_INCH;
			} else if (metric_change == 2) {
				//metric is feet
				metric_multiplier = CM_TO_FEET;
			} else if (metric_change == 3) {
				//metric is meter
				metric_multiplier = CM_TO_METER;
			}
			curr_metric = metric_change;
		}
		long stop = xTaskGetTickCount() - start;
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MotorTask */
/**
 * @brief Function implementing the Motor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_MotorTask */
void MotorTask(void *argument) {
	/* USER CODE BEGIN MotorTask */
	MX_USB_HOST_Init();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
	const TickType_t xFrequency = MOTOR_TASK_RATE;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		long start = xTaskGetTickCount();
		if (distance_high > 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 850);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 850);
		} else if (distance_low > 150) {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, distance_low * 3);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, distance_low * 3);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		} else if (distance_low > 50) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, distance_low * 3.5);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, distance_low * 3.5);
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 500);

			// Pseudocode for turning and calculating area
			// At the beginning, before starting to move, get the distance and store into a variable
			// Then move until a certain threshold
			// Once at the wall, turn 90 degrees to face the next wall
			// Then repeat the steps, before moving, get distance and store into variable
			// At the beginning of each turn check a turn counter to see if 3 turns have been performed.
			// If turn counter is 3, calculate area, if it is less than 3 then continue like normal
		}
		long stop = xTaskGetTickCount() - start;
		osDelay(1);
	}
	/* USER CODE END MotorTask */
}

/* USER CODE BEGIN Header_LidarTask */
/**
 * @brief Function implementing the Lidar thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LidarTask */
void LidarTask(void *argument) {
	/* USER CODE BEGIN LidarTask */
	const TickType_t xFrequency = LIDAR_TASK_RATE;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		long start = xTaskGetTickCount();
		HAL_StatusTypeDef status = HAL_UART_GetState(&huart7);
		// send 5A 04 04 00 to tell lidar sensor to send data
		HAL_StatusTypeDef statusT = HAL_UART_Transmit(&huart7, lidar_tx_buffer, 4, 25);

		//get 9 bytes of data and place into buffer
		HAL_StatusTypeDef statusR = HAL_UART_Receive_DMA(&huart7, lidar_rx_buffer, 9);
		if (lidar_rx_buffer[0] == 89 && lidar_rx_buffer[1] == 89) {
			distance_high = lidar_rx_buffer[3];
			distance_low = lidar_rx_buffer[2];
		}
		long stop = xTaskGetTickCount() - start;
		osDelay(1);
	}
	/* USER CODE END LidarTask */
}

/* USER CODE BEGIN Header_LCDTask */
/**
 * @brief Function implementing the LCD thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LCDTask */
void LCDTask(void *argument) {
	/* USER CODE BEGIN LCDTask */
	const TickType_t xFrequency = LCD_TASK_RATE;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		long start = xTaskGetTickCount();
		char num_string[30] = { 0 };
		char string_high[15] = { 0 };
		char string_low[15] = { 0 };
		if (curr_metric == 0) {
			metric[0] = '\0';
			strcpy(metric, " cm   ");
		} else if (curr_metric == 1) {
			metric[0] = '\0';
			strcpy(metric, " inch   ");
		}
		if (curr_metric == 2) {
			metric[0] = '\0';
			strcpy(metric, " ft   ");
		}
		if (curr_metric == 3) {
			metric[0] = '\0';
			strcpy(metric, " meter  ");
		}
		uint16_t temp = metric_multiplier
				* (distance_high * 255 + distance_low);

		itoa(temp, string_high, 10);
		strcat(num_string, string_high);
		strcat(num_string, metric);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(0, 112, 240, 15);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DisplayStringAt(0, 112, num_string, CENTER_MODE);
		long stop = xTaskGetTickCount() - start;
		osDelay(1);
	}
	/* USER CODE END LCDTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
