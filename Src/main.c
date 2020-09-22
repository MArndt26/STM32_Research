/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *
 * This code will blink the blue led while it waits for user to press b1
 * this will trigger the adc inputs --> serial
 * BAUD RATE = 115200
 * TX --> PA9
 * RX --> PA10
 *
 * Push Button
 * PA0
 *
 * LED
 * Blue --> PC8
 * Green -> PC9
 *
 *
 * ADC
 * 1 --> PA1
 * 2 --> PA2
 * 3 --> PA3
 * 4 --> PA4
 * 5 --> PA5
 * 6 --> PA6
 * 7 --> PA7
 * 8 --> PB0
 * 9 --> PB1
 * 10 -> PC0
 *
 * SPI
 * SCK --> PB3
 * MISO -> PB4
 * MOSI -> PB5
 * CS  --> PB6
 *
 *
 * Questions to ponder:
 * - what value of Max_ss --> max sector size are needed
 * 	I cannot do 4096 --> currently at 512
 *
 *
 * Resources Used:
 * https://www.youtube.com/watch?v=spVIZO-jbxE
 *
 * new resources:
 * - https://01001000.xyz/2020-08-09-Tutorial-STM32CubeIDE-SD-card/
 * - https://github.com/kiwih/cubeide-sd-card
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <fatfs_sd.h>
#include <string.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHOW_UART_WRITE 0
#define ADC_NUM_CHANNELS 10
#define ADC_PRINT_BUF_SIZE 480
#define ADC_NUM_MUX_CHANNELS 30

#define CMD_START 's'
#define CMD_CREATE_DEFAULT 'd'
#define ACK_INVALID 'e'
#define CMD_VIEW 'v'
#define CMD_HALT 'f'
#define CMD_LOAD 'l'
#define CMD_HELP 'h'

#define UART_BUF_SIZE 1
#define MSG_STR_SIZE 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum STATE {
	IDLE,
	HELPING,
	LOADING_FILE,
	LOADED,
	VIEWING,
	RUNNING,
	CREATING_FILE,
	CLOSING_FILE
};

enum STATE cur_state = IDLE;

volatile int muxState = 0;

volatile uint16_t adc_buf[ADC_NUM_CHANNELS]; //working buffer for the adc values
volatile uint16_t adc_print_buf[ADC_PRINT_BUF_SIZE]; //to current buffer for the adc values
volatile int adc_flag = 0;
volatile int adc_buf_ready = 0;
volatile int adc_print_buf_ofset = 0;

volatile int line_count = 0;
volatile int numConversions = 0;
volatile int numPrints = 0;

char RxData[UART_BUF_SIZE];

FATFS fs;          // file system
FIL fil;           // file
FRESULT fresult;   // to store the result
//char buffer[1024]; // to store data

UINT br, bw; // file read/write count

/* capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

volatile int bp = 0;                   //number of times button has been pressed
char str[MSG_STR_SIZE]; //string var for sending to usart

char name[9];

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart1, (uint8_t *) &RxData, UART_BUF_SIZE);

	send_uart("Begin 10 Chan ADC to Micro SD\n");

	// Calibrate The ADC On Power-Up For Better Accuracy
	HAL_ADCEx_Calibration_Start(&hadc);

	HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		switch (cur_state) {
		case IDLE:
			blink(1, LD4_BLUE_LED_GPIO_Port, LD4_BLUE_LED_Pin);
			break;
		case HELPING:
			//print commands
			send_uart("s\nd\nv\nf\n");
			break;
		case LOADING_FILE:
			break;
		case CREATING_FILE:
			mount_sd();
			create_file();
			fresult = f_open(&fil, name, FA_OPEN_ALWAYS | FA_WRITE); // Open the file with write access
			cur_state = LOADED;
			break;
		case LOADED:
			blink(1, LD3_GREEN_LED_GPIO_Port, LD3_GREEN_LED_Pin);
			break;
		case VIEWING:
			//todo: write function to view file preview
			cur_state = LOADED;
			break;
		case RUNNING:
			if (adc_flag == 0) {
				//restart adc collection

				// Pass (The ADC Instance, Result Buffer Address, Buffer Length)
				HAL_ADC_Start_DMA(&hadc, (uint32_t*) &adc_buf,
				ADC_NUM_CHANNELS);

				line_count = 0;

				adc_flag = 1;
			} else if (adc_buf_ready) {
				adc_buf_ready = 0;

				numPrints++;


				int temp = 0;
				//f_write(file pointer, pointer to data buffer, number of bytes to write, pointer to variable to return number of bytes written)
				fresult = f_write(&fil, adc_print_buf, ADC_PRINT_BUF_SIZE * 2,
						&temp);

				line_count += temp / (ADC_NUM_MUX_CHANNELS * 2);

				if (fresult != FR_OK) {
					sprintf(str, "main f_printf err: %d\n", fresult);
					send_uart(str);
				}
			}

			break;
		case CLOSING_FILE:

			HAL_ADC_Stop_DMA(&hadc);

			adc_flag = 0;

			/* close file */
			fresult = f_close(&fil);

			if (fresult != FR_OK) {
				sprintf(str, "main f_printf err: %d\n", fresult);
			}

			send_uart("Data Collection Halted\n\n");

			unmount_sd();

			cur_state = IDLE;

			break;
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 4000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2-1;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEL_A_Pin|SEL_B_Pin|SEL_C_Pin|LD4_BLUE_LED_Pin
                          |LD3_GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : SEL_A_Pin SEL_B_Pin SEL_C_Pin LD4_BLUE_LED_Pin
                           LD3_GREEN_LED_Pin */
  GPIO_InitStruct.Pin = SEL_A_Pin|SEL_B_Pin|SEL_C_Pin|LD4_BLUE_LED_Pin
                          |LD3_GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*********************User Defined Functions********************/

/* to send the data to the uart */
void send_uart(char *string) {
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart1, (uint8_t *) string, len, 2000); // transmit in blocking mode
}

/* to find the size of data in the buffer */
int bufsize(char *buf) {
	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

void mount_sd() {
	/* Mount SD Card */
	fresult = f_mount(&fs, "", 1);
	if (fresult != FR_OK)
		send_uart("error in mounting SD CARD...\n");
	else
		send_uart("SD CARD mounted successfully...\n");
}

void read_card_details() {
	/*************** Card capacity details ********************/

	/* Check free space */
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf(str, "SD CARD Total Size: \t%d\n", total);
	send_uart(str);
	bufclear();
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	sprintf(str, "SD CARD Free Space: \t%d\n", free_space);
	send_uart(str);
}

void create_file() {
	/*************** Create File For Data Storage ********************/

	int fileNumber = 0;

	//check if filename exist
	sprintf(name, "F%d.bin", fileNumber);
	while (f_stat(name, NULL) == FR_OK) {
		fileNumber++;
		sprintf(name, "F%d.bin", fileNumber);
	}

	/* once filename is new create file */
	fresult = f_open(&fil, name, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	if (fresult != FR_OK) {
		sprintf(str, "f_open err: %d\n", fresult);
	}

	/* Writing text */
	fresult = f_printf(&fil,
			"ADC0 ADC1 ADC2 ADC3 ADC4 ADC5 ADC6 ADC7 ADC8 ADC9\n");

	if (fresult != FR_OK) {
		sprintf(str, "f_printf err: %d\n", fresult);
	}

	/* Close file */
	fresult = f_close(&fil);

	if (fresult != FR_OK) {
		sprintf(str, "f_close err: %d\n", fresult);
	}

	send_uart(name); //ex: File1.bin created and is ready for data to be written

	send_uart(" created and header was written \n");
}

void unmount_sd() {
	/* Unmount SDCARD */
	fresult = f_mount(NULL, "", 1);
	if (fresult == FR_OK)
		send_uart("SD CARD UNMOUNTED successfully...\n");

	sprintf(str, "line count: %d\n", line_count);

	send_uart(str);

	sprintf(str, "conversion ct: %d\n", numConversions);

	send_uart(str);

	sprintf(str, "num prints: %d\n", numPrints);

	send_uart(str);

	//adc values buffered with dma
	sprintf(str, "dma vals: %d\n", numConversions * 10);

	send_uart(str);

	//adc vals printed in main loop
	sprintf(str, "main vals: %d\n", numPrints * ADC_PRINT_BUF_SIZE);

	send_uart(str);
}

/*Wrapper to blink LEDs*/
void blink(int num_blinks, GPIO_TypeDef* port, uint16_t pin) {
	//blink led <num_blink> times to show that data collection is initialized
	for (int i = 0; i < num_blinks * 2; i++) {
		HAL_GPIO_TogglePin(port, pin);
		HAL_Delay(100); //1000ms delay
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	switch (muxState) {
	case 0:
		HAL_GPIO_WritePin(SEL_A_GPIO_Port, SEL_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SEL_B_GPIO_Port, SEL_B_Pin, GPIO_PIN_RESET);
		muxState = 2;
		break;
	case 2:
		HAL_GPIO_WritePin(SEL_A_GPIO_Port, SEL_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL_B_GPIO_Port, SEL_B_Pin, GPIO_PIN_RESET);
		muxState = 3;
		break;
	case 3:
		HAL_GPIO_WritePin(SEL_A_GPIO_Port, SEL_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEL_B_GPIO_Port, SEL_B_Pin, GPIO_PIN_SET);
		muxState = 0;
		break;
//	case 4: //currently implemented as synonym for case 0
//		HAL_GPIO_WritePin(SEL_A_GPIO_Port, SEL_A_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(SEL_B_GPIO_Port, SEL_B_Pin, GPIO_PIN_RESET);
//		muxState = 6;
//		break;
//	case 6: //currently implemented as synonym for case 2
//		HAL_GPIO_WritePin(SEL_A_GPIO_Port, SEL_A_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(SEL_B_GPIO_Port, SEL_B_Pin, GPIO_PIN_RESET);
//		muxState = 7;
//		break;
//	case 7: //currently implemented as synonym for case 3
//		HAL_GPIO_WritePin(SEL_A_GPIO_Port, SEL_A_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(SEL_B_GPIO_Port, SEL_B_Pin, GPIO_PIN_SET);
//		muxState = 0;
//		break;
	}

	int i;
	for (i = adc_print_buf_ofset; i < adc_print_buf_ofset + ADC_NUM_CHANNELS;
			i++) {
		adc_print_buf[i] = adc_buf[i - adc_print_buf_ofset]; // store the values in adc[]
	}
	adc_print_buf_ofset = i;

	if (adc_print_buf_ofset > ADC_PRINT_BUF_SIZE) {
		adc_print_buf_ofset = 0;
		adc_buf_ready = 1;
	}

	numConversions++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle) {
	//echoback command for debugging
	HAL_UART_Transmit(&huart1, (uint8_t *) &RxData, UART_BUF_SIZE, 1000); // transmit in blocking mode
	send_uart("\n");
	HAL_UART_Receive_IT(&huart1, (uint8_t *) &RxData, UART_BUF_SIZE); //restart listening for interrupt

	int valid_cmd = 0;

	switch (cur_state) {
	case IDLE:
		if (RxData[0] == CMD_CREATE_DEFAULT) {
			valid_cmd = 1;
			cur_state = CREATING_FILE;
		} else if (RxData[1] == CMD_HELP) {
			valid_cmd = 1;
			cur_state = HELPING;
		}
		break;
	case LOADED:
		if (RxData[0] == CMD_START) {
			valid_cmd = 1;
			cur_state = RUNNING;
		} else if (RxData[0] == CMD_VIEW) {
			valid_cmd = 1;
			cur_state = VIEWING;
		}
		break;
	case RUNNING:
		if (RxData[0] == CMD_HALT) {
			valid_cmd = 1;
			cur_state = CLOSING_FILE;
		}
	}

	if (!valid_cmd) //notify user of invalid command entered
	{
		char c_buff = ACK_INVALID;
		HAL_UART_Transmit(&huart1, &c_buff, 1, 1000); // transmit in blocking mode

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
