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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "crc8.h"
#include "conv_enc.h"
#include "scrambler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_LENGTH 100
#define TX_SYM_BUFFER_LENGTH 250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Circular buffer for ADC reading
uint16_t ADC_BUFFER[ADC_BUFFER_LENGTH*2] = {0};
// We unpack the 16QAM symbols into 4bit chunks
// Read as a circular buffer that is reset by ADC ISR
uint8_t TX_SYM_BUFFER[TX_SYM_BUFFER_LENGTH*2*2] = {0};

// We encode our data into this buffer before we unpack it into TX_SYM_BUFFER
uint8_t ENCODING_SCRATCH_BUFFER[TX_SYM_BUFFER_LENGTH] = {0};
// We convert our ADC readings into 8bit for encoding
uint8_t ADC_SCRATCH_BUFFER[ADC_BUFFER_LENGTH] = {0};

// Generate gray code for better FEC
uint8_t GRAY_CODE[16] = {0};

// Start from the second buffer for symbol transmit
// We alternate between halves of the ADC/TX_SYM buffer for double buffered processing
volatile int tx_sym_buffer_index = TX_SYM_BUFFER_LENGTH*2;
volatile int is_half_buffer_ready = 0;
volatile int is_full_buffer_ready = 0;

const uint32_t PREAMBLE_CODE = 0b11111001101011111100110101101101;
const uint16_t SCRAMBLER_CODE = 0b1000010101011001;
const uint8_t CRC8_POLY = 0xD5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void user_init(void);
static int generate_packet(uint8_t* x, const int N, uint8_t* y);
static void adc_buffer_convert(uint16_t* x, uint8_t* y, const int N);
static void qam_symbol_unpack(uint8_t* x, uint8_t* y, const int N);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void user_init(void) {
	crc8_generate_table(CRC8_POLY);
	conv_enc_calculate_table(0b111, 0b101);
	scrambler_init(SCRAMBLER_CODE);

	tx_sym_buffer_index = TX_SYM_BUFFER_LENGTH*2;

	// generate gray code for 2D
	uint8_t _gray_code[4] = {0b00, 0b01, 0b11, 0b10}; // 1D gray code
	for (uint8_t i = 0; i < 4; i++) {
		for (uint8_t q = 0; q < 4; q++) {
			uint8_t I = _gray_code[i];
			uint8_t Q = _gray_code[q];
			uint8_t idx = (i<<2) | q;
			uint8_t sym = (I<<2) | Q;
			// shift by 4 bits to align with output pins
			GRAY_CODE[idx] = sym;
		}
	}
}

// Our encoding pipeline
int generate_packet(uint8_t* x, const int N, uint8_t* y) {
	conv_enc_reset();
	scrambler_reset();

	// We cast this to uint16_t so we can write the trellis terminator along with it
	// NOTE: this uses a big endian notation so the null byte is located correctly without a shift
	uint8_t crc8 = crc8_calculate(x, N);
	uint8_t trellis_terminator = 0x00;
	uint16_t len = (uint16_t)(N);

	int offset = 0;
	// NOTE: We are generating this before hand to save a few instructions
	y[0] = (PREAMBLE_CODE >> 24) & 0xFF;
	y[1] = (PREAMBLE_CODE >> 16) & 0xFF;
	y[2] = (PREAMBLE_CODE >> 8 ) & 0xFF;
	y[3] = (PREAMBLE_CODE      ) & 0xFF;
	offset += 4;

	int scrambler_start = offset;

	// Encode then scramble the output
	offset += conv_enc_process((uint8_t*)(&len),	&y[offset], 2);
	offset += conv_enc_process(x,					&y[offset], N);
	offset += conv_enc_process(&crc8,				&y[offset], 1);
	offset += conv_enc_process(&trellis_terminator,	&y[offset], 1);

	int scrambler_end = offset;

	for (int i = scrambler_start; i < scrambler_end; i++) {
		y[i] = scrambler_process(y[i]);
	}

	return offset;
}

void adc_buffer_convert(uint16_t* x, uint8_t* y, const int N) {
	for (int i = 0; i < N; i++) {
		y[i] = x[i];
	}
}

void qam_symbol_unpack(uint8_t* x, uint8_t* y, const int N) {
	int j = 0;
	for (int i = 0; i < N; i++) {
		y[j++] = GRAY_CODE[x[i] >> 4];
		y[j++] = GRAY_CODE[x[i] & 0x0F];
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	user_init();
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);

  // Circular buffer for adc reading
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_BUFFER, ADC_BUFFER_LENGTH*2);

  HAL_TIM_Base_Start_IT(&htim1);

  // Define the halves of our ADC and TX_SYM buffer
  uint16_t* ADC_BUFFER_0 = &ADC_BUFFER[0];
  uint16_t* ADC_BUFFER_1 = &ADC_BUFFER[ADC_BUFFER_LENGTH];
  uint8_t*  TX_SYM_BUFFER_0 = &TX_SYM_BUFFER[0];
  uint8_t*  TX_SYM_BUFFER_1 = &TX_SYM_BUFFER[TX_SYM_BUFFER_LENGTH*2];
  uint8_t SAMPLE_MESSAGE[13] = {0};
  for (uint8_t i = 0; i < 13; i++) {
	  SAMPLE_MESSAGE[i] = 'a'+i;
  }

  int offset = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// adc:		  writes to second half of ADC_BUFFER
	// tx_isr:    reads from first half of TX_BUFFER
	// main_loop: reads from first half of ADC_BUFFER
	//			  writes to second half of TX_BUFFER
	while (!is_half_buffer_ready);
	is_half_buffer_ready = 0;

	HAL_GPIO_WritePin(ENCODE_FLAG_OUT_GPIO_Port, ENCODE_FLAG_OUT_Pin, GPIO_PIN_SET);

	offset = 0;
	adc_buffer_convert(ADC_BUFFER_0, ADC_SCRATCH_BUFFER, ADC_BUFFER_LENGTH);
	offset += generate_packet(ADC_SCRATCH_BUFFER, ADC_BUFFER_LENGTH, &ENCODING_SCRATCH_BUFFER[offset]);
	offset += generate_packet(SAMPLE_MESSAGE, 13, &ENCODING_SCRATCH_BUFFER[offset]);
	qam_symbol_unpack(ENCODING_SCRATCH_BUFFER, TX_SYM_BUFFER_1, TX_SYM_BUFFER_LENGTH);

	HAL_GPIO_WritePin(ENCODE_FLAG_OUT_GPIO_Port, ENCODE_FLAG_OUT_Pin, GPIO_PIN_RESET);

	// adc:		  writes to first half of ADC_BUFFER
	// tx_isr:    reads from second half of TX_BUFFER
	// main_loop: reads from second half of ADC_BUFFER
	//			  writes to first half of TX_BUFFER
	while (!is_full_buffer_ready);
	is_full_buffer_ready = 0;

	HAL_GPIO_WritePin(ENCODE_FLAG_OUT_GPIO_Port, ENCODE_FLAG_OUT_Pin, GPIO_PIN_SET);

	offset = 0;
	adc_buffer_convert(ADC_BUFFER_1, ADC_SCRATCH_BUFFER, ADC_BUFFER_LENGTH);
	offset += generate_packet(ADC_SCRATCH_BUFFER, ADC_BUFFER_LENGTH, &ENCODING_SCRATCH_BUFFER[offset]);
	offset += generate_packet(SAMPLE_MESSAGE, 13, &ENCODING_SCRATCH_BUFFER[offset]);
	qam_symbol_unpack(ENCODING_SCRATCH_BUFFER, TX_SYM_BUFFER_0, TX_SYM_BUFFER_LENGTH);

	HAL_GPIO_WritePin(ENCODE_FLAG_OUT_GPIO_Port, ENCODE_FLAG_OUT_Pin, GPIO_PIN_RESET);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  htim1.Init.Prescaler = 10-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 42-1;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 5;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

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

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |ENCODE_FLAG_OUT_Pin|TX_SYM_FLAG_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7
                           ENCODE_FLAG_OUT_Pin TX_SYM_FLAG_OUT_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |ENCODE_FLAG_OUT_Pin|TX_SYM_FLAG_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// adc:		  writes to second half of ADC_BUFFER
// tx_isr:    reads from first half of TX_BUFFER
// main_loop: reads from first half of ADC_BUFFER
//			  writes to second half of TX_BUFFER
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	//tx_sym_buffer_index = 0;
	is_half_buffer_ready = 1;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

// adc:		  writes to first half of ADC_BUFFER
// tx_isr:    reads from second half of TX_BUFFER
// main_loop: reads from second half of ADC_BUFFER
//			  writes to first half of TX_BUFFER
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	is_full_buffer_ready = 1;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	HAL_GPIO_WritePin(TX_SYM_FLAG_OUT_GPIO_Port, TX_SYM_FLAG_OUT_Pin, GPIO_PIN_SET);

	// b now is equivalent to GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
	const uint8_t b = TX_SYM_BUFFER[tx_sym_buffer_index];
	const uint8_t b_set = b << 4;
	const uint8_t b_reset = ~b << 4;
	const uint32_t v = ((uint32_t)b_reset << 16) | (uint32_t)(b_set);
	// 0xFFFF0000 resets pin, 0x0000FFFF sets pin
	GPIOB->BSRR = v;
	tx_sym_buffer_index = (tx_sym_buffer_index+1) % (TX_SYM_BUFFER_LENGTH*2*2);
	//tx_sym_buffer_index++;

	HAL_GPIO_WritePin(TX_SYM_FLAG_OUT_GPIO_Port, TX_SYM_FLAG_OUT_Pin, GPIO_PIN_RESET);
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
