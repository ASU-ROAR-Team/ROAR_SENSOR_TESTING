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
#include "../../roboclaw/roboclaw.h"
//#include "../../pH/pH.h"
#include "string.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint8_t messageReceived = 0;
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];

SERIAL_HandleTypeDef *hserial_uart1;
SERIAL_HandleTypeDef *hserial_uart2;
SERIAL_HandleTypeDef *hserial_uart3;
RoboClaw_HandleTypeDef hroboclaw_mc1;
RoboClaw_HandleTypeDef hroboclaw_mc2;
RoboClaw_HandleTypeDef hroboclaw_mc3;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define gear_ratio 188 // change value
#define pulses_per_revolution_gearbox 28 // change value
#define reduction_ratio 188 // change value
#define pi 3.14159265358979
#define angles_id 0x004
#define CAN_ID_ENCODER_FEEDBACK 0x005


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t accel = 1000;
uint32_t speed = 2000;
uint32_t deccel = 1000;
volatile uint32_t position[5] = { 0, 0, 0, 0, 0 };
uint8_t input_min = 0;
uint8_t input_max = 255;
int32_t output_min = 0;
int32_t output_max = 5290;
int32_t encoder;
uint8_t flag = 0;
uint16_t angle_degree = 0;
uint8_t m1_speed_buffer[2];
uint32_t debug_buff[100];
uint8_t SendMotorRPMs[6];

uint64_t packed = 0;
uint16_t joint[6];
float angle_deg[6];
uint8_t storage1=0,storage2=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

volatile bool g_limitSwitchPressed = false; // A global flag

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void can_irq(CAN_HandleTypeDef *pcan);
HAL_StatusTypeDef can_send(uint32_t id, uint8_t data[], uint8_t length);

/* this function used to map the recieved message from jetson in form of 0:255 to 0:11000 to be suitable for the roboclaw input*/
uint32_t map_value(uint8_t value, uint8_t input_min, uint8_t input_max,
		int32_t output_min, int32_t output_max);

int32_t GoToAngle(float angle, RoboClaw_HandleTypeDef *hroboClaw);

void processArmControlData(uint8_t *data);

void setServoAngle(TIM_HandleTypeDef* htim, uint32_t channel, uint16_t angle);
uint16_t angleToPulseValue(uint16_t angle);

void SendEncoderFeedback(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void can_irq(CAN_HandleTypeDef *pcan) {
	/* Simply receive whatever message comes in */
	if (HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
		messageReceived = 1;
		// Check if the message is for Arm Control
//		if (rxHeader.StdId == angles_id) {
//			messageReceived = 1;
//		}
	}
}

HAL_StatusTypeDef can_send(uint32_t id, uint8_t data[], uint8_t length) {
	CAN_TxHeaderTypeDef txHeader;
	uint32_t mailbox;

	/* Configure TX header */
	txHeader.StdId = id;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = length;
	txHeader.TransmitGlobalTime = DISABLE;

	/* Check if mailbox is available and send */
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
		return HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &mailbox);
	}
	return HAL_BUSY;
}

/* this function used to map the recieved message from jetson in form of 0:255 to 0:11000 to be suitable for the roboclaw input*/
uint32_t map_value(uint8_t value, uint8_t input_min, uint8_t input_max,
		int32_t output_min, int32_t output_max) {

	// Normalize the input value
	uint32_t normalized_value = (float) (value - input_min)
			/ (input_max - input_min);

	// Map the normalized value to the output range
	uint32_t mapped_value = normalized_value * (output_max - output_min)
			+ output_min;

	return mapped_value;
}

int32_t GoToAngle(float angle, RoboClaw_HandleTypeDef *hroboClaw) {
	uint8_t status;
	bool valid;

	encoder = ReadEncM2(hroboClaw, &status, &valid);
	float no_of_ticks = (angle / 360) * output_max;
	SpeedAccelDeccelPositionM2(hroboClaw, accel, speed, deccel, no_of_ticks,
			flag);

	return ReadEncM2(hroboClaw, &status, &valid);
}

void processArmControlData(uint8_t *data) {
	packed = 0;
	// Combine all 8 bytes into a 64-bit integer

	for (int i = 0; i < 8; i++) {
		packed |= ((uint64_t) data[i]) << (8 * i);
	}

	// Extract each 10-bit joint value
	for (int j = 0; j < 6; j++) {
		joint[j] = (packed >> (j * 10)) & 0x3FF; // 0x3FF = 10-bit mask
		// Now you have joint[0]..joint[5] as raw values (0–1023)
		angle_deg[j] = ((float) joint[j] / 1023.0f) * 360.0f; // Example: map to 0–360°
	}
	storage1 = (packed >> 61) & 0x1 ;
	storage2 = (packed >> 62) & 0x1 ;
	GoToAngle(angle_deg[0],&hroboclaw_mc2);




	return;
}

void SendEncoderFeedback(void) {
//    uint8_t status;
//    bool valid;
//    uint8_t feedback_data[8];

    // Read encoder values
//    int32_t enc0 = ReadEncM1(&hroboclaw_mc2, &status, &valid);//in ticks
//    uint16_t actual_angle0 = ((enc0)/output_max)*360;// in degrees
//    int32_t enc1 = ReadEncM2(&hroboclaw_mc2, &status, &valid);
//    int32_t enc2 = ReadEncM1(&hroboclaw_mc2, &status, &valid);
//    int32_t enc3 = ReadEncM1(&hroboclaw_mc2, &status, &valid);
//    int32_t enc4 = ReadEncM1(&hroboclaw_mc2, &status, &valid);
//    int32_t enc5 = ReadEncM1(&hroboclaw_mc2, &status, &valid);

    // For this example, we'll just pack the first 4.
    // The CAN standard frame only has 8 bytes. You may need multiple CAN messages
    // to send all 6 encoder values if you need full 32-bit resolution.
    // Here, we'll send them as 16-bit values to fit.
//    feedback_data[0] = (enc0 >> 8) & 0xFF; // Joint 0 MSB
//    feedback_data[1] = enc0 & 0xFF;        // Joint 0 LSB
//    feedback_data[2] = (enc1 >> 8) & 0xFF; // Joint 1 MSB
//    feedback_data[3] = enc1 & 0xFF;        // Joint 1 LSB
//    feedback_data[4] = (enc2 >> 8) & 0xFF; // Joint 2 MSB
//    feedback_data[5] = enc2 & 0xFF;        // Joint 2 LSB

//    can_send(CAN_ID_ENCODER_FEEDBACK, feedback_data, 8);
}


/**
 * @brief  Maps a desired angle (0-180) to the corresponding PWM pulse count.
 * @param  angle The desired angle in degrees (0 to 180).
 * @retval The calculated pulse value for the timer's compare register.
 */
uint16_t angleToPulseValue(uint16_t angle)
{
    // Clamp the angle to ensure it stays within 0-180 degrees
    if (angle > 180) {
        angle = 180;
    }

    // These values are correct for a 50Hz PWM with a Period of 9999 (10000 total counts)
    // which corresponds to a 20ms period.
    // 0.5ms pulse (0 deg)  -> (0.5ms / 20ms) * 10000 = 250
    // 2.5ms pulse (180 deg) -> (2.5ms / 20ms) * 10000 = 1250
    const uint16_t MIN_PULSE_COUNTS = 250;
    const uint16_t MAX_PULSE_COUNTS = 1250;
    const uint16_t PULSE_RANGE_COUNTS = MAX_PULSE_COUNTS - MIN_PULSE_COUNTS; // 1000 counts

    // Linear mapping formula:
    // pulse_counts = MIN_PULSE_COUNTS + (angle * PULSE_RANGE_COUNTS) / 180
    uint16_t pulse = MIN_PULSE_COUNTS + ((uint32_t)angle * PULSE_RANGE_COUNTS) / 180;

    return pulse;
}

/**
 * @brief  Sets the servo to a specific angle by updating the PWM compare register.
 * @param  htim A pointer to the TIM_HandleTypeDef structure for your timer.
 * @param  channel The timer channel used for the servo (e.g., TIM_CHANNEL_2).
 * @param  angle The desired servo angle in degrees (0 to 180).
 */
void setServoAngle(TIM_HandleTypeDef* htim, uint32_t channel, uint16_t angle)
{
    uint16_t pulse = angleToPulseValue(angle);

    // Update the Capture Compare Register (CCR) for the specified channel.
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
		uint8_t txData[] = "FuckCAN"; // Example data to send
		uint32_t lastSendTime = 0;
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	// MOTOR 1&2 -> ROBOCLAW 1
	hserial_uart1 = serial_init(&huart1);
	hroboclaw_mc1.hserial = hserial_uart1;
	hroboclaw_mc1.packetserial_address = 0x80;

	////MOTOR 3&4 -> ROBOCLAW 2
	hserial_uart2 = serial_init(&huart2);
	hroboclaw_mc2.hserial = hserial_uart2;
	hroboclaw_mc2.packetserial_address = 0x80;
	//
	////MOTOR 5&6 -> ROBOCLAW 3
	hserial_uart3 = serial_init(&huart3);
	hroboclaw_mc3.hserial = hserial_uart3;
	hroboclaw_mc3.packetserial_address = 0x80;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // LED off
	//Homing position
	SpeedM1(&hroboclaw_mc2, 12000);	//Start
	//Motor rotates continuously until it hits the limit switch and triggers the interrupt


//  	// Start the PWM signal on TIM2 Channel 2
//    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//
//	// Set initial position to center (90 degrees)
//	setServoAngle(&htim2, TIM_CHANNEL_3, 90);
//	HAL_Delay(1000); // Give servo time to reach initial position



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		uint8_t status;
		bool valid;

//		GoToAngle(180, &hroboclaw_mc2);
//		HAL_Delay(1000);
//		GoToAngle(90, &hroboclaw_mc2);
//		HAL_Delay(1000);
//		encoder = ReadEncM1(&hroboclaw_mc2, &status, &valid);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// CAN message from High level

		if (messageReceived) {
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
			messageReceived = 0;		// Clear flag
			processArmControlData(rxData);

		}

		/* Send message every 1000ms */

//		if (HAL_GetTick() - lastS
		/* If message received, toggle LED */


		HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	CAN_FilterTypeDef sf;
	sf.FilterMaskIdHigh = 0x0000;
	sf.FilterMaskIdLow = 0x0000;
	sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sf.FilterBank = 0;
	sf.FilterMode = CAN_FILTERMODE_IDMASK;
	sf.FilterScale = CAN_FILTERSCALE_32BIT;
	sf.FilterActivation = CAN_FILTER_ENABLE;
	if (HAL_CAN_ConfigFilter(&hcan, &sf) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
			can_irq) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Enable interrupts for CAN: FIFO0 message pending, transmit mailbox empty, error */
	__HAL_CAN_ENABLE_IT(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | // RX FIFO0 message
			CAN_IT_TX_MAILBOX_EMPTY |// Transmit mailbox empty
			CAN_IT_ERROR_WARNING |// Error warning
			CAN_IT_ERROR_PASSIVE |// Error passive
			CAN_IT_BUSOFF |// Bus-off
			CAN_IT_LAST_ERROR_CODE |// Last error code
			CAN_IT_ERROR);// General error

	/* Configure NVIC for CAN interrupts */
	HAL_NVIC_SetPriority(CEC_CAN_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
  /* USER CODE END CAN_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_5) {
		// The limit switch interrupt was triggered!
		// Set a flag. Keep the ISR (Interrupt Service Routine) short and fast.
		g_limitSwitchPressed = true;
	}
}

void HomingSequence_IT(RoboClaw_HandleTypeDef *hroboClaw) {
	uint8_t status;
	bool valid;
	// Reset the flag and start moving towards the switch
	g_limitSwitchPressed = false;
//    ForwardM1(hroboClaw, 20); // Adjust direction/speed as needed
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);

	// Wait for the flag to be set by the interrupt
	while (!g_limitSwitchPressed) {
		HAL_Delay(5);
	}

	// --- Switch has been triggered! ---
	ForwardM2(hroboClaw, 0); // Stop motor
	HAL_Delay(50);
	ResetEncoders(hroboClaw);
	HAL_Delay(50);

	// Optionally move off the switch
	GoToAngle(2, hroboClaw);
	HAL_Delay(500);
	encoder = ReadEncM2(hroboClaw, &status, &valid);

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
#ifdef USE_FULL_ASSERT
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
