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
uint8_t rxData[8]={0};
uint32_t lastSendTime = 0;
SERIAL_HandleTypeDef *hserial_uart1;
SERIAL_HandleTypeDef *hserial_uart2;
SERIAL_HandleTypeDef *hserial_uart3;
RoboClaw_HandleTypeDef hroboclaw_mc1;
RoboClaw_HandleTypeDef hroboclaw_mc2;
RoboClaw_HandleTypeDef hroboclaw_mc3;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define gear_ratio 188 // change value
//#define pulses_per_revolution_gearbox 28 // change value
//#define reduction_ratio 188 // change value
//#define pi 3.14159265358979
#define end_eff_PWM   TIM_CHANNEL_2 //Extra 3
#define end_eff_Tim   &htim3

#define Storage1_PWM  TIM_CHANNEL_1 //Extra 1
#define Storage1_Tim  &htim14

#define Storage2_PWM  TIM_CHANNEL_1 //Extra 2
#define Storage2_Tim  &htim2

#define STORAGE_OPEN_ANGLE 	90 // adjust with Alaa
#define STORAGE_CLOSED_ANGLE 0

#define angles_id 				0x004
#define CAN_ID_ENCODER_FEEDBACK 0x222

#define M1 1	// will be used later in GoToAngle() function
#define M2 2

#define JOINT2 1
#define JOINT5 2
#define NORMAL 3
// Each motor's timer + channel
typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
} MotorConfig;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart4_rx;
DMA_HandleTypeDef hdma_usart4_tx;

/* USER CODE BEGIN PV */

/********** Values for SpeedAccelDeccelPositionM() function ************/
/* you can modify accel deccel and speed as you want to reach the most suitable values
 * check with علاء if the speed of the motor while rotating is cool or not either slowing it down or make it faster
 */
uint32_t accel = 1000;
uint32_t speed = 2000;
uint32_t deccel = 1000;
uint8_t flag = 0;
int anglee[1] = {0};


/************************************************************************/

const int32_t GoBilda_PPR = 5290; //PPR
const int32_t JOINT5_PPR = 7160; //PPR adjust for joint 5
const int32_t GEAR_RATIO = 4;
const int32_t max_angle = 210; // adjust with mechanical team
int32_t JOINT1_PPR = (GoBilda_PPR * GEAR_RATIO)*(max_angle/360); // map from 0 to 210 //ppr = 12931

/************************************************************************/
int32_t encoderM1; //global variable stores encoder ticks while debugging
int32_t encoderM2; //global variable stores encoder ticks while debugging
/************************************************************************/
uint8_t uart_enc_data[8] = { 0 };
uint64_t packed = 0; 	// a variable to store the 64 bit received by CAN
uint16_t joint[6] = { 0 }; // contains 6 angles received in each 10 bits from 0 to 1023
float angle_deg[6] = { 0 };	// contains actual angles in degrees after mapping to 0 to 360 degree
uint8_t storage1 = 0, storage2 = 0; //for servo 1 and servo 2

MotorConfig servos[3] = { { end_eff_Tim, end_eff_PWM }, { Storage1_Tim,
Storage1_PWM }, { Storage2_Tim, Storage2_PWM } };

volatile bool g_limitSwitchPressed = false; // A global flag to disable the interrupt after triggered first time

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */
void can_irq(CAN_HandleTypeDef *pcan);

HAL_StatusTypeDef can_send(uint32_t id, uint8_t data[], uint8_t length);

void GoToAngle(float angle, RoboClaw_HandleTypeDef *hroboClaw, uint8_t MOTOR,
		uint8_t MODE);

void processArmControlData(uint8_t *data); //main control function

void setServoAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t angle);

uint16_t angleToPulseValue(uint16_t angle);

void SendEncoderFeedback(uint8_t *data); //TO DO

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

void GoToAngle(float angle, RoboClaw_HandleTypeDef *hroboClaw, uint8_t MOTOR,
		uint8_t MODE) {
	uint8_t status;
	bool valid;
	int64_t no_of_ticks = 0;
	// different PPR case motors -> joint 2 and joint 5
	switch(MODE){

	case JOINT2 :
		no_of_ticks = (angle / 360) * JOINT1_PPR; //map the angle (0->360) to ticks (0->7000)

	break;
	case JOINT5:
		no_of_ticks = (angle / 360) * JOINT5_PPR; //map the angle (0->360) to ticks (0->7000)

	break;
	case NORMAL:
		no_of_ticks = (angle / 360) * GoBilda_PPR; //map the angle (0->360) to ticks (0->5290)

	break;
}


	switch (MOTOR) {
	case M1:
		encoderM1 = ReadEncM1(hroboClaw, &status, &valid);
		SpeedAccelDeccelPositionM1(hroboClaw, accel, speed, deccel, no_of_ticks,
				flag);
		break;
	case M2:
		encoderM2 = ReadEncM2(hroboClaw, &status, &valid);
		SpeedAccelDeccelPositionM2(hroboClaw, accel, speed, deccel, no_of_ticks,
				flag);
		break;
	}
	return;

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
		// Now you have joint[0]..joint[5] as raw values (0–1023) << ركززز محتاجين نحولها لزاوية
		angle_deg[j] = ((float) joint[j] / 1023.0f) * 360.0f; // map to 0–360°
	}
	storage1 = (packed >> 61) & 0x1; //if 0 closed if 1 open
	storage2 = (packed >> 62) & 0x1; //same

	// ---- Control DC Motors ----
	// joints[0] & [1] -> RoboClaw1 //Arm base
	GoToAngle(angle_deg[0], &hroboclaw_mc3, M1,NORMAL); // adjust each motor with each joint // joint1
	GoToAngle(angle_deg[1], &hroboclaw_mc3, M2,JOINT2); // joint 2

	// joints[2] & [3] -> RoboClaw2 //Arm Middle
	GoToAngle(angle_deg[2], &hroboclaw_mc2, M1,NORMAL); // joint 3
	GoToAngle(angle_deg[3], &hroboclaw_mc2, M2,NORMAL); // joint 4

	// joint[4] -> RoboClaw3	//Arm end + end effector
	GoToAngle(angle_deg[4], &hroboclaw_mc1, M1,JOINT5); // joint 5
	setServoAngle(end_eff_Tim, end_eff_PWM, angle_deg[5]); // end effector move
	// ----------------------------

	/*
	 * storage control
	 */
	// ---- Control Storage Servos ----
	/* commented عشان السيرفو متكسرش الغطيان
	if (storage1 == 1) {
		setServoAngle(Storage1_Tim, Storage1_PWM, STORAGE_OPEN_ANGLE);
	} else {
		setServoAngle(Storage2_Tim, Storage1_PWM, STORAGE_CLOSED_ANGLE);
	}

	if (storage2 == 1) {
		setServoAngle(Storage1_Tim, Storage2_PWM, STORAGE_OPEN_ANGLE);
	} else {
		setServoAngle(Storage2_Tim, Storage2_PWM, STORAGE_CLOSED_ANGLE);
	}
	*/

	return;
}

///////////////////ADHAM func xxxxx Chatgpt func vvvv  ظبطها ياعم أدهم مجربتش السيرفو قبل كدة
/**
 * @brief Maps a desired angle (0–360) to the corresponding PWM pulse count.
 * Works with TIM3 configured: Prescaler=0, Period=65535.
 * Assumes APB1 timer clock = 48 MHz → tick = 20.8 ns.
 * For standard hobby servos (50 Hz):
 * - 0° → 1 ms pulse
 * - 180° → 2 ms pulse
 * Extended here to allow mapping up to 360°
 */

uint16_t angleToPulseValue(uint16_t angle) {
	// Clamp to 0–360
	if (angle < 0)
		angle = 0;
	if (angle > 360)
		angle = 360;

	// Servo pulse range: 1 ms (min) to 2 ms (max)
	const uint16_t MIN_PULSE = 1000;  // 1 ms → 1000 ticks
	const uint16_t MAX_PULSE = 2000;  // 2 ms → 2000 ticks
	const uint16_t RANGE = MAX_PULSE - MIN_PULSE;

	// Map 0–360° → 1–2 ms range
	// Convert to timer counts
	uint16_t pulse = MIN_PULSE + ((uint32_t) angle * RANGE) / 360;

	return pulse;
}
/**
 * @brief  Sets the servo to a specific angle by updating the PWM compare register.
 * @param  htim A pointer to the TIM_HandleTypeDef structure for your timer.
 * @param  channel The timer channel used for the servo (e.g., TIM_CHANNEL_2).
 * @param  angle The desired servo angle in degrees (0 to 180).
 */
void setServoAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t angle) {
	uint16_t pulse = angleToPulseValue(angle);

	// Update the Capture Compare Register (CCR) for the specified channel.
	__HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

//NOT TESTED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!<========== VERY IMPORTANT
void SendEncoderFeedback(uint8_t *buffer) {
	uint8_t status1a, status1b;
	bool valid1a, valid1b;

	// --- Angle 0 Processing ---
	int32_t enc0 = ReadEncM1(&hroboclaw_mc2, &status1a, &valid1a);
	// Calculate angle as a float first for precision
	float actual_angle0_f = ((float) enc0 / GoBilda_PPR) * 360.0f;
	// Normalize angle to be within 0-360 range
	actual_angle0_f = fmod(actual_angle0_f, 360.0f);
	if (actual_angle0_f < 0) {
		actual_angle0_f += 360.0f;
	}
	// Map the float angle to a 10-bit integer (0-1023)
	uint16_t ten_bit_angle0 = (uint16_t) roundf(
			(actual_angle0_f / 360.0f) * 1023.0f);

	// --- Angle 1 Processing ---
	int32_t enc1 = ReadEncM2(&hroboclaw_mc2, &status1b, &valid1b);
	float actual_angle1_f = ((float) enc1 / GoBilda_PPR) * 360.0f;
	// Normalize angle
	actual_angle1_f = fmod(actual_angle1_f, 360.0f);
	if (actual_angle1_f < 0) {
		actual_angle1_f += 360.0f;
	}
	// Map the float angle to a 10-bit integer (0-1023)
	uint16_t ten_bit_angle1 = (uint16_t) roundf(
			(actual_angle1_f / 360.0f) * 1023.0f);

	// --- Packing ---
	// Combine the two 10-bit values into one 20-bit number.
	uint32_t combined_angles = (ten_bit_angle1 << 10) | ten_bit_angle0;

	// Place the 3 bytes into your buffer to be sent via UART
	buffer[0] = combined_angles & 0xFF;       // Byte 0: Lower 8 bits of angle 0
	buffer[1] = (combined_angles >> 8) & 0xFF; // Byte 1: Upper 2 bits of angle 0 & lower 6 of angle 1
	buffer[2] = (combined_angles >> 16) & 0xFF; // Byte 2: Upper 4 bits of angle 1

	// buffer[3] through buffer[7] are available for other data
	// For example, you could pack another two angles into buffer[3], buffer[4], and buffer[5].
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
//		uint8_t txData[] = "FuckCAN"; // Example data to send
//		uint32_t lastSendTime = 0;
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
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_TIM14_Init();
	MX_ADC_Init();
	MX_USART4_UART_Init();
	/* USER CODE BEGIN 2 */

	////MOTOR 5 -> ROBOCLAW 1
	hserial_uart1 = serial_init(&huart4);
	hroboclaw_mc1.hserial = hserial_uart1;
	hroboclaw_mc1.packetserial_address = 0x80;

	////MOTOR 3&4 -> ROBOCLAW 2
	hserial_uart2 = serial_init(&huart2);
	hroboclaw_mc2.hserial = hserial_uart2;
	hroboclaw_mc2.packetserial_address = 0x82;

	////MOTOR 1&2 -> ROBOCLAW 3
//	hserial_uart3 = serial_init(&huart3);
//	hroboclaw_mc3.hserial = hserial_uart3;
//	hroboclaw_mc3.packetserial_address = 0x84;

	//Homing position
//	SpeedM1(&hroboclaw_mc2, 3000);	//Start
	//Motor rotates continuously until it hits the limit switch and triggers the interrupt

	// Start all PWM channels
	for (int i = 0; i < 3; i++) {
		HAL_TIM_PWM_Start(servos[i].htim, servos[i].channel);
	}
//
//	// Set initial position to center (90 degrees)
//	setServoAngle(&htim2, TIM_CHANNEL_3, 90);
//	HAL_Delay(1000); // Give servo time to reach initial position

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/******* Test code ********/
//		if(HAL_UART_Receive(&huart3, &anglee, sizeof(anglee) , HAL_MAX_DELAY ) == HAL_OK) {
			//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
			uint8_t status;
			bool valid;
			ResetEncoders(&hroboclaw_mc2);
			GoToAngle(180, &hroboclaw_mc2,M2,NORMAL);
	//		HAL_Delay(1000);
			GoToAngle(90, &hroboclaw_mc2,M2,NORMAL);
	//		HAL_Delay(1000);
			encoderM1 = ReadEncM2(&hroboclaw_mc2, &status, &valid);
			HAL_Delay(1000);

//		 }
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// CAN message from High level
//		if (messageReceived) {
//			messageReceived = 0;		// Clear flag
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
//			processArmControlData(rxData);//this function decodes the CAN message into actual angles
//
//		}
//		SendEncoderFeedback(uart_enc_data);
////			HAL_UART_Transmit_DMA(&huart2, uart_enc_data, sizeof(uart_enc_data));
//		/* Send message every 1000ms */
//		//redefine the txData to send encoder values
//		if (HAL_GetTick() - lastSendTime > 1000) {
//			if (can_send(CAN_ID_ENCODER_FEEDBACK, uart_enc_data,
//					sizeof(uart_enc_data)) == HAL_OK) {
//			}
//			lastSendTime = HAL_GetTick();
//		}
//		HAL_Delay(500);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
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
static void MX_CAN_Init(void) {

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
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */
	CAN_FilterTypeDef sf;
	  // Set the filter to only accept messages with ID 0x6A5
	  sf.FilterIdHigh = 0x004 << 5; // Left-shift to align with the 11-bit identifier field
	  sf.FilterIdLow = 0x0000;
	  // A mask of 0xFFE0 will check all 11 bits of the standard ID
	  sf.FilterMaskIdHigh = 0xFFE0;
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 7;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 19999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 1;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 19999;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */
	HAL_TIM_MspPostInit(&htim14);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART4_UART_Init(void) {

	/* USER CODE BEGIN USART4_Init 0 */

	/* USER CODE END USART4_Init 0 */

	/* USER CODE BEGIN USART4_Init 1 */

	/* USER CODE END USART4_Init 1 */
	huart4.Instance = USART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART4_Init 2 */

	/* USER CODE END USART4_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

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
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

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
	ForwardM1(hroboClaw, 0); // Stop motor
	HAL_Delay(50);
	ResetEncoders(hroboClaw);
	HAL_Delay(50);

	// Optionally move off the switch
//	GoToAngle(2, hroboClaw);
//	HAL_Delay(500);
	encoderM1 = ReadEncM1(hroboClaw, &status, &valid);

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
