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
#include <string.h>
#include <stdio.h>
#include "../../Adafruit_BNO055/IMU.h"
#include <stdbool.h>
#include "../../GPS/GPS.h"
#include "../../roboclaw/roboclaw.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//-------------------- MOTORs defines -------------------------
#define pulses_per_revolution_encoder 5281
#define pulses_per_revolution_gearbox 28
#define reduction_ratio 188


#define CPR 212000.0  //212000         // Quadrature-decoded counts per revolution of the motor shaft
#define GEAR_RATIO 168      // Gear reduction ratio 168:1
//-------------------------------------------------------------------------
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart4_rx;
DMA_HandleTypeDef hdma_usart4_tx;

/* USER CODE BEGIN PV */
//---------------------------  IMU Variables ------------------------------------
IMU_INFO imu;
uint8_t CAN_OrientationBuffer[8] = {0}; //0x200
uint8_t CAN_LinearAccelBuffer[8] = {0}; //0x201
uint8_t CAN_LinearVelBuffer[8] = {0};
//-----------------------------------------------------------------------------
//---------------------------  GPS Variables  --------------------------
GPS_INFO gps;
uint8_t CAN_LatBuffer[8] = {0};
uint8_t CAN_LongBuffer[8] = {0};
//------------------------------------------------------------------------------

//---------------------------  CAN Variables  --------------------------
volatile uint8_t messageReceived = 0;
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
uint8_t txData[] = {0x01, 0x02, 0xAB, 0xFF}; // Example data to send
uint32_t lastSendTime_IMU = 0; // Use a single timer for both messages
char* Debug_IMU1 ;
char* Debug_IMU2 ;
static uint16_t sequence_counter = 0; // The single counter for both messages
//------------------------------------------------------------------------------

//--------------------------- MOTOR Variables -------------------------------------
uint32_t encoderrrr;


SERIAL_HandleTypeDef* hserial_uart1;
SERIAL_HandleTypeDef* hserial_uart2;
SERIAL_HandleTypeDef* hserial_uart3;
RoboClaw_HandleTypeDef hroboclaw_mc1;
RoboClaw_HandleTypeDef hroboclaw_mc2;
RoboClaw_HandleTypeDef hroboclaw_mc3;

uint8_t motorsSpeeds[8];


ROBOCLAW_StatusTypeDef statusm1;

uint32_t m1_enc_cnt=0;

uint32_t m1a_speed=0;
uint32_t m1b_speed=0;
uint32_t m2a_speed=0;
uint32_t m2b_speed=0;
uint32_t m3a_speed=0;
uint32_t m3b_speed=0;

uint32_t prev_m1a_speed = 0;
uint32_t prev_m1b_speed = 0;
uint32_t prev_m2a_speed = 0;
uint32_t prev_m2b_speed = 0;
uint32_t prev_m3a_speed = 0;
uint32_t prev_m3b_speed = 0;
uint8_t data1=0;
uint8_t data3=0;

uint8_t status1a=0;
uint8_t status1b=0;
uint8_t status2a=0;
uint8_t status2b=0;
uint8_t status3a=0;
uint8_t status3b=0;
bool valid1a;
bool valid1b;
bool valid2a;
bool valid2b;
bool valid3a;
bool valid3b;

uint32_t debug_buff[100];
uint8_t SendMotorRPMs[6];


// Velocity Control Variables
uint8_t rx_byte[50];
float rpm;
uint8_t status;
bool valid;
uint32_t qpps;

//---------------------------------------------------------------------------------

//----------------------- GPS Configuration Variables -------------------------

//-----------------------------------------------------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void can_irq(CAN_HandleTypeDef *pcan);
HAL_StatusTypeDef can_send(uint32_t id, uint8_t data[], uint8_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//---------------------------  CAN Functions  --------------------------
void can_irq(CAN_HandleTypeDef *pcan) {
  /* Simply receive whatever message comes in */
  if (HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    messageReceived = 1;
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
//----------------------------------------------------------------------

//---------------------------  GPS Configuration Functions  --------------------------

//------------------------------------------------------------------------------------

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//
//	SERIAL_HandleTypeDef* serial_handler = get_serial_handler(huart);
//
//	ring_buffer_queue_arr(serial_handler->buffer_Rx, serial_handler->hal_reg_Rx, PRIMARY_REG_SIZE);
//	HAL_UART_Receive_DMA(huart, serial_handler->hal_reg_Rx, PRIMARY_REG_SIZE);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  // ---------------------Initialize GPS-----------------------
  	  /*
  	   *
  	   * UART 1 -> GPS
  	   *
  	   * */
  //-----------------------------------------------------------

  // ---------------------Initialize IMU-----------------------
  IMU_Setup(&hi2c1);
  //-----------------------------------------------------------


  /*-------------------- Initialize MOTORS (RoboClaws) --------------------------*/

  	/*
  	 * UART 4 -> MOToR 1
  	 *
  	 * UART 2 -> MOTOR 2
  	 *
  	 * UART 3 -> MOTOR 3
  	 *
  	 * */

// MOTOR 1&2 -> ROBOCLAW 1
//	hserial_uart1 = serial_init(&huart4);
//	hroboclaw_mc1.hserial = hserial_uart1;
//	hroboclaw_mc1.packetserial_address = 0x84;

////MOTOR 3&4 -> ROBOCLAW 2
//	 hserial_uart2 = serial_init(&huart2);
//	 hroboclaw_mc2.hserial = hserial_uart2;
//	 hroboclaw_mc2.packetserial_address = 0x80;
//
////MOTOR 5&6 -> ROBOCLAW 3
	 hserial_uart3 = serial_init(&huart2);
	 hroboclaw_mc3.hserial = hserial_uart3;
	 hroboclaw_mc3.packetserial_address = 0x80;

//-----------------------------------------------------------------------
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // LED off
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //------------------------  IMU Testing ------------------------------

	  IMU_Operation(&imu);
	      // Prepare both messages with the current sequence counter
	  IMU_CANMsgSendOrientation(&imu, CAN_OrientationBuffer, sequence_counter);
	  IMU_CANMsgSendLinearAccel(&imu, CAN_LinearAccelBuffer, sequence_counter);

	      // Send both messages if enough time has passed
	      if (HAL_GetTick() - lastSendTime_IMU > 1000) {
	          // Wait until at least 2 mailboxes are free
	          if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) >= 2) {
	              // Send Orientation first
	              if (can_send(0x200, CAN_OrientationBuffer, sizeof(CAN_OrientationBuffer)) == HAL_OK) {
	                  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
	                  Debug_IMU1 = "Can Message Sent";
	              } else {
	                  Debug_IMU1 = "Failed To Send";
	              }

	              // Send Linear Acceleration immediately after
	              if (can_send(0x201, CAN_LinearAccelBuffer, sizeof(CAN_LinearAccelBuffer)) == HAL_OK) {
	                  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
	                  Debug_IMU2 = "Can Message Sent";
	              } else {
	                  Debug_IMU2 = "Failed To Send";
	              }

	              // Increment the counter and reset the timer only after both messages have been sent
	              sequence_counter++;
	              lastSendTime_IMU = HAL_GetTick();
	          }
	      }
	  HAL_Delay(10);

//	  if (HAL_GetTick() - lastSendTime_IMU1 > 1000) {
//	    if (can_send(0x123, txData, sizeof(txData)) == HAL_OK) {
//	      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); // Toggle LED on successful send
//	    }
//	    lastSendTime_IMU1 = HAL_GetTick();
//	  }

	  //--------------------------------------------------------------------

//	  // -----------------------  GPS Testing  ----------------------------
//	  GPS_Operation(&huart1, &gps);
//	  GPS_CANMsgSendLatitude(&gps, CAN_LatBuffer);
//	  GPS_CANMsgSendLongitude(&gps, CAN_LongBuffer);
//	  //--------------------------------------------------------------------

	  //---------------------- MOTORS Testing (Velocity Control) ----------------------
	  	  //to read the speed we use the following functions

	  	  /*
	  	   * -> RoboClaw_ReadSpeedM1 Return the speed in QPPS so we need to use the following
	  	   * formula to calculate the RPM or speed
	  	   *
	  	   * */

//	  	 ForwardBackwardM1(&hroboclaw_mc1, 108);
//	  	SpeedM1(&hroboclaw_mc3, 12000);
//	  	qpps = ReadSpeedM1(&hroboclaw_mc3, &status, (bool *)&valid);
//
//	  	if (valid) {
//	 	  rpm = (qpps / CPR) * 60.0;
//	  	} else {
//	 	  rpm = 0.0f;
//	  	}
//
//	  	  HAL_Delay(10);


//      // Poll for a received byte
//      if (HAL_UART_Receive_DMA(&huart3, rx_byte, sizeof(rx_byte)) == HAL_OK) {
//          // Echo the received byte back
//          HAL_UART_Transmit_DMA(&huart3, rx_byte,sizeof(rx_byte));
      //}
//      // Optional: small delay to avoid spamming
//      HAL_Delay(10);
//    HAL_UART_Transmit(&huart4, (uint8_t *)"ADHAM", 5, 1000);
	  //----------------------------------------------------------------------------------


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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
    if (HAL_CAN_ConfigFilter(&hcan, &sf) != HAL_OK) { Error_Handler(); }

    if (HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, can_irq) != HAL_OK) { Error_Handler(); }

    if (HAL_CAN_Start(&hcan) != HAL_OK) { Error_Handler(); }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) { Error_Handler(); }

    /* Enable interrupts for CAN: FIFO0 message pending, transmit mailbox empty, error */
   __HAL_CAN_ENABLE_IT(&hcan,
                       CAN_IT_RX_FIFO0_MSG_PENDING | // RX FIFO0 message
                       CAN_IT_TX_MAILBOX_EMPTY    | // Transmit mailbox empty
                       CAN_IT_ERROR_WARNING       | // Error warning
                       CAN_IT_ERROR_PASSIVE       | // Error passive
                       CAN_IT_BUSOFF              | // Bus-off
                       CAN_IT_LAST_ERROR_CODE     | // Last error code
                       CAN_IT_ERROR               ); // General error

   /* Configure NVIC for CAN interrupts */
   HAL_NVIC_SetPriority(CEC_CAN_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.Timing = 0x00201D2B;
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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
