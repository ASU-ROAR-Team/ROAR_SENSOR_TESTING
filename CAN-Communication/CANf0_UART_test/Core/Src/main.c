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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// --- Original, Unchanged CAN Variables ---
volatile uint8_t messageReceived = 0;
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];

// --- New UART Gateway Specific Typedefs ---
// A simple structure to hold a CAN message for UART framing
typedef struct
{
  uint16_t id;
  uint8_t dlc;
  uint8_t data[8];
} UART_CanMessage_t;

// State machine for receiving UART messages
typedef enum
{
  UART_STATE_WAITING_FOR_START,
  UART_STATE_READING_ID_HIGH,
  UART_STATE_READING_ID_LOW,
  UART_STATE_READING_DLC,
  UART_STATE_READING_DATA,
  UART_STATE_READING_CHECKSUM,
  UART_STATE_READING_END
} UART_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- New UART Gateway Defines ---
const uint8_t UART_START_BYTE = 0xAA;
const uint8_t UART_END_BYTE = 0x55;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- New UART Gateway Specific Variables ---
UART_State_t uart_currentState = UART_STATE_WAITING_FOR_START;
UART_CanMessage_t uart_receivedMsg;
uint8_t uart_incomingByte;
int uart_dataIndex = 0;
uint8_t uart_calculatedChecksum = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
// --- Original, Unchanged CAN Function Prototypes ---
void can_irq(CAN_HandleTypeDef *pcan);
HAL_StatusTypeDef can_send(uint32_t id, uint8_t data[], uint8_t length);

// --- New UART Gateway Function Prototypes ---
void gateway_send_can_over_uart(uint32_t id, uint8_t* data, uint8_t dlc);
void gateway_process_uart_byte(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// --- Original, Unchanged CAN Functions ---
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

// --- New UART Gateway Functions ---

/**
 * @brief  Takes CAN data and sends it over UART with framing and checksum.
 * @param  id: The CAN message ID.
 * @param  data: Pointer to the CAN data payload.
 * @param  dlc: The data length code of the CAN message.
 * @retval None
 */
void gateway_send_can_over_uart(uint32_t id, uint8_t* data, uint8_t dlc)
{
  uint8_t tx_buffer[14]; // Max size: 1 start + 2 id + 1 dlc + 8 data + 1 checksum + 1 end
  int idx = 0;

  // Calculate the checksum
  uint8_t checksum = 0;
  checksum ^= (id >> 8);
  checksum ^= (id & 0xFF);
  checksum ^= dlc;
  for (int i = 0; i < dlc; ++i)
  {
    checksum ^= data[i];
  }

  // Populate the transmission buffer
  tx_buffer[idx++] = UART_START_BYTE;
  tx_buffer[idx++] = (uint8_t)(id >> 8);
  tx_buffer[idx++] = (uint8_t)(id & 0xFF);
  tx_buffer[idx++] = dlc;
  memcpy(&tx_buffer[idx], data, dlc);
  idx += dlc;
  tx_buffer[idx++] = checksum;
  tx_buffer[idx++] = UART_END_BYTE;

  // Transmit the message
  HAL_UART_Transmit(&huart2, tx_buffer, idx, 100);
}


/**
  * @brief  Processes a single byte received from UART using a state machine.
  * @note   This function is called from the UART RX interrupt callback.
  * When a full, valid message is parsed, it calls can_send().
  * @retval None
  */
void gateway_process_uart_byte(void)
{
    switch (uart_currentState)
    {
    case UART_STATE_WAITING_FOR_START:
      if (uart_incomingByte == UART_START_BYTE)
      {
        uart_currentState = UART_STATE_READING_ID_HIGH;
        uart_calculatedChecksum = 0;
        uart_dataIndex = 0;
        memset(&uart_receivedMsg, 0, sizeof(uart_receivedMsg));
      }
      break;

    case UART_STATE_READING_ID_HIGH:
      uart_receivedMsg.id = (uint16_t)uart_incomingByte << 8;
      uart_calculatedChecksum ^= uart_incomingByte;
      uart_currentState = UART_STATE_READING_ID_LOW;
      break;

    case UART_STATE_READING_ID_LOW:
      uart_receivedMsg.id |= uart_incomingByte;
      uart_calculatedChecksum ^= uart_incomingByte;
      uart_currentState = UART_STATE_READING_DLC;
      break;

    case UART_STATE_READING_DLC:
      uart_receivedMsg.dlc = uart_incomingByte;
      uart_calculatedChecksum ^= uart_incomingByte;
      if (uart_receivedMsg.dlc <= 8)
      {
        uart_currentState = (uart_receivedMsg.dlc == 0) ? UART_STATE_READING_CHECKSUM : UART_STATE_READING_DATA;
      }
      else
      { // Invalid DLC, reset
        uart_currentState = UART_STATE_WAITING_FOR_START;
      }
      break;

    case UART_STATE_READING_DATA:
      uart_receivedMsg.data[uart_dataIndex] = uart_incomingByte;
      uart_calculatedChecksum ^= uart_incomingByte;
      uart_dataIndex++;
      if (uart_dataIndex == uart_receivedMsg.dlc)
      {
        uart_currentState = UART_STATE_READING_CHECKSUM;
      }
      break;

    case UART_STATE_READING_CHECKSUM:
      if (uart_incomingByte == uart_calculatedChecksum)
      { // Checksum is valid
        uart_currentState = UART_STATE_READING_END;
      }
      else
      { // Checksum mismatch, reset
        uart_currentState = UART_STATE_WAITING_FOR_START;
      }
      break;

    case UART_STATE_READING_END:
      if (uart_incomingByte == UART_END_BYTE)
      {
        // Message successfully received and validated!
        // Now, transmit this message onto the CAN bus using the original function.
        can_send(uart_receivedMsg.id, uart_receivedMsg.data, uart_receivedMsg.dlc);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); // Toggle LED to indicate UART->CAN activity
      }
      // Reset state machine for the next message
      uart_currentState = UART_STATE_WAITING_FOR_START;
      break;
    }
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Process the received byte
        gateway_process_uart_byte();
        // Re-arm the interrupt for the next byte
        HAL_UART_Receive_IT(&huart2, &uart_incomingByte, 1);
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
  MX_CAN_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // LED off

  // Start UART reception in interrupt mode
  HAL_UART_Receive_IT(&huart2, &uart_incomingByte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* If a CAN message was received (flag set in can_irq), process it. */
    if (messageReceived) {
      // 1. Forward the received CAN message over UART
      gateway_send_can_over_uart(rxHeader.StdId, rxData, rxHeader.DLC);

      // 2. Toggle LED to indicate CAN->UART activity
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);

      // 3. Clear the flag (original behavior)
      messageReceived = 0;
    }

    HAL_Delay(50);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
