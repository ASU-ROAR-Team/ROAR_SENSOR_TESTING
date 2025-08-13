/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Improved CAN-UART Gateway with message buffering
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* USER CODE BEGIN PTD */
// --- CAN-to-UART Message Buffer System ---
#define CAN_RX_BUFFER_SIZE 32  // Circular buffer for incoming CAN messages (power of 2)
#define UART_TX_BUFFER_SIZE 16 // <<< NEW >>> Circular buffer for outgoing UART frames (power of 2)

typedef struct
{
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} CanMessage_t;

// <<< NEW >>> Structure to hold a fully formatted UART frame
typedef struct
{
  uint8_t data[14]; // Max UART frame: 1(start)+2(id)+1(dlc)+8(data)+1(chk)+1(end) = 14
  uint8_t length;
} UartFrame_t;

// Circular buffer for incoming CAN messages
typedef struct
{
  CanMessage_t messages[CAN_RX_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint32_t drops;
} CanRxBuffer_t;

// <<< NEW >>> Circular buffer for outgoing UART frames
typedef struct
{
  UartFrame_t frames[UART_TX_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint32_t drops; // Count of UART frames dropped because TX buffer was full
} UartTxBuffer_t;


// UART transmission state
typedef enum
{
  UART_TX_IDLE,
  UART_TX_BUSY
} UART_TxState_t;

// UART Gateway state machine (unchanged)
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

// Simple structure for UART->CAN messages (unchanged)
typedef struct
{
  uint16_t id;
  uint8_t dlc;
  uint8_t data[8];
} UART_CanMessage_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
// --- Protocol Constants ---
const uint8_t UART_START_BYTE = 0xAA;
const uint8_t UART_END_BYTE = 0x55;

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- Message Buffers ---
static CanRxBuffer_t canRxBuffer = {0};
static UartTxBuffer_t uartTxBuffer = {0}; // <<< NEW >>>

// --- Diagnostic Counters ---
volatile uint32_t can_rx_total = 0;
volatile uint32_t uart_tx_queued = 0;     // <<< MODIFIED >>> Total UART frames successfully queued
volatile uint32_t uart_tx_sent = 0;       // <<< MODIFIED >>> Total UART frames actually sent
volatile uint32_t can_tx_total = 0;
volatile uint32_t can_tx_errors = 0;

// --- UART Transmission State ---
static volatile UART_TxState_t uart_tx_state = UART_TX_IDLE; // <<< MODIFIED >>> Made volatile
static uint8_t uart_tx_hal_buffer[14]; // <<< MODIFIED >>> Renamed buffer used by HAL_UART_Transmit_IT

// --- UART Reception State (unchanged) ---
static UART_State_t uart_currentState = UART_STATE_WAITING_FOR_START;
static UART_CanMessage_t uart_receivedMsg;
static uint8_t uart_incomingByte;
static int uart_dataIndex = 0;
static uint8_t uart_calculatedChecksum = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
// --- CAN RX Buffer Management ---
static uint8_t can_rx_buffer_put(uint32_t id, uint8_t* data, uint8_t dlc);
static uint8_t can_rx_buffer_get(CanMessage_t* msg);
static uint16_t can_rx_buffer_count(void);

// <<< NEW >>> --- UART TX Buffer Management ---
static uint8_t uart_tx_buffer_put(uint8_t* data, uint8_t length);
static uint8_t uart_tx_buffer_get(UartFrame_t* frame);
static uint8_t uart_tx_buffer_is_empty(void);

// --- CAN Functions ---
void can_irq(CAN_HandleTypeDef *pcan);
HAL_StatusTypeDef can_send(uint32_t id, uint8_t data[], uint8_t length);

// --- UART Gateway Functions ---
void gateway_queue_can_for_uart(uint32_t id, uint8_t* data, uint8_t dlc); // <<< MODIFIED >>> Renamed
void gateway_process_uart_byte(void);
void gateway_uart_tx_complete_callback(void);
void uart_tx_try_send(void); // <<< NEW >>>

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// --- CAN RX Buffer Management Functions ---
// (Unchanged, but renamed for clarity)
static uint8_t can_rx_buffer_put(uint32_t id, uint8_t* data, uint8_t dlc)
{
  uint16_t next_head = (canRxBuffer.head + 1) & (CAN_RX_BUFFER_SIZE - 1);
  if (next_head == canRxBuffer.tail) {
    canRxBuffer.drops++;
    return 0;
  }
  canRxBuffer.messages[canRxBuffer.head].id = id;
  canRxBuffer.messages[canRxBuffer.head].dlc = dlc;
  memcpy(canRxBuffer.messages[canRxBuffer.head].data, data, dlc);
  canRxBuffer.head = next_head;
  return 1;
}

static uint8_t can_rx_buffer_get(CanMessage_t* msg)
{
  if (canRxBuffer.head == canRxBuffer.tail) {
    return 0;
  }
  *msg = canRxBuffer.messages[canRxBuffer.tail];
  canRxBuffer.tail = (canRxBuffer.tail + 1) & (CAN_RX_BUFFER_SIZE - 1);
  return 1;
}

static uint16_t can_rx_buffer_count(void)
{
  return (canRxBuffer.head - canRxBuffer.tail) & (CAN_RX_BUFFER_SIZE - 1);
}


// <<< NEW >>> --- UART TX Buffer Management Functions ---

/**
 * @brief  Adds a pre-formatted UART frame to the TX circular buffer.
 * @param  data: Pointer to the frame data.
 * @param  length: Length of the frame data.
 * @retval 1 if successful, 0 if buffer full (frame dropped).
 */
static uint8_t uart_tx_buffer_put(uint8_t* data, uint8_t length)
{
    uint16_t next_head = (uartTxBuffer.head + 1) & (UART_TX_BUFFER_SIZE - 1);

    // Check if buffer is full
    if (next_head == uartTxBuffer.tail) {
        uartTxBuffer.drops++;
        return 0; // Buffer full, frame dropped
    }

    // Store frame
    uartTxBuffer.frames[uartTxBuffer.head].length = length;
    memcpy(uartTxBuffer.frames[uartTxBuffer.head].data, data, length);

    // Update head pointer
    uartTxBuffer.head = next_head;

    return 1; // Success
}

/**
 * @brief  Gets a UART frame from the TX circular buffer.
 * @param  frame: Pointer to store the retrieved frame.
 * @retval 1 if frame retrieved, 0 if buffer empty.
 */
static uint8_t uart_tx_buffer_get(UartFrame_t* frame)
{
    // Check if buffer is empty
    if (uartTxBuffer.head == uartTxBuffer.tail) {
        return 0; // Buffer empty
    }

    // Retrieve frame
    *frame = uartTxBuffer.frames[uartTxBuffer.tail];

    // Update tail pointer
    uartTxBuffer.tail = (uartTxBuffer.tail + 1) & (UART_TX_BUFFER_SIZE - 1);

    return 1; // Success
}

/**
 * @brief  Checks if the UART TX buffer is empty.
 * @retval 1 if empty, 0 if not empty.
 */
static uint8_t uart_tx_buffer_is_empty(void)
{
    return (uartTxBuffer.head == uartTxBuffer.tail);
}


// --- CAN Functions ---

void can_irq(CAN_HandleTypeDef *pcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  if (HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    can_rx_total++;
    // <<< MODIFIED >>> Store in canRxBuffer
    can_rx_buffer_put(rxHeader.StdId, rxData, rxHeader.DLC);
  }
}

HAL_StatusTypeDef can_send(uint32_t id, uint8_t data[], uint8_t length)
{
  // (Unchanged)
  CAN_TxHeaderTypeDef txHeader;
  uint32_t mailbox;
  HAL_StatusTypeDef status;
  txHeader.StdId = id;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = length;
  txHeader.TransmitGlobalTime = DISABLE;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
    status = HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &mailbox);
    if (status == HAL_OK) {
      can_tx_total++;
    } else {
      can_tx_errors++;
    }
    return status;
  }
  can_tx_errors++;
  return HAL_BUSY;
}

// --- UART Gateway Functions ---

/**
 * @brief  <<< MODIFIED >>> Builds a UART frame and queues it for transmission.
 * Also attempts to start transmission if UART is idle.
 * @param  id: CAN message ID
 * @param  data: Pointer to message data
 * @param  dlc: Data length code
 */
void gateway_queue_can_for_uart(uint32_t id, uint8_t* data, uint8_t dlc)
{
  uint8_t frame_buffer[14];
  int idx = 0;

  // Calculate checksum
  uint8_t checksum = 0;
  checksum ^= (id >> 8);
  checksum ^= (id & 0xFF);
  checksum ^= dlc;
  for (int i = 0; i < dlc; ++i) {
    checksum ^= data[i];
  }

  // Build frame
  frame_buffer[idx++] = UART_START_BYTE;
  frame_buffer[idx++] = (uint8_t)(id >> 8);
  frame_buffer[idx++] = (uint8_t)(id & 0xFF);
  frame_buffer[idx++] = dlc;
  memcpy(&frame_buffer[idx], data, dlc);
  idx += dlc;
  frame_buffer[idx++] = checksum;
  frame_buffer[idx++] = UART_END_BYTE;

  // Put the assembled frame into the UART TX buffer
  if (uart_tx_buffer_put(frame_buffer, idx)) {
      uart_tx_queued++;
  }

  // Try to send the next message from the queue
  uart_tx_try_send();
}

/**
 * @brief  <<< NEW >>> Checks queue and starts UART transmission if possible.
 * This function is safe to call from both main loop and interrupt context.
 */
void uart_tx_try_send(void)
{
    // --- Critical Section ---
    // Prevent race condition between main loop and TxCpltCallback interrupt.
    __disable_irq();

    if (uart_tx_state == UART_TX_IDLE && !uart_tx_buffer_is_empty())
    {
        // UART is free and there's a frame to send, so let's start
        uart_tx_state = UART_TX_BUSY;

        // We are in a critical section, so we can safely get the next frame
        UartFrame_t frame_to_send;
        uart_tx_buffer_get(&frame_to_send);

        __enable_irq(); // Exit critical section before HAL call

        // Copy to the buffer used by HAL and start non-blocking transmission
        memcpy(uart_tx_hal_buffer, frame_to_send.data, frame_to_send.length);
        if (HAL_UART_Transmit_IT(&huart2, uart_tx_hal_buffer, frame_to_send.length) != HAL_OK) {
            // If transmission fails to start, revert state and try again later
            uart_tx_state = UART_TX_IDLE;
        }
    }
    else
    {
        // UART is busy or queue is empty, nothing to do.
        __enable_irq(); // Exit critical section
    }
}


/**
 * @brief  UART transmission complete callback.
 */
void gateway_uart_tx_complete_callback(void)
{
  uart_tx_sent++;
  uart_tx_state = UART_TX_IDLE;

  // <<< MODIFIED >>> Transmission is complete, try to send the next frame in the queue.
  uart_tx_try_send();
}

/**
 * @brief  Process UART byte (unchanged from original)
 */
void gateway_process_uart_byte(void)
{
  // (Unchanged)
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
    {
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
    {
      uart_currentState = UART_STATE_READING_END;
    }
    else
    {
      uart_currentState = UART_STATE_WAITING_FOR_START;
    }
    break;

  case UART_STATE_READING_END:
    if (uart_incomingByte == UART_END_BYTE)
    {
      can_send(uart_receivedMsg.id, uart_receivedMsg.data, uart_receivedMsg.dlc);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    }
    uart_currentState = UART_STATE_WAITING_FOR_START;
    break;
  }
}

// --- HAL Callbacks ---

/**
 * @brief  UART RX complete callback (unchanged)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    gateway_process_uart_byte();
    HAL_UART_Receive_IT(&huart2, &uart_incomingByte, 1);
  }
}

/**
 * @brief  UART TX complete callback
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    gateway_uart_tx_complete_callback();
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();
  
  /* Initialize peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  
  // Initialize buffers
  memset(&canRxBuffer, 0, sizeof(canRxBuffer));
  memset(&uartTxBuffer, 0, sizeof(uartTxBuffer)); // <<< NEW >>>
  
  // Start UART reception
  HAL_UART_Receive_IT(&huart2, &uart_incomingByte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    // <<< MODIFIED >>> Main loop logic is now cleaner
    CanMessage_t msg;
    
    // Process all buffered CAN messages
    // This will pull messages from the CAN RX buffer and place them
    // into the UART TX buffer. The TX process is handled by interrupts.
    if (can_rx_buffer_get(&msg))
    {
      gateway_queue_can_for_uart(msg.id, msg.data, msg.dlc);
    }
    
    // The HAL_Delay from your original code is removed. The main loop
    // can now continuously pull from the CAN buffer and queue for UART
    // without blocking. If both buffers are empty, the loop will spin,
    // which is fine. You can add a __WFI() (Wait For Interrupt) instruction
    // here for power saving if desired.
    
    // e.g., if (can_rx_buffer_is_empty() && uart_tx_buffer_is_empty()) { __WFI(); }

    /* USER CODE END 3 */
  }
}

// ... (SystemClock_Config, MX_CAN_Init, MX_USART2_UART_Init, MX_GPIO_Init, Error_Handler, assert_failed remain unchanged)
// ... (Your existing peripheral initialization functions go here)


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