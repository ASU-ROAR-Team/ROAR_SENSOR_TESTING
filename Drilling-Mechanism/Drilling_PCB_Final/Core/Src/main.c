/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Position Control Final Logic
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
#include "../../PID/pid_controller.h"
#include "../../PID/motor_control.h"
#include "../../Servo/servo.h"
#include "../../Loadcell/loadcell.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f0xx_hal_tim.h"
#include <math.h> // For fabs()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/***********************************************UART CONFIGS************************************************/
// --- UART TX Buffer System (from Gateway Example) ---
#define UART_TX_BUFFER_SIZE 32 // Circular buffer for outgoing UART frames (power of 2)

// Structure to hold a fully formatted UART frame
typedef struct
{
  uint8_t data[20]; // Max UART frame size
  uint8_t length;
} UartFrame_t;

// Circular buffer for outgoing UART frames
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


// --- UART RX State Machine (from Gateway Example) ---
// This is included for future expansion if you need to receive commands.
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

// Simple structure for UART messages to be processed
typedef struct
{
  uint16_t id;
  uint8_t dlc;
  uint8_t data[8];
} UART_IncomingMessage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ENCODER_PPR 12000 // Counts Per Revolution
#define KP_POS 0.5f
#define KI_POS 0.001f
#define KD_POS 0.01f

// Servos
#define SERVO_1_TIMER     &htim2
#define SERVO_1_CHANNEL   TIM_CHANNEL_3

#define SERVO_2_TIMER     &htim2
#define SERVO_2_CHANNEL   TIM_CHANNEL_4

#define SERVO_3_TIMER     &htim2
#define SERVO_3_CHANNEL   TIM_CHANNEL_1

#define SERVO_4_TIMER     &htim16
#define SERVO_4_CHANNEL   TIM_CHANNEL_1

// Known weight in milligrams for calibration.
// Example: If you are using a 100g weight, set this to 100000.
#define KNOWN_CALIBRATION_WEIGHT_MG 194000

// --- UART Protocol Constants & Message IDs ---
const uint8_t UART_START_BYTE = 0xAA;
const uint8_t UART_END_BYTE = 0x55;

// MODIFIED: Define message IDs for our specific protocol
#define MSG_ID_REMOTE_CONTROL   0x0101 // Incoming command message
#define MSG_ID_SYSTEM_STATUS    0x0201 // Outgoing status message

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// PID controller instance
PID_TypeDef motorPID;

// Global variables for encoder and velocity calculation
volatile int32_t encoder_position = 0;
volatile float current_velocity = 0.0f;

// Global variables for target values
volatile int32_t target_position = 0;
int32_t target_velocity = 0;

// Global variables for PID calculation feedback (for Live Expressions)
volatile float current_error = 0.0f;
volatile float current_pid_output = 0.0f;

// Variable to store the current control mode
PID_CONTROL_MODE current_control_mode = POSITION_CONTROL;

// Encoder state tracking for interrupt-driven quadrature decoding
volatile uint8_t prev_encoder_state = 0;

// Variables for the manual motor
// Flag for the undecoded motor (on/off)
volatile uint8_t undecoded_motor_on_flag = 0;
volatile uint8_t manual_encoded_up_flag = 0;
volatile uint8_t manual_encoded_down_flag = 0;
uint16_t manual_pwm_value = 500; // PWM value for the manual motor
uint16_t manual_encoded_pwm_value = 600; // PWM for manual encoded motor control


// --- Declare Load Cell instances ---
LoadCell loadcell_1;
LoadCell loadcell_2;
LoadCell loadcell_3;

// --- Variables to hold the latest weights ---
long weight_1_mg = 0;
long weight_2_mg = 0;
long weight_3_mg = 0;

static UartTxBuffer_t uartTxBuffer = {0};
static volatile UART_TxState_t uart_tx_state = UART_TX_IDLE;
static uint8_t uart_tx_hal_buffer[20];
static UART_State_t uart_currentState = UART_STATE_WAITING_FOR_START;
static UART_IncomingMessage_t uart_receivedMsg;
static uint8_t uart_incomingByte;
static int uart_dataIndex = 0;
static uint8_t uart_calculatedChecksum = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Perform_Startup_Calibration(void);

void Gateway_QueueTxMessage(uint16_t id, uint8_t* data, uint8_t dlc);
void Gateway_ProcessUartByte(void);
void Uart_Tx_Try_Send(void);
void Send_Status_Over_UART(void); // MODIFIED: Renamed function
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/******************************************************UART FUNCTIONS*******************************************/

/******************************************************************************/
/* UART GATEWAY LOGIC (FROM EXAMPLE)                       */
/******************************************************************************/

static uint8_t uart_tx_buffer_put(uint8_t* data, uint8_t length) {
    uint16_t next_head = (uartTxBuffer.head + 1) & (UART_TX_BUFFER_SIZE - 1);
    if (next_head == uartTxBuffer.tail) return 0; // Buffer full
    uartTxBuffer.frames[uartTxBuffer.head].length = length;
    memcpy(uartTxBuffer.frames[uartTxBuffer.head].data, data, length);
    uartTxBuffer.head = next_head;
    return 1;
}

static uint8_t uart_tx_buffer_get(UartFrame_t* frame) {
    if (uartTxBuffer.head == uartTxBuffer.tail) return 0; // Buffer empty
    *frame = uartTxBuffer.frames[uartTxBuffer.tail];
    uartTxBuffer.tail = (uartTxBuffer.tail + 1) & (UART_TX_BUFFER_SIZE - 1);
    return 1;
}

static uint8_t uart_tx_buffer_is_empty(void) {
    return (uartTxBuffer.head == uartTxBuffer.tail);
}

void Gateway_QueueTxMessage(uint16_t id, uint8_t* data, uint8_t dlc) {
  uint8_t frame_buffer[20];
  int idx = 0;
  uint8_t checksum = 0;
  checksum ^= (id >> 8); checksum ^= (id & 0xFF); checksum ^= dlc;
  for (int i = 0; i < dlc; ++i) { checksum ^= data[i]; }
  frame_buffer[idx++] = UART_START_BYTE;
  frame_buffer[idx++] = (uint8_t)(id >> 8);
  frame_buffer[idx++] = (uint8_t)(id & 0xFF);
  frame_buffer[idx++] = dlc;
  memcpy(&frame_buffer[idx], data, dlc);
  idx += dlc;
  frame_buffer[idx++] = checksum;
  frame_buffer[idx++] = UART_END_BYTE;
  uart_tx_buffer_put(frame_buffer, idx);
  Uart_Tx_Try_Send();
}

void Uart_Tx_Try_Send(void) {
    __disable_irq();
    if (uart_tx_state == UART_TX_IDLE && !uart_tx_buffer_is_empty()) {
        uart_tx_state = UART_TX_BUSY;
        UartFrame_t frame_to_send;
        uart_tx_buffer_get(&frame_to_send);
        __enable_irq();
        memcpy(uart_tx_hal_buffer, frame_to_send.data, frame_to_send.length);
        if (HAL_UART_Transmit_IT(&huart1, uart_tx_hal_buffer, frame_to_send.length) != HAL_OK) {
            uart_tx_state = UART_TX_IDLE;
        }
    } else {
        __enable_irq();
    }
}

/**
 * @brief MODIFIED: Processes a single byte received from UART and executes commands.
 */
void Gateway_ProcessUartByte(void) {
  switch (uart_currentState) {
  case UART_STATE_WAITING_FOR_START:
    if (uart_incomingByte == UART_START_BYTE) {
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
    if (uart_receivedMsg.dlc <= 8) {
      uart_currentState = (uart_receivedMsg.dlc == 0) ? UART_STATE_READING_CHECKSUM : UART_STATE_READING_DATA;
    } else {
      uart_currentState = UART_STATE_WAITING_FOR_START;
    }
    break;
  case UART_STATE_READING_DATA:
    uart_receivedMsg.data[uart_dataIndex] = uart_incomingByte;
    uart_calculatedChecksum ^= uart_incomingByte;
    uart_dataIndex++;
    if (uart_dataIndex == uart_receivedMsg.dlc) {
      uart_currentState = UART_STATE_READING_CHECKSUM;
    }
    break;
  case UART_STATE_READING_CHECKSUM:
    if (uart_incomingByte == uart_calculatedChecksum) {
      uart_currentState = UART_STATE_READING_END;
    } else {
      uart_currentState = UART_STATE_WAITING_FOR_START;
    }
    break;
  case UART_STATE_READING_END:
    if (uart_incomingByte == UART_END_BYTE) {
      // A valid message has been received. ACT ON IT.
      switch(uart_receivedMsg.id) {
          case MSG_ID_REMOTE_CONTROL:
              if (uart_receivedMsg.dlc >= 8) { // Ensure message has enough data
                  // Byte 5: Manual/Auto mode
                  if (uart_receivedMsg.data[5] == 1) {
                      current_control_mode = MANUAL_ENCODED_CONTROL;
                  } else {
                      current_control_mode = POSITION_CONTROL;
                      // Bytes 0,1: Height data
                      int16_t height_data = (uart_receivedMsg.data[1] << 8) | uart_receivedMsg.data[0];
                      target_position = (int32_t)(((float)height_data / 8.0f) * ENCODER_PPR);
                  }
                  // Byte 3: Servo 1
                  if (uart_receivedMsg.data[3] == 1) {
                      setServoAngle(SERVO_1_TIMER, SERVO_1_CHANNEL, 180);
                  } else {
                      setServoAngle(SERVO_1_TIMER, SERVO_1_CHANNEL, 0);
                  }
                  // Byte 4: Undecoded motor
                  undecoded_motor_on_flag = (uart_receivedMsg.data[4] == 1);
                  // Bytes 6,7: Manual encoded motor
                  manual_encoded_up_flag = (uart_receivedMsg.data[6] == 1);
                  manual_encoded_down_flag = (uart_receivedMsg.data[7] == 1);
              }
              break;
      }
    }
    uart_currentState = UART_STATE_WAITING_FOR_START;
    break;
  }
}

/**
 * @brief MODIFIED: Packages and sends system status over UART.
 */
void Send_Status_Over_UART(void) {
    static uint32_t last_send_time = 0;
    if (HAL_GetTick() - last_send_time >= 100) {
        last_send_time = HAL_GetTick();
        uint8_t payload[4];
        // Bytes 0,1: Target Height in mm
        uint16_t height_mm = (uint16_t)(((float)target_position / (float)ENCODER_PPR) * 8.0f);
        payload[0] = height_mm & 0xFF;
        payload[1] = (height_mm >> 8) & 0xFF;
        // Bytes 2,3: Weight in grams
        uint16_t weight_g = (uint16_t)(weight_1_mg / 1000);
        payload[2] = weight_g & 0xFF;
        payload[3] = (weight_g >> 8) & 0xFF;
        Gateway_QueueTxMessage(MSG_ID_SYSTEM_STATUS, payload, 4);
    }
}

/****************************************************PID FUNCTIONS***********************************************/

/**
 * @brief HAL_GPIO_EXTI_Callback - Interrupt handler for encoder pins.
 * @param GPIO_Pin Specifies the pins connected to the EXTI line.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t current_A_state = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
    uint8_t current_B_state = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);
    uint8_t current_encoder_state = (current_A_state << 1) | current_B_state;

    if (current_encoder_state != prev_encoder_state)
    {
        static const int8_t lookup_table[] = {
            0,  -1, 1,  0,
            1,  0,  0,  -1,
            -1,  0,  0, 1,
            0, 1,  -1,  0
        };

        int index = (prev_encoder_state << 2) | current_encoder_state;
        if (index >= 0 && index < sizeof(lookup_table) / sizeof(lookup_table[0])) {
            encoder_position += lookup_table[index];
        }
    }
    prev_encoder_state = current_encoder_state;
}

void Motor_SetPosition(int32_t position_counts)
{
    target_position = position_counts;
    current_control_mode = POSITION_CONTROL;
}

void Motor_StopControl(void)
{
    current_control_mode = IDLE_MODE;
    Motor_SetPWM(0);
    Motor_SetDirection(MOTOR_STOP);
    PID_Reset(&motorPID); // Reset PID on stop
    target_velocity = 0;
    target_position = encoder_position;
    current_velocity = 0.0f;
}

/*************************************************LOADCELLS FUNCTIONS*******************************************************/

void Perform_Startup_Calibration(void) {
    // --- Step 1: Tare all load cells (ensure they are empty) ---
    // You might want to signal the user (e.g., blink an LED) that taring is in progress.
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    LoadCell_Tare(&loadcell_1, 15);
    LoadCell_Tare(&loadcell_2, 15);
    LoadCell_Tare(&loadcell_3, 15);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    // --- Step 2: Wait for user to place the known weight ---
    // Signal the user to place the calibration weight on one of the scales.
    // Here, we just wait for 5 seconds as an example.
    // A better approach would be to wait for a user button press.
    HAL_Delay(5000); // Wait 5 seconds

    // --- Step 3: Calibrate the load cells one by one ---
    // Example: Calibrating load cell 1.
    // The user should have placed the weight on load cell 1 during the delay.
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    LoadCell_Calibrate(&loadcell_1, KNOWN_CALIBRATION_WEIGHT_MG, 15);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_Delay(1000); // Short delay

    // IMPORTANT: Repeat the process for the other two load cells.
    // You will need a way to signal the user which scale to place the weight on.
    // For simplicity, this example only calibrates the first one. You can copy the
    // HAL_Delay -> Calibrate -> Delay sequence for the other two.
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
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // LED off

  /******************************************* UART SETUP ****************************************************/
    // Initialize the UART TX buffer
    memset(&uartTxBuffer, 0, sizeof(uartTxBuffer));
    // Start listening for incoming UART data (for future command/control)
    HAL_UART_Receive_IT(&huart1, &uart_incomingByte, 1);

  /*******************************************PID SETEUP****************************************************/

  // PID_Init moved here to be called only once.
  PID_Init(&motorPID, KP_POS, KI_POS, KD_POS, (float)htim15.Init.Period, -(float)htim15.Init.Period);
  // Start the main PWM channel (required for the timer base)
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  // Specifically start the complementary channel 'N' output
  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Start PWM for the manual motor
  prev_encoder_state = (HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin) << 1) |
                       HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);


  /*******************************************SERVOS SETEUP****************************************************/

  // Start the PWM signal for all four servos using their new names
//      HAL_TIM_PWM_Start(SERVO_3_TIMER, SERVO_3_CHANNEL); // Servo 3
//      HAL_TIM_PWM_Start(SERVO_1_TIMER, SERVO_1_CHANNEL); // Servo 1
//      HAL_TIM_PWM_Start(SERVO_2_TIMER, SERVO_2_CHANNEL); // Servo 2
//      HAL_TIM_PWM_Start(SERVO_4_TIMER, SERVO_4_CHANNEL); // Servo 4

//  	// Set all servos to the initial center position (90 degrees)
//      setServoAngle(SERVO_3_TIMER, SERVO_3_CHANNEL, 90);
//      setServoAngle(SERVO_1_TIMER, SERVO_1_CHANNEL, 90);
//      setServoAngle(SERVO_2_TIMER, SERVO_2_CHANNEL, 90);
//      setServoAngle(SERVO_4_TIMER, SERVO_4_CHANNEL, 90);
//
//      HAL_Delay(200); // Give servos time to reach initial position


/*******************************************LOADCELLS SETEUP****************************************************/

//      HAL_TIM_Base_Start(&htim1);
//
//      // Initialize all three load cells
//	  LoadCell_Init(&loadcell_1, LC1_DATA_GPIO_Port, LC1_DATA_Pin, LC1_SCK_GPIO_Port, LC1_SCK_Pin, &htim1);
//	  LoadCell_Init(&loadcell_2, LC2_DATA_GPIO_Port, LC2_DATA_Pin, LC2_SCK_GPIO_Port, LC2_SCK_Pin, &htim1);
//	  LoadCell_Init(&loadcell_3, LC3_DATA_GPIO_Port, LC3_DATA_Pin, LC3_SCK_GPIO_Port, LC3_SCK_Pin, &htim1);
//
//	  // Perform the startup calibration routine
//	  Perform_Startup_Calibration();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/***************************************************PID LOGIC***********************************************/

	  switch (current_control_mode)
	      {
	          case POSITION_CONTROL:
	              current_error = (float)target_position - encoder_position;
	              current_pid_output = PID_Compute(&motorPID, current_error, 0.01f);
	              if (current_pid_output > 50) {
	                  Motor_SetDirection(MOTOR_FORWARD);
	                  Motor_SetPWM((uint16_t)current_pid_output);
	              } else if (current_pid_output < -50) {
	                  Motor_SetDirection(MOTOR_REVERSE);
	                  Motor_SetPWM((uint16_t)fabs(current_pid_output));
	              } else {
	                  Motor_SetDirection(MOTOR_STOP);
	                  Motor_SetPWM(0);
	              }
	              break;
	          case MANUAL_ENCODED_CONTROL:
	              PID_Reset(&motorPID);
	              if (manual_encoded_up_flag && !manual_encoded_down_flag) {
	                  Motor_SetDirection(MOTOR_FORWARD);
	                  Motor_SetPWM(manual_encoded_pwm_value);
	              } else if (manual_encoded_down_flag && !manual_encoded_up_flag) {
	                  Motor_SetDirection(MOTOR_REVERSE);
	                  Motor_SetPWM(manual_encoded_pwm_value);
	              } else {
	                  Motor_SetDirection(MOTOR_STOP);
	                  Motor_SetPWM(0);
	              }
	              break;
	          case IDLE_MODE:
	          default:
	              Motor_SetDirection(MOTOR_STOP);
	              Motor_SetPWM(0);
	              PID_Reset(&motorPID);
	              break;
	      }

	      if (undecoded_motor_on_flag) {
	          Motor_SetManualPWM(manual_pwm_value, GPIO_PIN_SET);
	      } else {
	          Motor_SetManualPWM(0, GPIO_PIN_RESET);
	      }


/********************************************LOADCELLS CODE****************************************/

//      // Continuously run the non-blocking manager for each load cell
//	  LoadCell_Manager(&loadcell_1);
//	  LoadCell_Manager(&loadcell_2);
//	  LoadCell_Manager(&loadcell_3);
//
//	  // Check if a new, averaged weight reading is ready from any load cell
//	  if (loadcell_1.new_weight_available) {
//		weight_1_mg = LoadCell_GetWeight(&loadcell_1);
//		// You can now use the `weight_1_mg` variable.
//	  }
//
//	  if (loadcell_2.new_weight_available) {
//		weight_2_mg = LoadCell_GetWeight(&loadcell_2);
//		// You can now use the `weight_2_mg` variable.
//	  }
//
//	  if (loadcell_3.new_weight_available) {
//		weight_3_mg = LoadCell_GetWeight(&loadcell_3);
//		// You can now use the `weight_3_mg` variable.
//	  }
//
//	  // You can add other non-blocking tasks here.
//	  // Example: Print weights via UART or update an LCD.
//	  // Note: The main loop no longer needs a HAL_Delay() as the managers are non-blocking.

      /********************************************LOADCELLS & UART****************************************/
           LoadCell_Manager(&loadcell_1); // Uncomment if using load cells
           if (loadcell_1.new_weight_available) {
               weight_1_mg = LoadCell_GetWeight(&loadcell_1);
           }

          Send_Status_Over_UART();
          HAL_Delay(10); // Maintain a consistent loop rate (100 Hz)

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 47;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 95;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  HAL_GPIO_WritePin(GPIOA, MOTOR_IN1_Pin|LC2_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LC1_SCK_Pin|MOTOR_MANUAL_DIR_Pin|LC3_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_IN1_Pin LC2_SCK_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_Pin|LC2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LC1_SCK_Pin MOTOR_MANUAL_DIR_Pin LC3_SCK_Pin */
  GPIO_InitStruct.Pin = LC1_SCK_Pin|MOTOR_MANUAL_DIR_Pin|LC3_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LC2_DATA_Pin LC3_DATA_Pin */
  GPIO_InitStruct.Pin = LC2_DATA_Pin|LC3_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LC1_DATA_Pin */
  GPIO_InitStruct.Pin = LC1_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LC1_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_A_Pin ENCODER_B_Pin */
  GPIO_InitStruct.Pin = ENCODER_A_Pin|ENCODER_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/******************************************************************************/
/* HAL CALLBACKS                                    */
/******************************************************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    Gateway_ProcessUartByte();
    HAL_UART_Receive_IT(&huart1, &uart_incomingByte, 1);
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    // This is a custom function from the gateway logic
    uart_tx_state = UART_TX_IDLE;
    Uart_Tx_Try_Send();
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
