/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Position Control Ready (Cleaned ISRs)
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
#include <stdio.h>          // For printf (if using UART for debug)
#include "stm32f0xx_hal_tim.h" // Explicitly include HAL TIM header to resolve MspPostInit warning
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ENCODER_PPR 12000 // Counts Per Revolution (as specified by user)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// PID controller instance
PID_TypeDef motorPID;

// Global variables for encoder and velocity calculation
volatile int32_t encoder_position = 0;      // Current unrolled 32-bit encoder position
volatile float current_velocity = 0.0f;     // Calculated current velocity

// Global variables for target values
volatile int32_t target_position = 0; // Made volatile for debugger modification
int32_t target_velocity = 0; // Not actively used in this position control setup

// Global variables for PID calculation feedback (for Live Expressions)
volatile float current_error = 0.0f;
volatile float current_pid_output = 0.0f;

// Variable to store the current control mode
PID_CONTROL_MODE current_control_mode = POSITION_CONTROL; // Start directly in POSITION_CONTROL

// Global variable for loop timing (for PID dt calculation)
uint32_t last_loop_time = 0;

// PID Gains for position control (you will tune these!)
#define KP_POS 0.5f   // Start with P-only, then add I, then D
#define KI_POS 0.001f
#define KD_POS 0.01f

// PID Gains for velocity control (defined here for completeness, not actively used)
#define KP_VEL 0.43f
#define KI_VEL 0.47f
#define KD_VEL 0.0042f

// Encoder state tracking for interrupt-driven quadrature decoding
volatile uint8_t prev_encoder_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief HAL_GPIO_EXTI_Callback - Interrupt handler for encoder pins.
 * This function is called by the HAL when an external interrupt occurs on PB6 or PB7.
 * It decodes the quadrature signal to update the encoder_position.
 * @param GPIO_Pin Specifies the pins connected to the EXTI line.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Read current states of both encoder channels
    uint8_t current_A_state = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
    uint8_t current_B_state = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);

    // Combine current states into a 2-bit value: (A << 1) | B
    uint8_t current_encoder_state = (current_A_state << 1) | current_B_state;

    // Only update if the state has actually changed to avoid double counting from noise
    // This also acts as a simple form of debounce.
    if (current_encoder_state != prev_encoder_state)
    {
        // Lookup table for 4x quadrature decoding.
        // The index is formed by (previous_state << 2) | current_state
        // Values: 1 for CW (forward), -1 for CCW (reverse), 0 for invalid/no change
        static const int8_t lookup_table[] = {
            0,  1, -1,  0,  // 00 -> 00, 01, 10, 11 (invalid)
            -1, 0,  0,  1,  // 01 -> 00, 01, 10 (invalid), 11
            1,  0,  0, -1,  // 10 -> 00, 01 (invalid), 10, 11
            0, -1,  1,  0   // 11 -> 00 (invalid), 01, 10, 11
        };

        // Calculate lookup table index
        int index = (prev_encoder_state << 2) | current_encoder_state;

        // Update encoder position based on the lookup table result
        if (index >= 0 && index < sizeof(lookup_table) / sizeof(lookup_table[0])) {
            encoder_position += lookup_table[index];
        }
    }

    // Store current state for the next interrupt
    prev_encoder_state = current_encoder_state;
}


/**
 * @brief Sets the motor to a specific position.
 * @param position_counts The desired target position in encoder counts.
 */
void Motor_SetPosition(int32_t position_counts)
{
    target_position = position_counts;
    current_control_mode = POSITION_CONTROL;
    // Re-initialize PID for position control with appropriate gains
    PID_Init(&motorPID, KP_POS, KI_POS, KD_POS, (float)htim2.Init.Period, -(float)htim2.Init.Period);
    // When switching modes, it's often good to reset integral sum to prevent windup from previous mode
    motorPID.integral_sum = 0.0f;
    motorPID.previous_error = 0.0f;
}

/**
 * @brief Sets the motor to a specific velocity.
 * @param velocity_counts_per_sec The desired target velocity in encoder counts per second.
 * @note This function is included for API consistency but velocity control will not be active
 * in this "position control only" setup unless you enable it in the main loop.
 */
void Motor_SetVelocity(int32_t velocity_counts_per_sec)
{
    target_velocity = velocity_counts_per_sec;
    current_control_mode = VELOCITY_CONTROL; // Still set the mode, but it won't be processed
    // Re-initialize PID for velocity control with appropriate gains
    PID_Init(&motorPID, KP_VEL, KI_VEL, KD_VEL, (float)htim2.Init.Period, -(float)htim2.Init.Period);
    // Reset integral sum and previous error to ensure clean start for velocity PID
    motorPID.integral_sum = 0.0f;
    motorPID.previous_error = 0.0f;
}

/**
 * @brief Stops the motor and disables the PID control loop.
 */
void Motor_StopControl(void)
{
    current_control_mode = IDLE_MODE;
    Motor_SetPWM(0); // Ensure motor is off
    Motor_SetDirection(MOTOR_STOP); // Ensure direction pin is neutral
    // Reset PID state to ensure clean start next time
    motorPID.previous_error = 0.0f;
    motorPID.integral_sum = 0.0f;
    // Reset target values and current states for clarity
    target_velocity = 0;
    target_position = encoder_position; // Hold current position if switching from position mode to idle
    current_velocity = 0.0f;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Initialize PID controller (default to position control gains at start)
  PID_Init(&motorPID, KP_POS, KI_POS, KD_POS, (float)htim2.Init.Period, -(float)htim2.Init.Period);

  // Start the PWM timer
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Assuming PWM is on TIM2 Channel 2

  // Initialize encoder state for interrupt decoding
  prev_encoder_state = (HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin) << 1) |
                       HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);

  // No Motor_StopControl() here, as we want to start directly in position control
  // with target_position initialized to 0.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // The encoder_position is updated asynchronously by HAL_GPIO_EXTI_Callback

      // Calculate time difference since last loop iteration (for dt)
      uint32_t current_loop_time = HAL_GetTick();
      float dt_ms = (float)(current_loop_time - last_loop_time);
      last_loop_time = current_loop_time; // Update for the next iteration

      float dt_sec = dt_ms / 1000.0f;
      if (dt_sec == 0.0f) { // Prevent division by zero if tick hasn't advanced
          dt_sec = 0.001f; // Assume a small non-zero time
      }


      // Control Mode Logic (only POSITION_CONTROL is active here)
      switch (current_control_mode)
      {
          case POSITION_CONTROL:
              // Calculate position error
              current_error = (float)target_position - encoder_position;
              // Compute PID output
              current_pid_output = PID_Compute(&motorPID, current_error);
              break;

          case VELOCITY_CONTROL: // Not actively used in this setup
          case IDLE_MODE:        // Not actively used in this setup (unless Motor_StopControl() is called)
          default:
              // If somehow not in POSITION_CONTROL, ensure motor is off
              current_error = 0.0f;
              current_pid_output = 0.0f;
              current_velocity = 0.0f;
              Motor_SetPWM(0);
              Motor_SetDirection(MOTOR_STOP);
              break;
      }


      // Apply PID output to motor
      if (current_control_mode == POSITION_CONTROL) // Only control motor if in position control mode
      {
          if (current_pid_output > 0)
          {
              Motor_SetDirection(MOTOR_FORWARD);
              Motor_SetPWM(current_pid_output);
          }
          else if (current_pid_output < 0)
          {
              Motor_SetDirection(MOTOR_REVERSE);
              Motor_SetPWM(fabs(current_pid_output));
          }
          else // current_pid_output is 0 or very close to 0
          {
              Motor_SetDirection(MOTOR_STOP);
              Motor_SetPWM(0);
          }
      }

      HAL_Delay(10); // Maintain a consistent loop rate (100 Hz) for PID calculations

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // No demo state machine here, you'll set target_position manually in debugger
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
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MOTOR_IN1_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_A_Pin ENCODER_B_Pin */
  GPIO_InitStruct.Pin = ENCODER_A_Pin|ENCODER_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
