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
#include <stdbool.h> // Include for boolean types

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_PORT GPIOB
#define DT_PIN GPIO_PIN_13
#define SCK_PORT GPIOB
#define SCK_PIN GPIO_PIN_9

#define NUM_SAMPLES 32 // Number of samples to average for a stable reading
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// --- HX711 State and Data Variables ---
static long tare_offset = 0;
// IMPORTANT: You must determine this calibration factor yourself!
// See the "How to Calibrate" section below. A value of 420.0 is a common starting point.
static float calibration_factor = 837.0;

volatile long current_weight_mg = 0;
volatile bool new_weight_available = false;

// Non-blocking state machine variables
typedef enum {
    HX711_IDLE,
    HX711_WAIT_FOR_READY,
    HX711_READING_BITS
} HX711_State;

static HX711_State hx711_state = HX711_IDLE;
static uint32_t raw_data_buffer = 0;
static uint8_t bit_counter = 0;
static long sample_accumulator = 0;
static uint8_t sample_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void microDelay(uint16_t delay);
long Read_HX711_Raw(void);
void HX711_Tare(void);
void HX711_Manager(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void microDelay(uint16_t delay) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < delay);
}

/**
  * @brief  Reads a single raw value from the HX711. This is a blocking function.
  * @note   This function is only used during initialization (Tare).
  * The main loop uses the non-blocking manager.
  * @retval The 24-bit raw ADC value.
  */
long Read_HX711_Raw(void) {
    long data = 0;
    uint32_t start_time = HAL_GetTick();

    // 1. Wait for the HX711 to become ready (DT pin goes LOW)
    while (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET) {
        if (HAL_GetTick() - start_time > 200) { // Timeout
            return 0;
        }
    }

    // 2. Read the 24 bits of data
    for (uint8_t i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
        microDelay(1);
        data = data << 1;
        HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
        microDelay(1);
        if (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET) {
            data++;
        }
    }

    // 3. Send 1 clock pulse to set gain to 128 for the next reading
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
    microDelay(1);
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
    microDelay(1);

    // 4. Convert from 24-bit two's complement to 32-bit signed long
    data = data ^ 0x800000;

    return data;
}

/**
  * @brief  Performs a tare operation by reading the sensor multiple times
  * to get a stable zero-offset.
  * @note   Ensure NO LOAD is on the scale when this is called.
  */
void HX711_Tare(void) {
    long total = 0;
    // Let the ADC settle
    for (int i = 0; i < 5; i++) {
        Read_HX711_Raw();
        HAL_Delay(10);
    }
    // Average several readings for a stable tare value
    for (int i = 0; i < NUM_SAMPLES; i++) {
        total += Read_HX711_Raw();
        HAL_Delay(10);
    }
    tare_offset = total / NUM_SAMPLES;
}


/**
  * @brief  Non-blocking state machine to manage HX711 readings.
  * @note   Call this function repeatedly in your main while loop.
  */
void HX711_Manager(void) {
    switch (hx711_state) {
        case HX711_IDLE:
            // If the HX711 is ready for a new reading, start the process
            if (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_RESET) {
                raw_data_buffer = 0;
                bit_counter = 0;
                hx711_state = HX711_READING_BITS;
            }
            break;

        case HX711_WAIT_FOR_READY: // This state is now implicitly handled by HX711_IDLE
            // Kept for conceptual clarity, but the logic is merged into IDLE
            break;

        case HX711_READING_BITS:
            // Pulse the clock high
            HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
            microDelay(1);

            // Shift and read the bit
            raw_data_buffer = raw_data_buffer << 1;
            if (HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET) {
                raw_data_buffer++;
            }

            // Pulse the clock low
            HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
            microDelay(1);

            bit_counter++;

            // Check if all 24 bits have been read
            if (bit_counter >= 24) {
                // Send the final clock pulse to set gain for the next reading
                HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
                microDelay(1);
                HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
                microDelay(1);

                // Convert to signed value and add to accumulator
                long final_data = (long)(raw_data_buffer ^ 0x800000);
                sample_accumulator += final_data;
                sample_counter++;

                // Reset for the next reading cycle
                hx711_state = HX711_IDLE;
            }
            break;
    }

    // Check if we have collected enough samples to calculate a weight
    if (sample_counter >= NUM_SAMPLES) {
        long average_raw = sample_accumulator / NUM_SAMPLES;

        // Calculate the final weight
        current_weight_mg = (long)(((float)average_raw - (float)tare_offset) / calibration_factor);
//        current_weight_mg = average_raw - tare_offset; //for calibration only
        new_weight_available = true; // Set flag for the main loop

        // Reset for the next averaging cycle
        sample_accumulator = 0;
        sample_counter = 0;
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);

  // Power up sequence for HX711
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);

  // Tare the scale at startup. Make sure nothing is on the scale!
  HX711_Tare();
  HAL_Delay(1500);
  HX711_Tare();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Continuously run the non-blocking manager
    HX711_Manager();

    // Check if a new, averaged weight reading is ready
    if (new_weight_available) {
      new_weight_available = false; // Clear the flag

      // You can now use the `current_weight_mg` variable.
      // For example, print it via UART, display it on an LCD, etc.
      // NOTE: For debugging, you can set a breakpoint here to check the value.
      __NOP(); // No operation, a good place for a breakpoint
    }

    // You can add other non-blocking tasks here.
    // For example, blinking an LED.
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Example for NUCLEO-F0 board LED
    // HAL_Delay(100); // Small delay to prevent the loop from hogging CPU

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HX711_DT_Pin */
  GPIO_InitStruct.Pin = HX711_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HX711_DT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HX711_SCK_Pin */
  GPIO_InitStruct.Pin = HX711_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HX711_SCK_GPIO_Port, &GPIO_InitStruct);

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
