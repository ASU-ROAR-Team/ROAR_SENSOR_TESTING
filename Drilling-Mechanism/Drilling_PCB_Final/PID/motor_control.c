/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor_control.c
  * @brief          : This file provides functions for motor control.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h" // Includes all necessary headers
#include "motor_control.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
/**
  * @brief Set the PWM duty cycle for the encoded motor.
  * @param pwm_value: The PWM value to set (0-999).
  */
void Motor_SetPWM(uint16_t pwm_value) {
    // NOTE: We set the compare value for Channel 1, which controls both CH1 and CH1N outputs.
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pwm_value);
}

/**
  * @brief Set the direction for the encoded motor.
  * @param direction: The direction to set (MOTOR_FORWARD, MOTOR_REVERSE, MOTOR_STOP).
  */
void Motor_SetDirection(MOTOR_DIRECTION direction) {
    if (direction == MOTOR_FORWARD) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
    } else if (direction == MOTOR_REVERSE) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
    } else {
        // Stop state, direction doesn't matter, but we can reset for safety
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
    }
}

/**
  * @brief Function to control the non-encoded motor.
  * @param pwm_value: The PWM value to set (0-999).
  * @param direction: The direction to set (GPIO_PIN_SET for up, GPIO_PIN_RESET for down).
  */
void Motor_SetManualPWM(uint16_t pwm_value, GPIO_PinState direction)
{
    // Set direction pin for the manual motor
    HAL_GPIO_WritePin(MOTOR_MANUAL_DIR_GPIO_Port, MOTOR_MANUAL_DIR_Pin, direction);
    // Set PWM for the manual motor on TIM3 Channel 2
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);
}

/* USER CODE END 0 */
