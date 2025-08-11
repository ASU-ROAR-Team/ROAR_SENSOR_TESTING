/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @author         : Adham Ehab
  * @brief          : Header for main.c file.
  * This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h> // For int32_t etc.
#include <math.h>   // For fabs
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// Define the control modes for PID
typedef enum {
    IDLE_MODE,          // Motor is stopped, no PID active
    POSITION_CONTROL,   // PID actively controls motor to a target position
    VELOCITY_CONTROL    // PID actively controls motor to a target velocity
} PID_CONTROL_MODE;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

// API functions for motor control modes
void Motor_SetPosition(int32_t position_counts);
void Motor_SetVelocity(int32_t velocity_counts_per_sec);
void Motor_StopControl(void); // Function to explicitly stop PID and motor

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_IN1_Pin GPIO_PIN_5
#define MOTOR_IN1_GPIO_Port GPIOA
#define ENCODER_A_Pin GPIO_PIN_6
#define ENCODER_A_GPIO_Port GPIOB
#define ENCODER_A_EXTI_IRQn EXTI4_15_IRQn
#define ENCODER_B_Pin GPIO_PIN_7
#define ENCODER_B_GPIO_Port GPIOB
#define ENCODER_B_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */
// Motor Direction Pin (Cytron driver uses one direction pin)
#define MOTOR_DIR_Pin GPIO_PIN_5
#define MOTOR_DIR_GPIO_Port GPIOA

// Encoder Channels (PB6 and PB7)
#define ENCODER_A_Pin GPIO_PIN_6
#define ENCODER_A_GPIO_Port GPIOB
#define ENCODER_B_Pin GPIO_PIN_7
#define ENCODER_B_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
