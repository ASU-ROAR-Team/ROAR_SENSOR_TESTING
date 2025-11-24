/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : servo.h
  * @brief          : Header file for the servo control module.
  ******************************************************************************
  * @attention
  *
  * This file provides function prototypes for controlling a hobby servo
  * using a timer in PWM mode.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported defines ----------------------------------------------------------*/
/* USER CODE BEGIN ED */
// These values define the PWM pulse widths for the servo motor.
// They are based on a 20ms (50Hz) PWM period with 10000 total counts (Period=9999).
// A 0.5ms pulse is 250 counts.
// A 2.5ms pulse is 1250 counts.
#define MIN_PULSE_COUNTS 250   // Pulse value for 0 degrees (0.5ms)
#define MAX_PULSE_COUNTS 1250  // Pulse value for 180 degrees (2.5ms)
#define PULSE_RANGE      1000  // The range of the pulse counts (1250 - 250)
/* USER CODE END ED */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/**
 * @brief  Sets the servo to a specific angle by updating the PWM compare register.
 * @param  htim A pointer to the TIM_HandleTypeDef structure for your timer.
 * @param  channel The timer channel used for the servo (e.g., TIM_CHANNEL_3).
 * @param  angle The desired servo angle in degrees (0 to 180).
 */
void setServoAngle(TIM_HandleTypeDef* htim, uint32_t channel, uint16_t angle);

/**
 * @brief  Maps a desired angle (0-180) to the corresponding PWM pulse count.
 * @param  angle The desired angle in degrees (0 to 180).
 * @retval The calculated pulse value for the timer's compare register.
 */
uint16_t angleToPulseValue(uint16_t angle);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
