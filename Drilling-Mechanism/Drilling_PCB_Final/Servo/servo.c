/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : servo.c
  * @brief          : Servo control functions.
  ******************************************************************************
  * @attention
  *
  * This file provides the implementation for the servo control functions.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "servo.h"

/* USER CODE BEGIN 0 */

/**
 * @brief  Maps a desired angle (0-180) to the corresponding PWM pulse count.
 * @param  angle The desired angle in degrees (0 to 180).
 * @retval The calculated pulse value for the timer's compare register.
 */
uint16_t angleToPulseValue(uint16_t angle)
{
    // Clamp the angle to ensure it stays within 0-180 degrees
    if (angle > 180) {
        angle = 180;
    }

    // Linear mapping formula:
    // pulse_counts = MIN_PULSE_COUNTS + (angle * PULSE_RANGE) / 180
    uint16_t pulse = MIN_PULSE_COUNTS + ((uint32_t)angle * PULSE_RANGE) / 180;

    return pulse;
}

/**
 * @brief  Sets the servo to a specific angle by updating the PWM compare register.
 * @param  htim A pointer to the TIM_HandleTypeDef structure for your timer.
 * @param  channel The timer channel used for the servo (e.g., TIM_CHANNEL_3).
 * @param  angle The desired servo angle in degrees (0 to 180).
 */
void setServoAngle(TIM_HandleTypeDef* htim, uint32_t channel, uint16_t angle)
{
    uint16_t pulse = angleToPulseValue(angle);

    // Update the Capture Compare Register (CCR) for the specified channel.
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

/* USER CODE END 0 */
