/*
 * motor_control.h
 *
 * Created on: Mar 15, 2025
 * Author: Adham
 */

#ifndef MOTOR_CONTROL_MOTOR_CONTROL_H_
#define MOTOR_CONTROL_MOTOR_CONTROL_H_

#include "main.h"

// Forward declare the global PWM timer handle from main.c
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim3;

// Enum for motor direction
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} MOTOR_DIRECTION;


// Function prototypes
void Motor_SetDirection(MOTOR_DIRECTION direction);
void Motor_SetPWM(uint16_t pwm_value);
void Motor_SetManualPWM(uint16_t pwm_value, GPIO_PinState direction);

#endif /* MOTOR_CONTROL_MOTOR_CONTROL_H_ */
