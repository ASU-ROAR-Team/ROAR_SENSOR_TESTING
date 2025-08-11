/*
 * motor_control.h
 *
 * Created on: Jun 26, 2025
 * Author: Adham
 */

#ifndef MOTOR_MOTOR_CONTROL_H_
#define MOTOR_MOTOR_CONTROL_H_

#include "stm32f0xx_hal.h" // Correct HAL for F072
#include <stdint.h> // For uint16_t
#include <math.h>   // For roundf, fmaxf, fminf

// Include main.h to get access to MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, etc.
#include "../Core/Inc/main.h"

typedef enum {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} MotorDirection_TypeDef;

void Motor_SetPWM(float pwm_value); // Takes float as input from PID
void Motor_SetDirection(MotorDirection_TypeDef direction);

// Declare htim2 as extern since it's used in motor_control.c for PWM
extern TIM_HandleTypeDef htim2;

#endif /* MOTOR_MOTOR_CONTROL_H_ */

