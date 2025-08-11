/*
 * motor_control.c
 *
 * Created on: Jun 26, 2025
 * Author: Adham
 */

#include "motor_control.h"
#include <math.h> // For roundf(), fmaxf(), fminf()

// Declare htim2 as extern since it's used here for PWM
extern TIM_HandleTypeDef htim2;

void Motor_SetDirection(MotorDirection_TypeDef direction)
{
    // Assuming MOTOR_DIR_Pin is correctly defined in main.h
    // For Cytron driver, typically HIGH for one direction, LOW for the other.
    // PWM controls speed.
    switch (direction)
    {
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
            break;
        case MOTOR_REVERSE:
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
            break;
        case MOTOR_STOP:
        default:
            // When stopped, direction pin state might not matter, but setting it to a default
            // (e.g., LOW) can be good practice. PWM should be 0.
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
            break;
    }
}

void Motor_SetPWM(float pwm_value)
{
    // Clamp the float PWM value to the valid uint16_t range [0, Period]
    // Use roundf to round to nearest integer for better precision
    uint16_t pwm_duty_cycle = (uint16_t)roundf(fmaxf(0.0f, fminf(pwm_value, (float)htim2.Init.Period)));

    // Set the compare value for TIM2 Channel 2 (connected to PWM)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_duty_cycle);
}
