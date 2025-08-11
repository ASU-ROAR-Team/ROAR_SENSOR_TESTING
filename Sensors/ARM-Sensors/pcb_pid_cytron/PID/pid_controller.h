/*
 * pid_controller.h
 *
 * Created on: Jun 26, 2025
 * Author: Adham
 */

#ifndef PID_PID_CONTROLLER_H_
#define PID_PID_CONTROLLER_H_

#include "stm32f0xx_hal.h" // Correct HAL for F072
#include <stdint.h> // For int32_t
#include <math.h>   // For fabs, fmaxf, fminf

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral_sum;
    float output_limit_max;
    float output_limit_min;
    uint32_t last_time; // For dt calculation
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_max, float output_min);
float PID_Compute(PID_TypeDef *pid, float error);


#endif /* PID_PID_CONTROLLER_H_ */

