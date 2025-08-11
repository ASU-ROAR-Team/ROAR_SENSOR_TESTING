/*
 * pid_controller.c
 *
 * Created on: Jun 26, 2025
 * Author: Adham
 */

#include "pid_controller.h"
#include <math.h> // Include for fmaxf/fminf if you use them, and for general math functions

// You might also need to define an epsilon for float comparisons if 0.0f is not exact
#define FLOAT_EPSILON 0.00001f

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_max, float output_min)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0.0f;
    pid->integral_sum = 0.0f;
    pid->output_limit_max = output_max;
    pid->output_limit_min = output_min;
    pid->last_time = HAL_GetTick(); // Initialize last_time
}

float PID_Compute(PID_TypeDef *pid, float error)
{
    uint32_t current_time = HAL_GetTick();
    float delta_time_ms = (float)(current_time - pid->last_time);
    pid->last_time = current_time;

    // Convert delta_time to seconds for more standard PID calculations
    // Ensure dt is not zero to prevent division by zero in derivative term if loop runs too fast
    float dt = delta_time_ms / 1000.0f;
    if (dt == 0.0f) { // If delta_time_ms is 0, dt would be 0. This can happen if HAL_GetTick() doesn't change
        dt = 0.001f; // Assume a small non-zero time if tick hasn't changed. Or, skip this loop iteration.
    }


    // Proportional term
    float proportional_term = pid->Kp * error;

    // Integral term
    float integral_term = 0.0f; // Initialize to 0.0f

    // Only update integral sum if Ki is non-zero to avoid division by zero and unnecessary accumulation
    if (fabs(pid->Ki) > FLOAT_EPSILON) // Use fabs and epsilon for robust float comparison to zero
    {
        pid->integral_sum += error * dt;

        // Anti-windup: Clamp integral_sum to prevent it from growing too large.
        // The limits for integral_sum are derived from the output limits divided by Ki.
        float integral_sum_limit_max = pid->output_limit_max / pid->Ki;
        float integral_sum_limit_min = pid->output_limit_min / pid->Ki;

        // Ensure proper clamping even if Ki is negative (min/max swap)
        if (pid->Ki < 0) { // If Ki is negative, swap the limits
            float temp = integral_sum_limit_max;
            integral_sum_limit_max = integral_sum_limit_min;
            integral_sum_limit_min = temp;
        }

        if (pid->integral_sum > integral_sum_limit_max) {
            pid->integral_sum = integral_sum_limit_max;
        } else if (pid->integral_sum < integral_sum_limit_min) {
            pid->integral_sum = integral_sum_limit_min;
        }

        integral_term = pid->Ki * pid->integral_sum;
    } else {
        pid->integral_sum = 0.0f; // Ensure integral_sum is reset if Ki is effectively zero
    }


    // Derivative term
    float derivative_term = pid->Kd * ((error - pid->previous_error) / dt);

    // Total PID output
    float output = proportional_term + integral_term + derivative_term;

    // Clamp total output to defined limits
    if (output > pid->output_limit_max) {
        output = pid->output_limit_max;
    } else if (output < pid->output_limit_min) {
        output = pid->output_limit_min;
    }

    // Store current error for next iteration's derivative calculation
    pid->previous_error = error;

    return output;
}
