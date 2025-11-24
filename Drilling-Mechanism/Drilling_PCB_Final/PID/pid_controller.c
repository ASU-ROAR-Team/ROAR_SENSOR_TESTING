/*
 * pid_controller.c
 *
 * Created on: Mar 15, 2025
 * Author: Adham
 */

#include "pid_controller.h"

/**
 * @brief Initializes a PID controller instance.
 * @param pid Pointer to the PID_TypeDef struct to initialize.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param output_max Maximum possible output value.
 * @param output_min Minimum possible output value.
 */
void PID_Init(PID_TypeDef* pid, float Kp, float Ki, float Kd, float output_max, float output_min)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral_sum = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_max = output_max;
    pid->output_min = output_min;
}

/**
 * @brief Computes a new PID output based on the current error and time step.
 * @param pid Pointer to the PID_TypeDef struct.
 * @param error Current error value (e.g., target_position - encoder_position).
 * @param dt The time difference since the last call, in seconds.
 * @return The computed PID output value.
 */
float PID_Compute(PID_TypeDef* pid, float error, float dt)
{
    // Proportional term
    float P_out = pid->Kp * error;

    // Integral term (anti-windup by clamping the integral sum)
    pid->integral_sum += error * dt;
    if (pid->integral_sum > pid->output_max) {
        pid->integral_sum = pid->output_max;
    } else if (pid->integral_sum < pid->output_min) {
        pid->integral_sum = pid->output_min;
    }
    float I_out = pid->Ki * pid->integral_sum;

    // Derivative term (avoiding noisy measurements by using a dt value)
    float derivative = (error - pid->previous_error) / dt;
    float D_out = pid->Kd * derivative;

    // Update previous error for the next iteration
    pid->previous_error = error;

    // Sum the terms and clamp the output
    float output = P_out + I_out + D_out;
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return output;
}

/**
 * @brief Resets the integral sum and previous error of the PID controller.
 * @param pid Pointer to the PID_TypeDef struct.
 */
void PID_Reset(PID_TypeDef* pid)
{
    pid->integral_sum = 0.0f;
    pid->previous_error = 0.0f;
}
