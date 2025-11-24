/*
 * pid_controller.h
 *
 * Created on: Mar 15, 2025
 * Author: Adham
 */

#ifndef PID_CONTROLLER_PID_CONTROLLER_H_
#define PID_CONTROLLER_PID_CONTROLLER_H_

#include <stdint.h>

// Struct to hold PID controller state
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral_sum;
    float previous_error;
    float output_max;
    float output_min;
} PID_TypeDef;

// Control modes for the PID controller
//typedef enum {
//    IDLE_MODE = 0,
//    POSITION_CONTROL,
//    VELOCITY_CONTROL
//} PID_CONTROL_MODE;

// Function prototypes
void PID_Init(PID_TypeDef* pid, float Kp, float Ki, float Kd, float output_max, float output_min);
float PID_Compute(PID_TypeDef* pid, float error, float dt);
void PID_Reset(PID_TypeDef* pid);

#endif /* PID_CONTROLLER_PID_CONTROLLER_H_ */
