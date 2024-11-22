#ifndef PICO_MOTOR_H
#define PICO_MOTOR_H

#include "pico/stdlib.h"

#define GO_FORWARD 1
#define GO_BACKWARD 2

// Define pins for Motor 1 (same as the current setup)
#define MOTOR1_PWM_PIN 2 // PWM-capable GPIO pin for motor 1 speed control (ENA)
#define MOTOR1_IN1_PIN 0 // GPIO pin for motor 1 direction control (IN1)
#define MOTOR1_IN2_PIN 1 // GPIO pin for motor 1 direction control (IN2)

// Define pins for Motor 2 (new setup)
#define MOTOR2_PWM_PIN 6 // PWM-capable GPIO pin for motor 2 speed control (ENA)
#define MOTOR2_IN1_PIN 4 // GPIO pin for motor 2 direction control (IN1)
#define MOTOR2_IN2_PIN 5 // GPIO pin for motor 2 direction control (IN2)

// Speed settings
#define MIN_DUTY_CYCLE 2500
#define MAX_DUTY_CYCLE 65535
// #define MAX_DUTY_CYCLE 12500
#define NORMAL_DUTY_CYCLE 5000      // ~35% speed for normal operation
#define BURST_DUTY_CYCLE 11250      // ~90% speed for initial burst
#define HIGH_POWER_DUTY 7500        // ~60% speed for secondary phase
#define BURST_DURATION_MS 500       // Initial burst duration
#define HIGH_POWER_DURATION_MS 200  // Secondary phase duration
#define TARGET_DISTANCE 10.0        // Target distance in cm
#define START_CONTROL_DISTANCE 15.0 // Start PID control distance
#define STOP_TOLERANCE 1.0          // Stop within 1cm of target

// PID controller structure
typedef struct
{
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float previous_error;
    float duty_cycle;
} PIDController;

void init_motors();
void pid_init(PIDController *pid, float kp, float ki, float kd,
              float setpoint, float output_min, float output_max);
float pid_update(PIDController *pid, float measurement);
float pid_compute(PIDController *pid, float measurement);
void pid_reset(PIDController *pid);

void move_forward(float duty_cycle_motor1, float duty_cycle_motor2);
void move_backward(float duty_cycle_motor1, float duty_cycle_motor2);
void turn_left(float duty_cycle_motor1, float duty_cycle_motor2);
void turn_right(float duty_cycle_motor1, float duty_cycle_motor2);
void strong_start(int direction);
void stop_motors();

void set_motor_speed(uint slice_num, float duty_cycle);
void set_direction(int direction, uint in1_pin, uint in2_pin);

void motor1_forward(float duty_cycle);
void motor1_backward(float duty_cycle);
void motor2_forward(float duty_cycle);
void motor2_backward(float duty_cycle);

#endif // PICO_MOTOR_H