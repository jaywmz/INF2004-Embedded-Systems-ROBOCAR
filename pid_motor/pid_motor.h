#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <string.h>

// Define pins for Motor 1
#define MOTOR1_PWM_PIN 2
#define MOTOR1_IN1_PIN 0
#define MOTOR1_IN2_PIN 1

// Define pins for Motor 2
#define MOTOR2_PWM_PIN 6
#define MOTOR2_IN1_PIN 4
#define MOTOR2_IN2_PIN 5

// Speed settings
#define MIN_DUTY_CYCLE 2500
#define MAX_DUTY_CYCLE 12500
#define NORMAL_DUTY_CYCLE 4375      // ~35% speed for normal operation
#define BURST_DUTY_CYCLE 11250      // ~90% speed for initial burst
#define HIGH_POWER_DUTY 7500        // ~60% speed for secondary phase
#define BURST_DURATION_MS 500       // Initial burst duration
#define HIGH_POWER_DURATION_MS 200  // Secondary phase duration
#define TARGET_DISTANCE 10.0        // Target distance in cm
#define START_CONTROL_DISTANCE 15.0 // Start PID control distance
#define STOP_TOLERANCE 1.0          // Stop within 1cm of target

// PID controller structure
typedef struct {
    double kp;
    double ki;
    double kd;
    double setpoint;
    double integral;
    double prev_error;
    double output_min;
    double output_max;
    uint32_t last_time;
} PIDController;

// Variables and PID controller
extern int duty_cycle_motor1;
extern int duty_cycle_motor2;
extern PIDController distance_pid;

// Function declarations
void setup_pwm(uint gpio_pin);
void setup_motor_pins(uint in1_pin, uint in2_pin);
void rotate_motor(const char *direction, uint in1_pin, uint in2_pin);
void start_motor(uint slice_num, int duty_cycle);
void stop_motor(uint slice_num);
void set_motor_speed(uint slice_num, int duty_cycle);
void rotate_both_motors(const char *direction, int duty_cycle_motor1, int duty_cycle_motor2);
void strong_start(const char *direction);

// PID control functions
void pid_init(PIDController *pid, double kp, double ki, double kd, 
              double setpoint, double output_min, double output_max);
double pid_update(PIDController *pid, double measurement);
void pid_reset(PIDController *pid);

#endif