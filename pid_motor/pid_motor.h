#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Define pins for Motor 1
#define MOTOR1_PWM_PIN 2
#define MOTOR1_IN1_PIN 0
#define MOTOR1_IN2_PIN 1

// Define pins for Motor 2
#define MOTOR2_PWM_PIN 6
#define MOTOR2_IN1_PIN 4
#define MOTOR2_IN2_PIN 5

#define MIN_DUTY_CYCLE 2500        // Minimum duty cycle
#define MAX_DUTY_CYCLE 12500       // Maximum possible duty cycle
#define NORMAL_DUTY_CYCLE 4375     // ~35% of max (12500 * 0.35 = 4375)
#define TARGET_DISTANCE 10.0       // Target distance in cm
#define START_CONTROL_DISTANCE 15.0 // Start PID control closer to target
#define STOP_TOLERANCE 1.0         // Stop within 1cm of target


// PID controller structure
typedef struct {
    double kp;           // Proportional gain
    double ki;           // Integral gain
    double kd;           // Derivative gain
    double setpoint;     // Target value
    double integral;     // Integral term
    double prev_error;   // Previous error for derivative term
    double output_min;   // Minimum output value
    double output_max;   // Maximum output value
    uint32_t last_time; // Last update time in milliseconds
} PIDController;

// Duty cycle variables for motor control
extern int duty_cycle_motor1;
extern int duty_cycle_motor2;
extern PIDController distance_pid;

// PWM and motor control function declarations
void setup_pwm(uint gpio_pin);
void setup_motor_pins(uint in1_pin, uint in2_pin);
void rotate_motor(const char *direction, uint in1_pin, uint in2_pin);  // Changed to const char*
void start_motor(uint slice_num, int duty_cycle);
void stop_motor(uint slice_num);
void set_motor_speed(uint slice_num, int duty_cycle);
void rotate_both_motors(const char *direction, int duty_cycle_motor1, int duty_cycle_motor2);  // Changed to const char*

// PID control functions
void pid_init(PIDController *pid, double kp, double ki, double kd, 
              double setpoint, double output_min, double output_max);
double pid_update(PIDController *pid, double measurement);
void pid_reset(PIDController *pid);
void adjust_speed_with_pid(double current_distance);

#endif // PID_MOTOR_H