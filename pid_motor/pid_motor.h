#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Define pins for Motor 1
#define MOTOR1_PWM_PIN 10  // PWM-capable GPIO pin for motor 1 speed control (ENA)
#define MOTOR1_IN1_PIN 12  // GPIO pin for motor 1 direction control (IN1)
#define MOTOR1_IN2_PIN 13  // GPIO pin for motor 1 direction control (IN2)

// Define pins for Motor 2
#define MOTOR2_PWM_PIN 11  // PWM-capable GPIO pin for motor 2 speed control (ENA)
#define MOTOR2_IN1_PIN 14  // GPIO pin for motor 2 direction control (IN1)
#define MOTOR2_IN2_PIN 15  // GPIO pin for motor 2 direction control (IN2)

// Duty cycle variables for motor control
extern int duty_cycle_motor1;
extern int duty_cycle_motor2;

// PWM and motor control function declarations
void setup_pwm(uint gpio_pin);
void setup_motor_pins(uint in1_pin, uint in2_pin);
void rotate_motor(char *direction, uint in1_pin, uint in2_pin);
void start_motor(uint slice_num, int duty_cycle);
void stop_motor(uint slice_num);
void set_motor_speed(uint slice_num, int duty_cycle);
void rotate_both_motors(char *direction, int duty_cycle_motor1, int duty_cycle_motor2);

#endif // PID_MOTOR_H
