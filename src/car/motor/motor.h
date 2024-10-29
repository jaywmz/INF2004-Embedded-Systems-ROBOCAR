#ifndef PICO_MOTOR_H
#define PICO_MOTOR_H

#include "hardware/pwm.h"
#include "pico/stdlib.h"

// Define pins for Motor 1 (same as the current setup)
#define MOTOR1_PWM_PIN 2 // PWM-capable GPIO pin for motor 1 speed control (ENA)
#define MOTOR1_IN1_PIN 0 // GPIO pin for motor 1 direction control (IN1)
#define MOTOR1_IN2_PIN 1 // GPIO pin for motor 1 direction control (IN2)

// Define pins for Motor 2 (new setup)
#define MOTOR2_PWM_PIN 6 // PWM-capable GPIO pin for motor 2 speed control (ENA)
#define MOTOR2_IN1_PIN 4 // GPIO pin for motor 2 direction control (IN1)
#define MOTOR2_IN2_PIN 5 // GPIO pin for motor 2 direction control (IN2)

void init_motors();

void move_forward();
void move_backward();
void turn_left();
void turn_right();
void stop_motors();
void set_motor_speed(uint slice_num, int duty_cycle);

#endif // PICO_MOTOR_H