#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>

// Define pins for ultrasonic sensor
#define TRIGPIN 9
#define ECHOPIN 8

// Kalman filter structure
typedef struct kalman_state_ {
    double q;  // Process noise covariance
    double r;  // Measurement noise covariance
    double x;  // Estimated value
    double p;  // Estimation error covariance
    double k;  // Kalman gain
} kalman_state;

// External variables
extern volatile bool obstacleDetected;
extern volatile uint64_t latest_pulse_width;

// Function declarations
kalman_state *kalman_init(double q, double r, double p, double initial_value);
void kalman_update(kalman_state *state, double measurement);
void setupUltrasonicPins(void);
void echo_isr(uint gpio, uint32_t events);
uint64_t getPulse(void);
double getCm(kalman_state *state);
void ultrasonic_task(void *pvParameters);

#endif