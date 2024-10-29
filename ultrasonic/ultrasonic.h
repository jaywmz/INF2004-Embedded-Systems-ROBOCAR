// ultrasonic.h

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>

// Define pins for ultrasonic sensor
#define TRIGPIN 1
#define ECHOPIN 0

// Kalman filter structure
typedef struct kalman_state_
{
    double q;  // Process noise covariance
    double r;  // Measurement noise covariance
    double x;  // Estimated value
    double p;  // Estimation error covariance
    double k;  // Kalman gain
} kalman_state;

// External variables
extern volatile bool obstacleDetected;

// Function declarations
kalman_state *kalman_init(double q, double r, double p, double initial_value);
void kalman_update(kalman_state *state, double measurement);
void setupUltrasonicPins();
uint64_t getPulse();
double getCm(kalman_state *state);
void ultrasonic_task(void *pvParameters);

#endif // ULTRASONIC_H
