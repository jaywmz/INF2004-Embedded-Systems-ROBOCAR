#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"
#include <stdlib.h>

// Define pins for ultrasonic sensor
#define TRIGPIN 9
#define ECHOPIN 8

// Kalman filter structure
typedef struct
{
    double q; // Process noise covariance
    double r; // Measurement noise covariance
    double x; // Estimated value
    double p; // Estimation error covariance
    double k; // Kalman gain
} KalmanState;

// External variables
extern volatile bool obstacleDetected;
extern volatile uint64_t latest_pulse_width;

// Function declarations
KalmanState *kalman_init(double q, double r, double p, double initial_value);
void kalman_update(KalmanState *state, double measurement);
void init_ultrasonic(void);
void echo_isr(uint gpio, uint32_t events);
uint64_t getPulse(void);
double getCm(KalmanState *state);
void ultrasonic_task(void *pvParameters);

double get_cm(KalmanState *state, uint64_t pulse_length);
void set_trigger_pin(int value);
uint64_t get_pulse_length(void);

#endif