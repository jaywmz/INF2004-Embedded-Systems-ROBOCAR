#ifndef WHEEL_ULTRA_H
#define WHEEL_ULTRA_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include <stdio.h>
#include <stdlib.h>

// Pin Definitions
#define TRIGPIN 1
#define ECHOPIN 0
#define LEFT_ENCODER_PIN 8
#define RIGHT_ENCODER_PIN 26

// Ultrasonic and Encoder Configuration
#define ULTRASONIC_TIMEOUT 26000  // Timeout for ultrasonic sensor
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065  // in meters
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14159265358979323846)
#define DISTANCE_PER_NOTCH (WHEEL_CIRCUMFERENCE / ENCODER_NOTCHES_PER_REV)
#define MICROSECONDS_IN_A_SECOND 1000000.0f
#define ENCODER_TIMEOUT_INTERVAL 1000000  // Timeout in microseconds (1 second)

// Global Variables (extern)
extern volatile absolute_time_t start_time;
extern volatile uint64_t latest_pulse_width;
extern volatile bool obstacleDetected;

// Message Buffers and Task Handles (extern)
extern MessageBufferHandle_t left_buffer;
extern MessageBufferHandle_t right_buffer;
extern TaskHandle_t left_encoder_task_handle;
extern TaskHandle_t right_encoder_task_handle;

// Kalman Filter Structure
typedef struct {
    double q;  // Process noise covariance
    double r;  // Measurement noise covariance
    double x;  // Estimated value
    double p;  // Estimation error covariance
    double k;  // Kalman gain
} kalman_state;

// Encoder Data Structure
typedef struct {
    uint32_t pulse_count;
    float speed_m_per_s;
    float distance_m;
    uint64_t pulse_width_us;
} EncoderData;

// Kalman Filter Functions
kalman_state *kalman_init(double q, double r, double p, double initial_value);
void kalman_update(kalman_state *state, double measurement);

// Ultrasonic Sensor Functions
void setupUltrasonicPins();
uint64_t getPulse();
double getCm(kalman_state *state);

// Encoder Functions
void log_encoder_data(const char *wheel, EncoderData *data);

// ISR for Ultrasonic and Encoder Sensors
void unified_gpio_callback(uint gpio, uint32_t events);

// Tasks for Processing Encoder Data
void process_left_encoder_task(void *pvParameters);
void process_right_encoder_task(void *pvParameters);
void ultrasonic_task(void *pvParameters);

// System Initialization Function
void system_init();

#endif
