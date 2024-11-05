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
#define TRIGPIN 9
#define ECHOPIN 8
#define LEFT_ENCODER_PIN 27
#define RIGHT_ENCODER_PIN 3

// Ultrasonic and Encoder Configuration
#define ULTRASONIC_TIMEOUT 26000
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14159265358979323846)
#define DISTANCE_PER_NOTCH (WHEEL_CIRCUMFERENCE / ENCODER_NOTCHES_PER_REV)
#define MICROSECONDS_IN_A_SECOND 1000000.0f
#define ENCODER_TIMEOUT_INTERVAL 1000000

// Global Variables (extern)
extern volatile absolute_time_t start_time;
extern volatile uint64_t latest_pulse_width;
extern volatile bool obstacleDetected;

extern MessageBufferHandle_t left_buffer;
extern MessageBufferHandle_t right_buffer;
extern TaskHandle_t left_encoder_task_handle;
extern TaskHandle_t right_encoder_task_handle;

extern volatile uint32_t left_pulse_count;
extern volatile uint32_t right_pulse_count;


typedef struct {
    double q;
    double r;
    double x;
    double p;
    double k;
} kalman_state;

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
void reset_encoders();

#endif