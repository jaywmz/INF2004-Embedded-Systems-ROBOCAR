#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"
#include <stdio.h>

// Encoder and Motor Configuration
#define LEFT_ENCODER_PIN 8
#define RIGHT_ENCODER_PIN 26

// Wheel and Encoder Measurements
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065  // in meters
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14159265358979323846) // calculated as π * diameter
#define DISTANCE_PER_NOTCH (WHEEL_CIRCUMFERENCE / ENCODER_NOTCHES_PER_REV)  // in meters
#define ENCODER_TIMEOUT_INTERVAL 1000000  // Timeout in microseconds (1 second)
#define MICROSECONDS_IN_A_SECOND 1000000.0f

// Define a data structure for encoder data
typedef struct {
    uint32_t pulse_count;
    float speed_m_per_s;
    float distance_m;
    uint64_t pulse_width_us;
} EncoderData;

// External declarations for message buffer handles and task handles
extern MessageBufferHandle_t left_buffer;
extern MessageBufferHandle_t right_buffer;
extern TaskHandle_t left_encoder_task_handle;
extern TaskHandle_t right_encoder_task_handle;

// Function declarations
void encoder_gpio_callback(uint gpio, uint32_t events);      // Unified callback for encoder interrupts
void process_left_encoder_task(void *pvParameters);          // Task to process left encoder data
void process_right_encoder_task(void *pvParameters);         // Task to process right encoder data
void log_encoder_data(const char *wheel, EncoderData *data); // Function to log encoder data for each wheel
void encoder_init(void);                                     // Function to initialize encoder settings

#endif // WHEEL_ENCODER_H
