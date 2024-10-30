// wheel_encoder.h

#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

// Encoder and Motor Configuration
#define LEFT_ENCODER_PIN 27
#define RIGHT_ENCODER_PIN 3

// Wheel and Encoder Measurements
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065
#define WHEEL_CIRCUMFERENCE 0.2042
#define DISTANCE_PER_NOTCH 0.01021
#define NO_PULSE_TIMEOUT_MS 1000

// Define a data structure for encoder data
typedef struct {
    uint32_t pulse_count;
    float speed_m_per_s;
    float distance_m;
    uint64_t pulse_width_us;  // Pulse width in microseconds
} EncoderData;

// External declarations for message buffer handles and task handles
extern MessageBufferHandle_t left_buffer;
extern MessageBufferHandle_t right_buffer;
extern TaskHandle_t left_encoder_task_handle;
extern TaskHandle_t right_encoder_task_handle;

// Function declarations
void left_encoder_callback(uint gpio, uint32_t events);
void right_encoder_callback(uint gpio, uint32_t events);
void process_left_encoder_task(void *pvParameters);
void process_right_encoder_task(void *pvParameters);
void log_encoder_data(const char *wheel, EncoderData *data);  // New log function for printing encoder data

#endif // WHEEL_ENCODER_H
