#include "wheel_encoder.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Global definitions of message buffer handles and task handles
MessageBufferHandle_t left_buffer = NULL;
MessageBufferHandle_t right_buffer = NULL;
TaskHandle_t left_encoder_task_handle = NULL;
TaskHandle_t right_encoder_task_handle = NULL;

// Log function to print encoder data
void log_encoder_data(const char *wheel, EncoderData *data) {
    printf("%s Wheel -> Pulses: %d, Speed: %.2f m/s, Total Distance: %.2f meters, Pulse Width: %llu us\n",
           wheel, data->pulse_count, data->speed_m_per_s, data->distance_m, data->pulse_width_us);
}

// Left encoder ISR
void left_encoder_callback(uint gpio, uint32_t events) {
    if (gpio == LEFT_ENCODER_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        static uint32_t left_pulse_count = 0;
        static float left_total_distance = 0.0;
        static uint64_t last_left_pulse_time = 0;

        uint64_t current_time = time_us_64();
        uint64_t pulse_width_us = current_time - last_left_pulse_time;
        last_left_pulse_time = current_time;

        left_pulse_count++;

        float left_speed = 0.0;
        if (pulse_width_us > 0 && pulse_width_us < 1000000) {
            float pulse_duration_s = pulse_width_us / 1000000.0f;
            left_speed = DISTANCE_PER_NOTCH / pulse_duration_s;
        }

        left_total_distance += DISTANCE_PER_NOTCH;

        // Package data with pulse width into the structure
        EncoderData data = {left_pulse_count, left_speed, left_total_distance, pulse_width_us};

        // Send data to the left message buffer
        xMessageBufferSendFromISR(left_buffer, &data, sizeof(data), NULL);
    }
}

// Right encoder ISR
void right_encoder_callback(uint gpio, uint32_t events) {
    if (gpio == RIGHT_ENCODER_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        static uint32_t right_pulse_count = 0;
        static float right_total_distance = 0.0;
        static uint64_t last_right_pulse_time = 0;

        uint64_t current_time = time_us_64();
        uint64_t pulse_width_us = current_time - last_right_pulse_time;
        last_right_pulse_time = current_time;

        right_pulse_count++;

        float right_speed = 0.0;
        if (pulse_width_us > 0 && pulse_width_us < 1000000) {
            float pulse_duration_s = pulse_width_us / 1000000.0f;
            right_speed = DISTANCE_PER_NOTCH / pulse_duration_s;
        }

        right_total_distance += DISTANCE_PER_NOTCH;

        // Package data with pulse width into the structure
        EncoderData data = {right_pulse_count, right_speed, right_total_distance, pulse_width_us};

        // Send data to the right message buffer
        xMessageBufferSendFromISR(right_buffer, &data, sizeof(data), NULL);
    }
}

// Task to process left encoder data
void process_left_encoder_task(void *pvParameters) {
    EncoderData data;
    while (1) {
        // Wait to receive data from the left buffer
        if (xMessageBufferReceive(left_buffer, &data, sizeof(data), portMAX_DELAY) > 0) {
            // Log the encoder data for the left wheel
            log_encoder_data("Left", &data);
        }
    }
}

// Task to process right encoder data
void process_right_encoder_task(void *pvParameters) {
    EncoderData data;
    while (1) {
        // Wait to receive data from the right buffer
        if (xMessageBufferReceive(right_buffer, &data, sizeof(data), portMAX_DELAY) > 0) {
            // Log the encoder data for the right wheel
            log_encoder_data("Right", &data);
        }
    }
}
