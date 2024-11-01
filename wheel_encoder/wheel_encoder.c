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

// Unified interrupt callback
void encoder_gpio_callback(uint gpio, uint32_t events) {
    uint64_t current_time = time_us_64();

    if (gpio == LEFT_ENCODER_PIN) {
        static uint32_t left_pulse_count = 0;
        static float left_total_distance = 0.0;
        static uint64_t last_left_pulse_time = 0;

        if (events & GPIO_IRQ_EDGE_RISE) {
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
            if (xMessageBufferSendFromISR(left_buffer, &data, sizeof(data), NULL) == pdFALSE) {
                printf("Error: Left buffer is full\n");
            }
        }
    } else if (gpio == RIGHT_ENCODER_PIN) {
        static uint32_t right_pulse_count = 0;
        static float right_total_distance = 0.0;
        static uint64_t last_right_pulse_time = 0;

        if (events & GPIO_IRQ_EDGE_RISE) {
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
            if (xMessageBufferSendFromISR(right_buffer, &data, sizeof(data), NULL) == pdFALSE) {
                printf("Error: Right buffer is full\n");
            }
        }
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

// Initialization function for encoders and GPIO setup
void encoder_init() {
    // Create message buffers
    left_buffer = xMessageBufferCreate(256);
    right_buffer = xMessageBufferCreate(256);

    if (left_buffer == NULL || right_buffer == NULL) {
        printf("Error: Message buffer allocation failed\n");
        return;
    }

    // Initialize GPIO pins for left and right encoders
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_gpio_callback);

    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_gpio_callback);

    // Create tasks to process encoder data
    BaseType_t left_task_status = xTaskCreate(process_left_encoder_task, "Process Left Encoder Task", 1024, NULL, 1, &left_encoder_task_handle);
    BaseType_t right_task_status = xTaskCreate(process_right_encoder_task, "Process Right Encoder Task", 1024, NULL, 1, &right_encoder_task_handle);

    // Check if tasks were created successfully
    if (left_task_status != pdPASS || right_task_status != pdPASS) {
        printf("Error: Task creation failed\n");
        return;
    }
}
