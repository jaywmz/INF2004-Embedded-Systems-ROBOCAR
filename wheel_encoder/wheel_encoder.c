#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include <stdio.h>

// Encoder and Motor Configuration
#define LEFT_ENCODER_PIN 2        // Left encoder signal input
#define RIGHT_ENCODER_PIN 3       // Right encoder signal input
#define MOTOR_PWM_PIN 15          // Motor PWM output pin

// Wheel and Encoder Measurements
#define ENCODER_NOTCHES_PER_REV 20    // Number of notches (slots) on the encoder disk
#define WHEEL_DIAMETER 0.065          // Diameter of the wheel in meters (65 mm)
#define WHEEL_CIRCUMFERENCE 0.2042    // Circumference of the wheel in meters
#define DISTANCE_PER_NOTCH 0.01021    // Distance traveled per notch
#define NO_PULSE_TIMEOUT_MS 1000      // Timeout for detecting zero speed in ms

// Define a data structure for encoder data
typedef struct {
    uint32_t pulse_count;
    float speed_m_per_s;
    float distance_m;
} EncoderData;

// Message buffer handles for each encoder
MessageBufferHandle_t left_buffer, right_buffer;

// Left encoder ISR
void left_encoder_callback(uint gpio, uint32_t events) {
    if (gpio == LEFT_ENCODER_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        static uint32_t left_pulse_count = 0;
        static float left_total_distance = 0.0;
        static uint64_t last_left_pulse_time = 0;

        uint64_t current_time = time_us_64();
        uint64_t pulse_duration_us = current_time - last_left_pulse_time;
        last_left_pulse_time = current_time;

        left_pulse_count++;

        float left_speed = 0.0;
        if (pulse_duration_us > 0 && pulse_duration_us < 1000000) {  // Filter out long durations
            float pulse_duration_s = pulse_duration_us / 1000000.0f;
            left_speed = DISTANCE_PER_NOTCH / pulse_duration_s;
        }

        left_total_distance += DISTANCE_PER_NOTCH;

        // Package data into the structure
        EncoderData data = {left_pulse_count, left_speed, left_total_distance};

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
        uint64_t pulse_duration_us = current_time - last_right_pulse_time;
        last_right_pulse_time = current_time;

        right_pulse_count++;

        float right_speed = 0.0;
        if (pulse_duration_us > 0 && pulse_duration_us < 1000000) {  // Filter out long durations
            float pulse_duration_s = pulse_duration_us / 1000000.0f;
            right_speed = DISTANCE_PER_NOTCH / pulse_duration_s;
        }

        right_total_distance += DISTANCE_PER_NOTCH;

        // Package data into the structure
        EncoderData data = {right_pulse_count, right_speed, right_total_distance};

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
            // Process data (e.g., print it)
            printf("Left Wheel -> Pulses: %d, Speed: %.2f m/s, Total Distance: %.2f meters\n",
                   data.pulse_count, data.speed_m_per_s, data.distance_m);
        }
    }
}

// Task to process right encoder data
void process_right_encoder_task(void *pvParameters) {
    EncoderData data;
    while (1) {
        // Wait to receive data from the right buffer
        if (xMessageBufferReceive(right_buffer, &data, sizeof(data), portMAX_DELAY) > 0) {
            // Process data (e.g., print it)
            printf("Right Wheel -> Pulses: %d, Speed: %.2f m/s, Total Distance: %.2f meters\n",
                   data.pulse_count, data.speed_m_per_s, data.distance_m);
        }
    }
}

// PWM setup for motor control
void setup_pwm() {
    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    pwm_set_wrap(slice_num, 12500);  // 100Hz frequency
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 6250);  // 50% duty cycle
    pwm_set_enabled(slice_num, true);
}

// Function to adjust motor speed dynamically
void set_motor_speed(float duty_cycle) {
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    if (duty_cycle > 100.0f) duty_cycle = 100.0f;
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    uint16_t level = (uint16_t)(duty_cycle * 12500 / 100.0f);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, level);
}

int main() {
    stdio_init_all();

    // Initialize the encoder pins
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_down(LEFT_ENCODER_PIN);

    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_down(RIGHT_ENCODER_PIN);

    // Create message buffers (size: 10 messages)
    left_buffer = xMessageBufferCreate(10 * sizeof(EncoderData));
    right_buffer = xMessageBufferCreate(10 * sizeof(EncoderData));

    if (left_buffer == NULL || right_buffer == NULL) {
        printf("Failed to create message buffers\n");
        return 1;
    }

    // Set interrupts for both encoders
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &left_encoder_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &right_encoder_callback);

    // Set up PWM for motor control
    setup_pwm();

    // Create tasks for each wheel's speed and distance processing
    xTaskCreate(process_left_encoder_task, "LeftEncoderTask", 1024, NULL, 1, NULL);
    xTaskCreate(process_right_encoder_task, "RightEncoderTask", 1024, NULL, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (true) {
        tight_loop_contents();
    }

    return 0;
}
