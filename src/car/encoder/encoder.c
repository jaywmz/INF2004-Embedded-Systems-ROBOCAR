#include "encoder.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include <stdio.h>

// Wheel and Encoder Measurements
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065                                               // in meters
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14159265358979323846)      // calculated as π * diameter
#define DISTANCE_PER_NOTCH (WHEEL_CIRCUMFERENCE / ENCODER_NOTCHES_PER_REV) // in meters
#define ENCODER_TIMEOUT_INTERVAL 1000000                                   // Timeout in microseconds (1 second)
#define MICROSECONDS_IN_A_SECOND 1000000.0f

// Debounce time in microseconds
// #define DEBOUNCE_TIME_US 1000 // Adjust this value as needed

// Function to calculate speed based on pulse width
static float calculate_speed(uint64_t pulse_width)
{
    if (pulse_width == 0)
    {
        return 0.0; // Avoid division by zero for the first pulse
    }

    // Calculate the time for one rotation based on pulse width and PPR
    float rotation_time = (pulse_width * ENCODER_NOTCHES_PER_REV) / 1e6; // Convert to seconds
    float rotations_per_second = 1.0 / rotation_time;                    // Rotations per second

    // Convert to linear speed (distance per second) in meters per second
    return rotations_per_second * WHEEL_CIRCUMFERENCE;
}

void read_encoder_data(uint encoder_pin, EncoderData *encoder_data)
{
    uint64_t current_time = time_us_64();

    bool current_state = gpio_get(encoder_pin);
    if (current_state != encoder_data->_last_pin_state)
    {

        uint64_t pulse_width = current_time - encoder_data->_last_pulse_time;

        // I only care about rising edges
        if (current_state == 1 && pulse_width > 0 && pulse_width < 1000000)
        {
            encoder_data->pulse_width = pulse_width;
            float speed = calculate_speed(pulse_width);
            encoder_data->speed_m_per_s = speed;
            printf("Speed: %.2f m/s, Pulse Width: %lluus\n ", speed,
                   pulse_width);
        }

        encoder_data->_last_pulse_time = current_time;
    }
    encoder_data->_last_pin_state = current_state;
}

// Initialization function for encoders and GPIO setup
void init_encoder()
{
    // Initialize GPIO pins for left and right encoders
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_PIN);

    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_PIN);
}