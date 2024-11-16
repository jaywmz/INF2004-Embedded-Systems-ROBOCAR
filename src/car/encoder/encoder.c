#include "encoder.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include <stdio.h>

// Wheel and Encoder Measurements
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065                                               // in meters
#define WHEEL_DIAMETER_CM 6.5
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14159265358979323846)      // calculated as π * diameter
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159265358979323846)
#define DISTANCE_PER_NOTCH (WHEEL_CIRCUMFERENCE / ENCODER_NOTCHES_PER_REV) // in meters
#define ENCODER_TIMEOUT_INTERVAL 1000000                                   // Timeout in microseconds (1 second)
#define MICROSECONDS_IN_A_SECOND 1000000.0f

// Debounce time in microseconds
// #define DEBOUNCE_TIME_US 1000 // Adjust this value as needed

typedef struct
{
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float p; // Estimation error covariance
    float k; // Kalman gain
    float x; // Estimated value
} KalmanState;

// Initializes the Kalman filter state with given parameters
void encoder_kalman_init(EncoderKalmanState *state, float q, float r, float p, float initial_value)
{
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;
    state->k = 0.0;
}
// USED in car.c - This function is called in `vTaskEncoder` to initialize the Kalman filter state for each encoder.

// Updates the Kalman filter with a new speed measurement
void encoder_kalman_update(EncoderKalmanState *state, float measurement)
{
    // Prediction update
    state->p += state->q;

    // Measurement update
    state->k = state->p / (state->p + state->r);
    state->x += state->k * (measurement - state->x);
    state->p *= (1 - state->k);
}
// USED indirectly in car.c - This function is called within `read_encoder_data` to update the Kalman filter with new speed measurements.

// Calculates the speed of the wheel based on the pulse width (time between encoder notches)
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
    return rotations_per_second * WHEEL_CIRCUMFERENCE_CM;
}
// NOT USED directly in car.c - This function is called within `read_encoder_data` to compute speed, but not directly in `car.c`.

// Reads data from the encoder pin, calculates the speed, and applies Kalman filtering
void read_encoder_data(uint encoder_pin, EncoderData *encoder_data)
{
    uint64_t current_time = time_us_64();

    bool current_state = gpio_get(encoder_pin);
    if (current_state != encoder_data->_last_pin_state)
    {
        uint64_t pulse_width = current_time - encoder_data->_last_pulse_time;

        // Only track rising edges of the encoder signal
        if (current_state == 1 && pulse_width > 0 && pulse_width < 1000000)
        {
            encoder_data->pulse_width = pulse_width;
            float speed = calculate_speed(pulse_width); // Calls `calculate_speed` to get speed
            encoder_kalman_update(&encoder_data->kalman_state, speed); // Updates speed using Kalman filter
            encoder_data->speed_cm_per_s = encoder_data->kalman_state.x;
            encoder_data->pulse_count++;
            if (encoder_pin == LEFT_ENCODER_PIN)
            {
                // Debugging output for the left encoder (commented out)
                // printf("Left Speed: %.2f m/s, Pulse Width: %lluus, Filtered Speed: %.2f m/s\n", speed,
                //        pulse_width, encoder_data->speed_cm_per_s);
            }
            else
            {
                // Debugging output for the right encoder (commented out)
                // printf("Right ");
                // printf("Speed: %.2f m/s, Pulse Width: %lluus\n ", speed,
                //        pulse_width);
            }
        }

        encoder_data->_last_pulse_time = current_time;
    }
    encoder_data->_last_pin_state = current_state;
}
// USED in car.c - This function is called in `vTaskEncoder` to continuously read encoder data, calculate speed, and update pulse counts.

// Initializes the GPIO pins for the left and right encoders
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
// USED in car.c - This function is called in `main` to set up the GPIO pins for the encoders.
