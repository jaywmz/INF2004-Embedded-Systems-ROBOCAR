#include "encoder.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include <stdio.h>

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

    // Convert to linear speed (distance per second) in cm per second
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
            encoder_data->speed = encoder_data->kalman_state.x;
            // encoder_data->speed = speed;     // Doesn't this override the kalman filter?
            encoder_data->pulse_count++;
        }

        encoder_data->_last_pulse_time = current_time;
    }
    encoder_data->_last_pin_state = current_state;
}
// USED in car.c - This function is called in `vTaskEncoder` to continuously read encoder data, calculate speed, and update pulse counts.

// Interrupt callbacks for encoders
// void encoder_isr_motor1(uint gpio, uint32_t events) {
//     motor1_encoder_data.pulse_count++;
// }

// void encoder_isr_motor2(uint gpio, uint32_t events) {
//     motor2_encoder_data.pulse_count++;
// }

void shared_encoder_isr(uint gpio, uint32_t events) {
    if (gpio == MOTOR1_ENCODER_PIN) {
        motor1_encoder_data.pulse_count++;
    } else if (gpio == MOTOR2_ENCODER_PIN) {
        motor2_encoder_data.pulse_count++;
    }
}

// Initializes the GPIO pins for the left and right encoders
void init_encoder()
{
    // Initialize GPIO pins for left and right encoders
    gpio_init(MOTOR2_ENCODER_PIN);
    gpio_set_dir(MOTOR2_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(MOTOR2_ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(MOTOR2_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &shared_encoder_isr);

    gpio_init(MOTOR1_ENCODER_PIN);
    gpio_set_dir(MOTOR1_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(MOTOR1_ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(MOTOR1_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &shared_encoder_isr);
}
// USED in car.c - This function is called in `main` to set up the GPIO pins for the encoders.
