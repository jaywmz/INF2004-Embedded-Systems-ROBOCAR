#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>
#include "pico/types.h"

// Encoder and Motor Configuration
#define MOTOR2_ENCODER_PIN 3
#define MOTOR1_ENCODER_PIN 7

// Wheel and Encoder Measurements
#define ENCODER_NOTCHES_PER_REV 20
#define WHEEL_DIAMETER 0.065                                               // in meters
#define WHEEL_DIAMETER_CM 6.5
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14159265358979323846)      // calculated as π * diameter
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159265358979323846)
#define DISTANCE_PER_NOTCH (WHEEL_CIRCUMFERENCE / ENCODER_NOTCHES_PER_REV) // in meters
#define ENCODER_TIMEOUT_INTERVAL 1000000                                   // Timeout in microseconds (1 second)
#define MICROSECONDS_IN_A_SECOND 1000000.0f
#define SAMPLE_INTERVAL_MS 200 // Sampling interval in milliseconds

typedef struct
{
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float p; // Estimation error covariance
    float k; // Kalman gain
    float x; // Estimated value
} EncoderKalmanState;

// Define a data structure for encoder data
typedef struct
{
    float speed;
    uint64_t pulse_width;
    int pulse_count;
    uint64_t _last_pulse_time; // Internal variable to keep track of the last pulse time
    int _last_pin_state;       // Internal variable to keep track of the last pin state
    EncoderKalmanState kalman_state;
} EncoderData;

extern EncoderData motor1_encoder_data;
extern EncoderData motor2_encoder_data;

void init_encoder(void);                                             // Function to initialize encoder settings
void read_encoder_data(uint encoder_pin, EncoderData *encoder_data); // Function to read encoder data

void encoder_task(void *pvParameters);
// void encoder_isr_motor1(uint gpio, uint32_t events);
// void encoder_isr_motor2(uint gpio, uint32_t events);
void shared_encoder_isr(uint gpio, uint32_t events);

void encoder_kalman_init(EncoderKalmanState *state, float q, float r, float p, float initial_value);
void encoder_kalman_update(EncoderKalmanState *state, float measurement);

#endif // WHEEL_ENCODER_H