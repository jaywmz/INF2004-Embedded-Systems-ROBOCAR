#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>
#include "pico/types.h"

// Encoder and Motor Configuration
#define LEFT_ENCODER_PIN 3
#define RIGHT_ENCODER_PIN 7

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
    float speed_cm_per_s;
    uint64_t pulse_width;
    int pulse_count;
    uint64_t _last_pulse_time; // Internal variable to keep track of the last pulse time
    int _last_pin_state;       // Internal variable to keep track of the last pin state
    EncoderKalmanState kalman_state;
} EncoderData;

void init_encoder(void);                                             // Function to initialize encoder settings
void read_encoder_data(uint encoder_pin, EncoderData *encoder_data); // Function to read encoder data

void encoder_kalman_init(EncoderKalmanState *state, float q, float r, float p, float initial_value);
void encoder_kalman_update(EncoderKalmanState *state, float measurement);

#endif // WHEEL_ENCODER_H