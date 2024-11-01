#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>
#include "pico/types.h"

// Encoder and Motor Configuration
#define LEFT_ENCODER_PIN 7
#define RIGHT_ENCODER_PIN 3

// Define a data structure for encoder data
typedef struct
{
    float speed_m_per_s;
    uint64_t pulse_width;
    uint64_t _last_pulse_time; // Internal variable to keep track of the last pulse time
    int _last_pin_state;       // Internal variable to keep track of the last pin state
} EncoderData;

void init_encoder(void);                                             // Function to initialize encoder settings
void read_encoder_data(uint encoder_pin, EncoderData *encoder_data); // Function to read encoder data

#endif // WHEEL_ENCODER_H