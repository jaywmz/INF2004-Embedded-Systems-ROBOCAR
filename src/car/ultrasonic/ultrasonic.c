#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <stdlib.h>

const int timeout = 26000;               // Timeout for ultrasonic sensor (about 4.5 meters)
volatile absolute_time_t start_time = 0; // Start time for echo pulse

// Function to trigger the HC-SR04 and measure the pulse length on the ECHO pin
uint64_t measure_pulse_length()
{
    // Send a 10 µs pulse to TRIG to start measurement
    gpio_put(TRIGPIN, 1);
    sleep_us(10);         // Wait for 10 microseconds
    gpio_put(TRIGPIN, 0); // Set TRIG low

    // Wait for the ECHO pin to go high
    while (gpio_get(ECHOPIN) == 0)
        ;

    // Measure the pulse width on the ECHO pin
    uint64_t start_time = time_us_64(); // Record the start time
    while (gpio_get(ECHOPIN) == 1)
        ;
    uint64_t end_time = time_us_64(); // Record the end time

    // Calculate the pulse length in microseconds
    uint64_t pulse_length = end_time - start_time;
    return pulse_length;
}

void set_trigger_pin(int value)
{
    gpio_put(TRIGPIN, value);
}

uint64_t get_pulse_length()
{
    // Wait for the ECHO pin to go high
    while (gpio_get(ECHOPIN) == 0)
        ;

    // Measure the pulse width on the ECHO pin
    uint64_t start_time = time_us_64(); // Record the start time
    while (gpio_get(ECHOPIN) == 1)
        ;
    uint64_t end_time = time_us_64(); // Record the end time

    // Calculate the pulse length in microseconds
    uint64_t pulse_length = end_time - start_time;
    return pulse_length;
}

// Function to calculate distance in centimeters
float calculate_distance(uint64_t pulse_length)
{
    // Speed of sound is approximately 343 meters per second
    // Divide by 2 to account for the round trip
    return (pulse_length / 58.0); // Distance in cm
}

KalmanState *kalman_init(double q, double r, double p, double initial_value)
{
    KalmanState *state = calloc(1, sizeof(KalmanState));
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;
    return state;
}

void kalman_update(KalmanState *state, double measurement)
{
    state->p = state->p + state->q;
    state->k = state->p / (state->p + state->r);
    state->x = state->x + state->k * (measurement - state->x);
    state->p = (1 - state->k) * state->p;
}

void init_ultrasonic()
{
    // Initialize TRIG pin as output
    gpio_init(TRIGPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_put(TRIGPIN, 0); // Ensure TRIG starts low

    // Initialize ECHO pin as input
    gpio_init(ECHOPIN);
    gpio_set_dir(ECHOPIN, GPIO_IN);
    gpio_pull_down(ECHOPIN); // Add pull-down resistor

    printf("Ultrasonic pins configured - TRIG: %d, ECHO: %d\n", TRIGPIN, ECHOPIN);
}

double getCm(KalmanState *state)
{
    // Measure the pulse length from the ECHO pin
    uint64_t pulse_length = measure_pulse_length();

    if (pulse_length == 0 || pulse_length > timeout)
    {
        printf("Error: Pulse timeout or out of range.\n");
        return 0;
    }

    // Calculate the distance based on pulse length
    float distance = calculate_distance(pulse_length);

    if (distance < 2 || distance > 400)
    {
        printf("Measured distance out of range: %.2f cm\n", distance);
        return 0;
    }

    kalman_update(state, distance);

    // Print the pulse length and calculated distance
    // printf("Pulse Length: %llu us\nDistance: %.2f cm\nFiltered Distance: %.2f cm\n", pulse_length, distance, state->x);
    return state->x;
}

double get_cm(KalmanState *state, uint64_t pulse_length)
{
    if (pulse_length == 0 || pulse_length > timeout)
    {
        printf("Error: Pulse timeout or out of range.\n");
        return 10 + 1;
    }

    // Calculate the distance based on pulse length
    float distance = calculate_distance(pulse_length);

    if (distance < 2 || distance > 400)
    {
        printf("Measured distance out of range: %.2f cm\n", distance);
        return 10 + 1;
    }

    kalman_update(state, distance);

    // Print the pulse length and calculated distance
    printf("Pulse Length: %llu us\nDistance: %.2f cm\nFiltered Distance: %.2f cm\n", pulse_length, distance, state->x);
    return state->x;
}