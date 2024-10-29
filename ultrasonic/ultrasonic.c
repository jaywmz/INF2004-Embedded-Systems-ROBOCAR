// ultrasonic.c

#include "ultrasonic.h"

// Constants and global variables
const int timeout = 50000;  // Timeout for ultrasonic sensor (~8.6 meters)
volatile absolute_time_t start_time;  // Start time for echo pulse
volatile uint64_t pulse_width = 0;    // Pulse width in microseconds
volatile bool obstacleDetected = false;  // Flag to detect obstacles within 10 cm

// Initialize Kalman filter
kalman_state *kalman_init(double q, double r, double p, double initial_value)
{
    kalman_state *state = calloc(1, sizeof(kalman_state));
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;
    return state;
}

// Update Kalman filter with new measurement
void kalman_update(kalman_state *state, double measurement)
{
    state->p = state->p + state->q;
    state->k = state->p / (state->p + state->r);
    state->x = state->x + state->k * (measurement - state->x);
    state->p = (1 - state->k) * state->p;
}

// Interrupt handler for echo pin (rising and falling edges)
void get_echo_pulse(uint gpio, uint32_t events)
{
    if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_RISE)
    {
        // Echo pulse start detected
        start_time = get_absolute_time();
    }
    else if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_FALL)
    {
        // Echo pulse end detected, calculate pulse width
        pulse_width = absolute_time_diff_us(start_time, get_absolute_time());
    }
}

// Set up the ultrasonic sensor pins for TRIG and ECHO
void setupUltrasonicPins()
{
    gpio_init(TRIGPIN);
    gpio_init(ECHOPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);  // Set TRIG as output
    gpio_set_dir(ECHOPIN, GPIO_IN);   // Set ECHO as input
    gpio_put(TRIGPIN, 0);  // Ensure TRIG pin is low initially

    // Enable interrupts for rising and falling edges on the ECHO pin
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &get_echo_pulse);

    printf("Ultrasonic sensor pins configured: TRIG = %d, ECHO = %d\n", TRIGPIN, ECHOPIN);
}

// Send a trigger pulse and get the pulse width
uint64_t getPulse()
{
    pulse_width = 0;  // Reset pulse width before sending trigger pulse
    gpio_put(TRIGPIN, 1);  // Send trigger pulse (high for 10 µs)
    vTaskDelay(pdMS_TO_TICKS(10));  // Delay for 10 µs
    gpio_put(TRIGPIN, 0);  // Set trigger pin low

    absolute_time_t startTime = get_absolute_time();  // Capture current time

    // Polling for pulse width with a timeout
    while (pulse_width == 0 && absolute_time_diff_us(startTime, get_absolute_time()) < timeout)
    {
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to allow other tasks to run
    }

    // Debugging: Print pulse width for diagnostics
    printf("Pulse width: %llu µs\n", pulse_width);

    return pulse_width;
}

// Convert pulse width to distance in centimeters and apply Kalman filter
double getCm(kalman_state *state)
{
    uint64_t pulseLength = getPulse();  // Get pulse width from the sensor
    if (pulseLength == 0)
    {
        printf("Error: Pulse timeout or out of range.\n");
        return 0;
    }

    // Calculate distance in centimeters (pulse width / 29.0 gives cm)
    double measured = pulseLength / 29.0 / 2.0;

    // Ensure distance is within a reasonable range (e.g., between 2 cm and 400 cm)
    if (measured < 2 || measured > 400)
    {
        printf("Measured distance out of range: %.2f cm\n", measured);
        return 0;
    }

    // Update Kalman filter with the measured distance
    kalman_update(state, measured);

    // Check if an obstacle is detected within 10 cm
    if (state->x < 10)
    {
        obstacleDetected = true;
        printf("Obstacle detected within 10 cm!\n");
    }
    else
    {
        obstacleDetected = false;
    }

    return state->x;  // Return the filtered distance
}

// Ultrasonic task to continuously measure distance
void ultrasonic_task(void *pvParameters)
{
    kalman_state *state = (kalman_state *)pvParameters;  // Retrieve Kalman filter state

    while (true)
    {
        double distance_cm = getCm(state);  // Get the current distance
        if (distance_cm > 0)
        {
            printf("Distance: %.2f cm\n", distance_cm);  // Print distance
        }
        else
        {
            printf("Out of range or timeout\n");  // Print timeout message if no valid distance is detected
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 500 ms before the next measurement
    }
}
