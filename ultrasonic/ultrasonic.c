#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>  // Include this header for calloc

#define TRIGPIN 1
#define ECHOPIN 0

#define MOVING_AVERAGE_SIZE 5  // Number of recent measurements to consider for moving average
const int timeout = 26100;     // Timeout value (~4.5 meters)

volatile absolute_time_t start_time;
volatile uint64_t pulse_width = 0;
volatile bool obstacleDetected = false;

double moving_average_buffer[MOVING_AVERAGE_SIZE];  // Buffer for moving average
int buffer_index = 0;
int buffer_filled = 0;

// Kalman filter structure
typedef struct kalman_state_
{
    double q; // process noise covariance
    double r; // measurement noise covariance
    double x; // estimated value
    double p; // estimation error covariance
    double k; // Kalman gain
} kalman_state;

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

// Update Kalman filter with a new measurement
void kalman_update(kalman_state *state, double measurement)
{
    state->p = state->p + state->q;
    state->k = state->p / (state->p + state->r);  // Calculate Kalman gain
    state->x = state->x + state->k * (measurement - state->x);  // Update estimate
    state->p = (1 - state->k) * state->p;  // Update uncertainty
}

// Interrupt handler for echo pin (rising and falling edges)
void get_echo_pulse(uint gpio, uint32_t events)
{
    if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_RISE)
    {
        start_time = get_absolute_time();  // Echo pulse start
    }
    else if (gpio == ECHOPIN && events & GPIO_IRQ_EDGE_FALL)
    {
        pulse_width = absolute_time_diff_us(start_time, get_absolute_time());  // Echo pulse end
    }
}

// Set up the ultrasonic sensor pins
void setupUltrasonicPins()
{
    gpio_init(TRIGPIN);
    gpio_init(ECHOPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_set_dir(ECHOPIN, GPIO_IN);
    gpio_put(TRIGPIN, 0);  // Ensure trigger starts low

    // Enable rising and falling edge interrupts on the echo pin
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &get_echo_pulse);

    printf("Ultrasonic sensor pins configured: TRIG = %d, ECHO = %d\n", TRIGPIN, ECHOPIN);
}

// Send a pulse and get the pulse width
uint64_t getPulse() {
    pulse_width = 0;  // Reset the pulse width
    gpio_put(TRIGPIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_put(TRIGPIN, 0);

    absolute_time_t startTime = get_absolute_time();
    while (pulse_width == 0 && absolute_time_diff_us(startTime, get_absolute_time()) < timeout) {
        vTaskDelay(pdMS_TO_TICKS(1));  // Allow other tasks to run
    }

    return pulse_width;
}


// Calculate the moving average of recent distance measurements
double calculate_moving_average(double new_value)
{
    moving_average_buffer[buffer_index] = new_value;
    buffer_index = (buffer_index + 1) % MOVING_AVERAGE_SIZE;  // Update index in a circular buffer

    if (buffer_index == 0)
        buffer_filled = 1;  // Buffer fully populated

    double sum = 0;
    int count = buffer_filled ? MOVING_AVERAGE_SIZE : buffer_index;
    for (int i = 0; i < count; i++)
    {
        sum += moving_average_buffer[i];
    }

    return sum / count;  // Return the average value
}

// Convert pulse width to distance in centimeters, apply Kalman filter and moving average
double getCm(kalman_state *state)
{
    uint64_t pulseLength = getPulse();
    if (pulseLength == 0)
    {
        printf("Error: Pulse timeout or out of range.\n");
        return 0;
    }

    // Speed of sound in air is ~29 us/cm
    double measured = pulseLength / 29.0 / 2.0;

    // Apply moving average filter
    double averaged_distance = calculate_moving_average(measured);

    // Update Kalman filter
    kalman_update(state, averaged_distance);

    // Check for obstacles within 10 cm
    if (state->x < 10)
    {
        obstacleDetected = true;
        printf("Obstacle detected within 10 cm!\n");
    }
    else
    {
        obstacleDetected = false;
    }

    return state->x;  // Return filtered distance
}

// Ultrasonic task for FreeRTOS
void ultrasonic_task(void *pvParameters)
{
    kalman_state *state = (kalman_state *)pvParameters;

    while (true)
    {
        double distance_cm = getCm(state);
        if (distance_cm > 0)
        {
            printf("Distance: %.2f cm\n", distance_cm);
        }
        else
        {
            printf("Out of range or timeout\n");
        }

        // Delay between measurements
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main()
{
    stdio_init_all();  // Initialize serial output
    printf("Setting up ultrasonic sensor pins\n");

    setupUltrasonicPins();  // Initialize pins for the ultrasonic sensor

    // Initialize the Kalman filter (example parameters)
    kalman_state *state = kalman_init(1, 100, 1, 0);

    // Create the ultrasonic task for FreeRTOS
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 1024, (void *)state, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (1);  // The program should not reach this point due to FreeRTOS scheduler
    return 0;
}
