#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Encoder and Motor Configuration
#define ENCODER_PIN 2       // Encoder signal input
#define MOTOR_PWM_PIN 15    // Motor PWM output pin
#define WHEEL_DIAMETER_MM 65  // Diameter of the wheel in mm
#define ENCODER_SLOTS 20    // Number of slots in the encoder

// Variables to track pulse count and speed
volatile uint32_t pulse_count = 0;
volatile uint64_t last_pulse_time = 0;
volatile float speed_m_per_s = 0.0;  // Now tracking speed in meters per second
volatile float total_distance_m = 0.0;

// Constants
const float distance_per_pulse_mm = (WHEEL_DIAMETER_MM * 3.1416) / ENCODER_SLOTS;  // Distance per pulse in mm
uint64_t debounce_time_us = 1000;  // 1 ms debounce time

// Timer for speed calculation
struct repeating_timer speed_timer;

// Callback function to handle edge detection (rising edge)
void encoder_callback(uint gpio, uint32_t events) {
    if (gpio == ENCODER_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        // Get the current time in microseconds
        uint64_t current_time = time_us_64();
        
        // Debounce: process pulse only if it's been at least debounce_time_us since the last pulse
        if (current_time - last_pulse_time > debounce_time_us) {
            // Measure the time difference between current and previous pulse
            uint64_t pulse_duration_us = current_time - last_pulse_time;
            last_pulse_time = current_time;

            // Increment pulse count
            pulse_count++;

            // Calculate speed in m/s (convert mm to meters and microseconds to seconds)
            if (pulse_duration_us > 0 && pulse_duration_us < 1000000) {  // Filter out long durations
                float pulse_duration_s = pulse_duration_us / 1000000.0f;
                speed_m_per_s = (distance_per_pulse_mm / 1000.0f) / pulse_duration_s;  // Speed in m/s
            } else {
                speed_m_per_s = 0;  // If pulse duration is too long, assume motor has stopped
            }

            // Update total distance traveled in meters
            total_distance_m += distance_per_pulse_mm / 1000.0f;  // Convert mm to meters

            // Print out the results for debugging
            printf("Pulse Duration: %.2f ms, Pulses: %d, Speed: %.2f m/s, Total Distance: %.2f meters\n", 
                   pulse_duration_us / 1000.0f, pulse_count, speed_m_per_s, total_distance_m);
        }
    }
}

// Timer callback for regular speed updates
bool speed_timer_callback(struct repeating_timer *t) {
    // Reset speed if no pulse has been detected in the last second
    if (time_us_64() - last_pulse_time > 1000000) {
        speed_m_per_s = 0;  // Assume motor stopped if no pulse detected
    }

    // Print speed update and total distance
    printf("Speed Update -> Speed: %.2f m/s, Total Distance: %.2f meters\n", speed_m_per_s, total_distance_m);
    return true;
}

// PWM setup for motor control
void setup_pwm() {
    // Set the GPIO function to PWM
    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);

    // Find the slice number for the PWM pin
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);

    // Set PWM frequency (e.g., 100 Hz frequency)
    pwm_set_wrap(slice_num, 12500);  // Set wrap value for a 10ms period (100Hz PWM frequency)

    // Set initial duty cycle (e.g., 50% duty cycle)
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 6250);  // 50% duty cycle

    // Enable PWM
    pwm_set_enabled(slice_num, true);
}

// Function to adjust motor speed dynamically
void set_motor_speed(float duty_cycle) {
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    if (duty_cycle > 100.0f) duty_cycle = 100.0f;  // Cap duty cycle to 100%
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;      // Prevent negative duty cycle
    uint16_t level = (uint16_t)(duty_cycle * 12500 / 100.0f);  // Set level based on duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, level);  // Adjust duty cycle
}

int main() {
    // Initialize stdio (for USB serial output)
    stdio_init_all();

    // Initialize the encoder pin (GP02)
    gpio_init(ENCODER_PIN);
    gpio_set_dir(ENCODER_PIN, GPIO_IN);
    gpio_pull_down(ENCODER_PIN);  // Ensure the pin is low when no signal is present

    // Set the interrupt to trigger on the rising edge (pulse detection)
    gpio_set_irq_enabled_with_callback(ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);

    // Set up PWM for motor control
    setup_pwm();

    // Start a repeating timer to print speed and distance every second
    add_repeating_timer_ms(1000, speed_timer_callback, NULL, &speed_timer);

    // Main loop (nothing to do here, everything happens in the interrupt and timer)
    while (true) {
        tight_loop_contents();  // Avoid CPU overload
    }

    return 0;
}
