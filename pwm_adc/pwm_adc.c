#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// ADC pin for the IR sensor
#define IR_SENSOR_PIN 26  // GPIO 26 is connected to the ADC input

// Number of samples for averaging
#define NUM_SAMPLES 20 // Number of samples to average

// Variables for pulse measurement
volatile uint64_t last_transition_time = 0;  // Time of the last transition
volatile uint64_t pulse_duration_us = 0;      // Duration of the last pulse in microseconds
bool is_black = false;                         // Track whether the current surface is black

// Function to read a single ADC value
uint16_t read_adc() {
    return adc_read();  
}

// Function to detect surface contrast using the IR sensor
void detect_surface_contrast() {
    uint32_t adc_sum = 0;  // Variable to hold the sum of ADC readings
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adc_sum += read_adc();  // Sum up ADC readings
        sleep_us(100);  // Short delay between readings to allow ADC to stabilize
    }
    uint16_t adc_value = adc_sum / NUM_SAMPLES;  // Calculate average ADC value
   // printf("Average ADC Value: %u\n", adc_value); // Print the average ADC value

    // Determine the current surface based on a static threshold (820)
    bool current_is_black = (adc_value > 1000);  // Black threshold: anything above 820

    // Update surface state and transition timing
    if (current_is_black != is_black) {
        uint64_t current_time = time_us_64();  // Get the current time
        if (is_black) {
            // Transition from black to white
            pulse_duration_us = current_time - last_transition_time;  // Calculate pulse width for black
            printf("Transition to White: Pulse width (Black): %.2f ms\n", pulse_duration_us / 1000.0f);
        } else {
            // Transition from white to black
            pulse_duration_us = current_time - last_transition_time;  // Calculate pulse width for white
            printf("Transition to Black: Pulse width (White): %.2f ms\n", pulse_duration_us / 1000.0f);
        }

        // Update state and last transition time
        is_black = current_is_black;  // Update current state
        last_transition_time = current_time;  // Record the time of the transition
    }
     // Print ADC value along with surface color
   // printf("ADC Value: %u, Surface Color: %s\n", adc_value, current_is_black ? "Black" : "White");
   
}

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(500);  // Small delay to allow initialization

    // Initialize ADC for the IR sensor
    adc_init();
    adc_gpio_init(IR_SENSOR_PIN);  // Initialize GPIO for ADC input (IR sensor)
    adc_select_input(0);  // Select ADC channel 0 (connected to GPIO 26)

    while (1) {
        // Detect surface contrast using the IR sensor
        detect_surface_contrast();

        // Small delay before the next iteration
        sleep_us(1000);  // Short delay for better responsiveness
    }

    return 0;
}
