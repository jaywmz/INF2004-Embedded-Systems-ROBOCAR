#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// ADC pin for the IR sensor
#define IR_SENSOR_PIN 26  // GPIO 26 is connected to the ADC input

// Threshold to detect black vs. white
#define THRESHOLD 3000  // Adjust based on calibration

// Number of samples for averaging
#define NUM_SAMPLES 20 // Number of samples to average

// Function to read a single ADC value
uint16_t read_adc() {
    return adc_read();  
}

// Function to detect surface contrast using the IR sensor and follow the line
void follow_line() {
    uint32_t adc_sum = 0;  // Variable to hold the sum of ADC readings
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adc_sum += read_adc();  // Sum up ADC readings
        sleep_us(100);  // Short delay between readings to allow ADC to stabilize
    }
    uint16_t adc_value = adc_sum / NUM_SAMPLES;  // Calculate average ADC value

    // Determine the current surface based on the threshold
    bool on_line = false;  // Explicitly initialize to false
    if (adc_value > THRESHOLD) {
        on_line = true;  // Set on_line to true if on black surface
    }

    // Adjust movement based on whether on line or off line
    if (on_line) {
        // Robot is on the line
        printf("On the line - moving straight\n");
        // Here, add logic to move both motors forward for straight movement
        // Example: move_forward();
    } else {
        // Robot is off the line
        printf("Off the line - adjusting\n");
        // Add logic to adjust direction to find the line again
        // Example: adjust_direction(); or adjust_left() / adjust_right();
    }

    // Print ADC value along with the surface detection result
    printf("ADC Value: %u, Surface: %s\n", adc_value, on_line ? "Black (line)" : "White (background)");
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
        // Call follow_line function to detect line and adjust movement
        follow_line();

        // Small delay before the next iteration
        sleep_us(1000);  // Short delay for better responsiveness
    }

    return 0;
}
