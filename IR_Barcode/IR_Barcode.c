#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include <string.h>  // Include this for strcmp

// ADC pin for the IR sensor
#define IR_SENSOR_PIN 26  // GPIO 26 is connected to the ADC input

// Thresholds and Constants
#define THRESHOLD 2200  // Fixed threshold for detecting black vs. white
#define NUM_SAMPLES 20  // Number of samples to average

// Barcode Scanning Parameters
#define CAR_SPEED_CM_PER_SEC 0.5              // Constant speed of the car in cm/s - testing 0.5cm per second
#define MAX_BARS 9                           // Number of bars (5 black, 4 white) in Code 39 character
#define END_BAR_SPACE_THRESHOLD 500000        // Increase threshold for detecting end-of-barcode space

// Variables for pulse measurement
volatile uint64_t last_transition_time = 0;  // Time of the last transition
volatile uint64_t pulse_duration_us = 0;     // Duration of the last pulse in microseconds
bool is_black = false;                       // Track whether the current surface is black
bool decoding_active = false;                // Flag to indicate if decoding has started

// Buffer for storing barcode data
uint16_t bar_widths[MAX_BARS];               // Array to store bar widths
bool bar_colors[MAX_BARS];                   // Array to store colors (true for black, false for white)
int bar_count = 0;                           // Counter for bars in a single character

// Code 39 Encoding (Narrow = 1, Wide = 2)
const char* code39_patterns[] = {
    "111221211", "211211112", "112211112", "212211111", "111221112", // 0-4
    "211221111", "112221111", "111211212", "211211211", "112211211", // 5-9
    "211112112", "112112112", "212112111", "111122112", "211122111", // A-E
    "112122111", "111112212", "211112211", "112112211", "111122211", // F-J
    "211111122", "112111122", "212111121", "111121122", "211121121", // K-O
    "112121121", "111111222", "211111221", "112111221", "111121221", // P-T
    "221111112", "122111112", "222111111", "121121112", "221121111", // U-Y
    "122121111", "121111212", "221111211", "122111211"             // Z-*
};
const char code39_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-*";

// Function to read a single ADC value
uint16_t read_adc() {
    return adc_read();  
}

// Adaptive function to classify a bar as narrow or wide based on relative widths
char classify_bar_width(uint16_t width, uint16_t min_width) {
    return (width <= min_width * 1.5) ? '1' : '2';  // Narrow if <= 1.5 times the smallest width; otherwise wide
}

// Function to decode a single character from the stored bar widths and colors
char decode_character() {
    if (bar_count != MAX_BARS) {
        printf("Debug: Bar count is not 9, unable to decode\n");
        return '?';  // Ensure there are exactly 9 bars (Code 39 pattern)
    }

    // Find the minimum width to use as a baseline for narrow vs. wide classification
    uint16_t min_width = bar_widths[0];
    for (int i = 1; i < MAX_BARS; i++) {
        if (bar_widths[i] < min_width) {
            min_width = bar_widths[i];
        }
    }
    
    // Construct the pattern as a string of '1's and '2's for both left-to-right and right-to-left
    char pattern[MAX_BARS + 1];
    char reversed_pattern[MAX_BARS + 1];

    printf("Debug: Constructing patterns for decoding\n");
    for (int i = 0; i < MAX_BARS; i++) {
        pattern[i] = classify_bar_width(bar_widths[i], min_width);
        reversed_pattern[MAX_BARS - i - 1] = pattern[i];  // Reverse the order for reversed_pattern
    }
    pattern[MAX_BARS] = '\0';  // Null-terminate the pattern string
    reversed_pattern[MAX_BARS] = '\0';

    printf("Debug: Pattern: %s, Reversed Pattern: %s\n", pattern, reversed_pattern);

    // Match the pattern against Code 39 patterns in both directions
    for (int i = 0; i < sizeof(code39_patterns) / sizeof(code39_patterns[0]); i++) {
        if (strcmp(pattern, code39_patterns[i]) == 0) {
            printf("Debug: Matched pattern in left-to-right direction\n");
            return code39_chars[i];  // Return the matching character for left-to-right
        }
        if (strcmp(reversed_pattern, code39_patterns[i]) == 0) {
            printf("Debug: Matched pattern in right-to-left direction\n");
            return code39_chars[i];  // Return the matching character for right-to-left
        }
    }
    printf("Debug: No matching pattern found\n");
    return '?';  // Return '?' if no match is found
}

// Function to decode the entire barcode sequence
void decode_barcode() {
    printf("Decoding barcode...\n");
    char decoded_char = decode_character();

    // Check if a valid character was returned
    if (decoded_char != '?') {
        printf("Decoded Character: %c\n", decoded_char);
    } else {
        printf("Debug: Unable to decode the character, returned '?'\n");
    }

    // Clear the buffer and reset decoding state after decoding
    bar_count = 0;
    decoding_active = false;  // Return to idle mode
}


// Function to detect surface contrast using the IR sensor and store pulse widths
void detect_surface_contrast() {
    uint32_t adc_sum = 0;  // Variable to hold the sum of ADC readings
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adc_sum += read_adc();  // Sum up ADC readings
        sleep_us(100);  // Short delay between readings to allow ADC to stabilize
    }
    uint16_t adc_value = adc_sum / NUM_SAMPLES;  // Calculate average ADC value
   // printf("Debug: ADC Value: %u\n", adc_value);  // Print ADC value for debugging

    // Determine the current surface based on a static threshold
    bool current_is_black = (adc_value > THRESHOLD);  // Black threshold: anything above 2200

    // If in idle mode, only start decoding when a black bar is detected
    if (!decoding_active && current_is_black) {
        decoding_active = true;  // Start decoding on the first black bar
        bar_count = 0;  // Reset the bar count for a new barcode
        printf("Debug: Starting barcode detection\n");
    }

    // If decoding is active, process the bar widths
    if (decoding_active) {
        // Update surface state and transition timing
        if (current_is_black != is_black) {
            uint64_t current_time = time_us_64();  // Get the current time
            pulse_duration_us = current_time - last_transition_time;  // Calculate pulse width for the previous color

            // Store the pulse width and color in the buffer if not exceeding MAX_BARS
            if (bar_count < MAX_BARS) {
                bar_widths[bar_count] = pulse_duration_us;
                bar_colors[bar_count] = current_is_black;
                printf("Debug: Captured bar #%d: Width = %u us", bar_count, pulse_duration_us);
                bar_count++;
            }

            // If we've collected 9 bars (5 black, 4 white), decode the character
            if (bar_count == MAX_BARS) {
                decode_barcode();
            }

            // Check for end-of-barcode signal based on a long white space
            if (!current_is_black && pulse_duration_us > END_BAR_SPACE_THRESHOLD) {
                printf("Debug: Detected end-of-barcode signal\n");
                decode_barcode();  // Trigger decoding for the accumulated bars
            }

            // Update state and last transition time
            is_black = current_is_black;  // Update current state
            last_transition_time = current_time;  // Record the time of the transition
        }
    }
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
