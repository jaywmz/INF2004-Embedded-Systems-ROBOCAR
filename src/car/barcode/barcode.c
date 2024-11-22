#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include <string.h>  
#include "barcode.h"

// ADC pin for the IR sensor
#define IR_SENSOR_PIN 26  // GPIO 26 is connected to the ADC input

// Thresholds and parameters
#define THRESHOLD 2000           // Threshold to distinguish between black and white
#define NUM_SAMPLES 5            // Number of samples to average
#define MAX_BARS 29              // Number of bars in Code 39 (start, character, stop)
#define END_BAR_SPACE_THRESHOLD 3000000  // Threshold for detecting end-of-barcode space
volatile uint64_t last_white_time = 0;       // Time when last white surface was detected
#define MAX_WHITE_TIME 3000000   // Maximum time to stay on white before forcing a decode
#define NOISE_THRESHOLD 5000     // Minimum pulse width to consider as valid bar
#define DEBOUNCE_DELAY 5000      // Minimum delay between transitions in microseconds

// Variables for pulse measurement
volatile uint64_t last_transition_time = 0;  // Time of the last transition
const char* previous_color = "White";        // Track previous surface color ("Black" or "White")
bool decoding_active = false;                // Flag to indicate if decoding has started
bool initial_black_detected = false;         // Flag to confirm first Black bar

// Buffer for storing barcode data
uint64_t bar_widths[MAX_BARS];               // Array to store bar widths
const char* bar_colors[MAX_BARS];            // Array to store colors ("Black" or "White")
int bar_count = 0;                           // Counter for bars in a single character
uint64_t max_width = 0;                      // Track max width dynamically

// Code 39 Encoding (Narrow = 1, Wide = 2)
const char* code39_patterns[] = {
    "12112121111112212111121121211", "12112121112112111121121121211", "12112121111122111121121121211", "12112121112122111111121121211", "12112121111112211121121121211", // 0-4
    "12112121112112211111121121211", "12112121111122211111121121211", "12112121111112112121121121211", "12112121112112112111121121211", "12112121111122112111121121211", // 5-9
    "12112121112111121121121121211", "12112121111121121121121121211", "12112121112121121111121121211", "12112121111111221121121121211", "12112121112111221111121121211", // A-E
    "12112121111121221111121121211", "12112121111111122121121121211", "12112121112111122111121121211", "12112121111121122111121121211", "12112121111111222111121121211", // F-J
    "12112121112111111221121121211", "12112121111121111221121121211", "12112121112121111211121121211", "12112121111111211221121121211", "12112121112111211211121121211", // K-O
    "12112121111121211211121121211", "12112121111111112221121121211", "12112121112111112211121121211", "12112121111121112211121121211", "12112121111111212211121121211", // P-T
    "12112121112211111121121121211", "12112121111221111121121121211", "12112121112221111111121121211", "12112121111211211121121121211", "12112121112211211111121121211", // U-Y
    "12112121111221211111121121211", "12112121111211112121121121211", "12112121112211112111121121211", "12112121112222222211121121211"              // Z-*
};

const char code39_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-*";

// Function to read a single ADC value
uint16_t read_adc() {
    return adc_read();  
}

// Adaptive function to classify a bar as narrow or wide based on max_width
char classify_bar_width(uint64_t width, uint64_t max_width) {
    return (width <= max_width / 3) ? '1' : '2';  // Narrow if <= 1/3 of max width; otherwise wide
}

// Function to reset bar data
void reset_bar_data() {
    for (int i = 0; i < MAX_BARS; i++) {
        bar_widths[i] = 0;
        bar_colors[i] = "Unknown";
    }
    bar_count = 0;
    max_width = 0;
    initial_black_detected = false;
}

// Function to display captured bars for debugging
void display_captured_bars() {
    printf("Captured Bar Widths (during decoding):\n");
    for (int i = 0; i < bar_count; i++) {
        printf("Bar #%d: Width = %llu, Color = %s\n", i, bar_widths[i], bar_colors[i]);
    }
}

// Function to decode a single character from the stored bar widths and colors
char decode_character() {
    // Calculate max width only for captured bars to avoid empty entries
    uint64_t threshold = max_width / 3;
    printf("Debug: Max Width = %llu, Threshold = %llu\n", max_width, threshold);

    // Construct the pattern and reverse pattern based on captured data
    char pattern[MAX_BARS + 1] = {0};
    char reverse_pattern[MAX_BARS + 1] = {0};

    for (int i = 0; i < bar_count; i++) {
        pattern[i] = classify_bar_width(bar_widths[i], max_width);
        reverse_pattern[bar_count - i - 1] = pattern[i];  // Reverse the pattern
    }

    printf("Debug: Pattern: %s\n", pattern);
    printf("Debug: Reverse Pattern: %s\n", reverse_pattern);

    // Check both patterns for matches
    for (int i = 0; i < sizeof(code39_patterns) / sizeof(code39_patterns[0]); i++) {
        if (strcmp(pattern, code39_patterns[i]) == 0) {
            printf("Debug: Matched pattern (L->R)\n");
            return code39_chars[i];
        }
        if (strcmp(reverse_pattern, code39_patterns[i]) == 0) {
            printf("Debug: Matched pattern (R->L)\n");
            return code39_chars[i];
        }
    }
    printf("Debug: No matching pattern found\n");
    return '?';
}

// Function to decode the entire barcode sequence
void decode_barcode() {
    printf("Decoding barcode...\n");
    display_captured_bars();
    char decoded_char = decode_character();
    printf("Decoded Character: %c\n", decoded_char);

    reset_bar_data();
    decoding_active = false;
}

// Function to detect surface contrast using the IR sensor and store pulse widths
void detect_surface_contrast() {
    uint32_t adc_sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adc_sum += read_adc();
        sleep_us(100);
    }
    uint16_t adc_value = adc_sum / NUM_SAMPLES;

    // Determine the current surface color
    const char* current_color = (adc_value > THRESHOLD) ? "Black" : "White";
    uint64_t current_time = time_us_64();

    // Start decoding only upon detecting the first black bar
    if (!decoding_active && strcmp(current_color, "Black") == 0) {
        decoding_active = true;
        reset_bar_data();
        last_transition_time = current_time;  // Reset transition time to start fresh timing
        printf("Starting barcode detection\n");
    }

    // Ensure the first bar is Black before recording
    if (decoding_active && !initial_black_detected && strcmp(current_color, "Black") == 0) {
        initial_black_detected = true;
    }

    // Process bar widths when decoding is active and first black has been detected
    if (decoding_active && initial_black_detected) {
        if (strcmp(current_color, previous_color) != 0) {  // Detect color transition
            uint64_t pulse_duration_us = current_time - last_transition_time;

            // Only store valid bar if it exceeds noise threshold and debounce delay
            if (pulse_duration_us > NOISE_THRESHOLD + DEBOUNCE_DELAY && bar_count < MAX_BARS) {
                bar_widths[bar_count] = pulse_duration_us;
                bar_colors[bar_count] = previous_color;
                printf("Captured Bar Color: %s\n", previous_color);
                printf("Captured bar #%d: Width = %llu us\n", bar_count, pulse_duration_us);

                // Update max width dynamically if this pulse width is larger
                if (pulse_duration_us > max_width) {
                    max_width = pulse_duration_us;
                    printf("Updated Max Width: %llu\n", max_width);
                }
                bar_count++;
            }

            // Decode when 29 bars have been captured
            if (bar_count == MAX_BARS) {
                decode_barcode();
            }

            previous_color = current_color;
            last_transition_time = current_time;
        }

        // Check for extended white period to force decode
        if (strcmp(current_color, "White") == 0) {
            if (last_white_time == 0) {
                last_white_time = current_time;  // Start timing white period
            } else if (current_time - last_white_time > MAX_WHITE_TIME) {
                printf("Detected extended white space - triggering decode.\n");
                decode_barcode();
                last_white_time = 0;  // Reset white timing
            }
        } else {
            last_white_time = 0;  // Reset white timing if black is detected
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(500);

    adc_init();
    adc_gpio_init(IR_SENSOR_PIN);
    adc_select_input(0);

    while (1) {
        detect_surface_contrast();
        sleep_us(100);
    }

    return 0;
}
