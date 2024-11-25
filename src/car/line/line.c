// Include necessary headers
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include "semphr.h" // For mutex
#include "line.h"
#include "../motor/motor.h"

const char *code39_patterns[] = {
    "12112121111112212111121121211", "12112121112112111121121121211", "12112121111122111121121121211", "12112121112122111111121121211", "12112121111112211121121121211", // 0-4
    "12112121112112211111121121211", "12112121111122211111121121211", "12112121111112112121121121211", "12112121112112112111121121211", "12112121111122112111121121211", // 5-9
    "12112121112111121121121121211", "12112121111121121121121121211", "12112121112121121111121121211", "12112121111111221121121121211", "12112121112111221111121121211", // A-E
    "12112121111121221111121121211", "12112121111111122121121121211", "12112121112111122111121121211", "12112121111121122111121121211", "12112121111111222111121121211", // F-J
    "12112121112111111221121121211", "12112121111121111221121121211", "12112121112121111211121121211", "12112121111111211221121121211", "12112121112111211211121121211", // K-O
    "12112121111121211211121121211", "12112121111111112221121121211", "12112121112111112211121121211", "12112121111121112211121121211", "12112121111111212211121121211", // P-T
    "12112121112211111121121121211", "12112121111221111121121121211", "12112121112221111111121121211", "12112121111211211121121121211", "12112121112211211111121121211", // U-Y
    "12112121111221211111121121211", "12112121111211112121121121211", "12112121112211112111121121211", "12112121112222222211121121211"                                   // Z-*
};

const char code39_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-*";

// Global Variables for Barcode Detection
static uint64_t white_line_timestamp = 0;
static uint64_t last_event_time = 0;
static const char *prev_color = "White";
static bool is_decoding = false;
static bool first_black = false;
static uint64_t array_bar_widths[LINE_MAXIMUM_BARS];
static const char *array_bar_colors[LINE_MAXIMUM_BARS];
static int total_bars = 0;
static uint64_t maximum_width = 0;

// Mutex for ADC access
SemaphoreHandle_t xAdcMutex;

// Barcode Detection Functions

// Function to decode the entire barcode sequence
void get_barcode()
{
    show_found_bars();
    char decoded_char = get_pattern();
    printf("Decoded Character: %c\n", decoded_char);

    reset_data();
    is_decoding = false;
}

uint16_t read_adc(uint8_t input)
{
    adc_select_input(input);
    return adc_read();
}

char get_bar_width(uint64_t width, uint64_t max_width)
{
    // Use a ratio to determine narrow vs wide
    if (width <= max_width * 0.6)
        return '1'; // Narrow
    else
        return '2'; // Wide
}

void show_found_bars()
{
    for (int i = 0; i < total_bars; i++)
    {
        printf("Color: %s, Bar Width: %llu\n, Bar Count: %d\n", array_bar_colors[i], array_bar_widths[i], i);
    }
}

// Levenshtein Distance Function
int levenshtein_distance(const char *s, const char *t)
{
    int len_s = strlen(s);
    int len_t = strlen(t);

    // Early exit if length difference is too large
    if (abs(len_s - len_t) > 10)
    {
        return INT_MAX;
    }

    int matrix[len_s + 1][len_t + 1];

    // Initialization
    for (int i = 0; i <= len_s; i++)
        matrix[i][0] = i;
    for (int j = 0; j <= len_t; j++)
        matrix[0][j] = j;

    // Compute distances
    for (int i = 1; i <= len_s; i++)
    {
        for (int j = 1; j <= len_t; j++)
        {
            int cost = s[i - 1] == t[j - 1] ? 0 : 1;
            int deletion = matrix[i - 1][j] + 1;
            int insertion = matrix[i][j - 1] + 1;
            int substitution = matrix[i - 1][j - 1] + cost;

            int min = deletion < insertion ? deletion : insertion;
            matrix[i][j] = min < substitution ? min : substitution;
        }
    }
    return matrix[len_s][len_t];
}

void reset_data()
{
    for (int i = 0; i < LINE_MAXIMUM_BARS; i++)
    {
        array_bar_widths[i] = 0;
        array_bar_colors[i] = "unk";
    }
    total_bars = 0;
    maximum_width = 0;
    first_black = false;
    prev_color = "White";
    white_line_timestamp = 0;
}

char get_pattern()
{
    printf("Max Width = %llu\n", maximum_width);

    char pattern[LINE_MAXIMUM_BARS + 1] = {0};

    for (int i = 0; i < total_bars; i++)
    {
        pattern[i] = get_bar_width(array_bar_widths[i], maximum_width);
    }
    pattern[total_bars] = '\0';

    printf("Captured Pattern: %s\n", pattern);

    int min_distance = INT_MAX;
    char best_matches[10]; // Adjust size as needed
    int best_matches_count = 0;
    int best_match_is_reverse[10]; // Array to track if match is reverse

    // Check both the pattern and its reverse
    char pattern_buffer[LINE_MAXIMUM_BARS + 1] = {0};
    for (int i = 0; i < total_bars; i++)
    {
        pattern_buffer[i] = pattern[total_bars - i - 1];
    }
    pattern_buffer[total_bars] = '\0';

    // Iterate over Code 39 patterns
    int num_patterns = sizeof(code39_patterns) / sizeof(code39_patterns[0]);
    for (int i = 0; i < num_patterns; i++)
    {
        const char *reference_pattern = code39_patterns[i];

        int distance = levenshtein_distance(pattern, reference_pattern);
        int reverse_distance = levenshtein_distance(pattern_buffer, reference_pattern);

        printf("Comparing with pattern %s (%c), Distance: %d, Reverse Distance: %d\n",
               reference_pattern, code39_chars[i], distance, reverse_distance);

        int current_min_distance = (distance < reverse_distance) ? distance : reverse_distance;
        int is_reverse = (reverse_distance <= distance) ? 1 : 0;

        if (current_min_distance < min_distance)
        {
            min_distance = current_min_distance;
            // Reset best matches
            best_matches[0] = code39_chars[i];
            best_match_is_reverse[0] = is_reverse;
            best_matches_count = 1;
        }
        else if (current_min_distance == min_distance)
        {
            if (best_matches_count < sizeof(best_matches))
            {
                best_matches[best_matches_count] = code39_chars[i];
                best_match_is_reverse[best_matches_count] = is_reverse;
                best_matches_count++;
            }
        }
    }

    int max_acceptable_distance = 10; // Adjust this value based on testing

    if (min_distance <= max_acceptable_distance)
    {
        printf("Best Match: ");
        for (int i = 0; i < best_matches_count; i++)
        {
            printf("%c%s ", best_matches[i], best_match_is_reverse[i] ? "(Reversed)" : "");
        }
        printf("with Distance: %d\n", min_distance);

        // Return the first best match for simplicity
        return best_matches[0];
    }
    else
    {
        printf("No acceptable match found (Min Distance: %d)\n", min_distance);
        return '?';
    }
}

// Task to detect surface contrast and capture barcode data
void barcode_task(void *pvParameters)
{
    adc_gpio_init(BARCODE_SENSOR_PIN); // Initialize ADC pin for barcode sensor

    while (1)
    {
        // Acquire ADC mutex
        xSemaphoreTake(xAdcMutex, portMAX_DELAY);

        uint16_t adc_value = read_adc(0); // Read from ADC input 0 (Pin 26)

        // Release ADC mutex
        xSemaphoreGive(xAdcMutex);

        const char *curr_color = (adc_value > BARCODE_THRESHOLD) ? "Black" : "White";
        LineColor line_color = read_line_sensor();
        uint64_t now = time_us_64();

        if (!is_decoding && line_color == BLACK)
        {
            is_decoding = true;
            reset_data();
            prev_color = "Black"; // Initialize previous_color
            last_event_time = now;
        }

        if (is_decoding && !first_black && line_color == BLACK)
        {
            prev_color = "Black";
            first_black = true;
        }

        if (is_decoding && first_black)
        {
            if (strcmp(curr_color, prev_color) != 0)
            {
                uint64_t pulse_width_microseconds = now - last_event_time;

                if (pulse_width_microseconds > NOISE_THRESHOLD + DEBOUNCE_DELAY && total_bars < LINE_MAXIMUM_BARS)
                {
                    array_bar_widths[total_bars] = pulse_width_microseconds;
                    array_bar_colors[total_bars] = prev_color;

                    printf("Bar Color: %s\n", prev_color);
                    printf("Bar #%d: Width = %llu us\n", total_bars, pulse_width_microseconds);

                    if (pulse_width_microseconds > maximum_width)
                    {
                        maximum_width = pulse_width_microseconds;
                        printf("Updated Max Width: %llu\n", maximum_width);
                    }
                    total_bars++;
                }

                if (total_bars >= LINE_MAXIMUM_BARS)
                {
                    get_barcode();
                }

                prev_color = curr_color;
                last_event_time = now;
            }

            // Check for extended white period to force decode
            if (line_color == WHITE)
            {
                if (white_line_timestamp == 0)
                {
                    white_line_timestamp = now; // Start timing white period
                }
                else if (now - white_line_timestamp > WHITE_LINE_MAX_DURATION)
                {
                    printf("Triggering decode - Extended Whitespace Detected.\n");
                    get_barcode();
                    white_line_timestamp = 0; // Reset white timing
                }
            }
            else
            {
                white_line_timestamp = 0; // Reset white timing if black is detected
            }
        }
        else
        {
            // If not decoding, ensure last_white_time is reset
            white_line_timestamp = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for next ADC reading
    }
}

uint16_t read_adc_avg(uint8_t input, uint8_t num_samples)
{
    uint32_t sum = 0;

    // Read multiple samples and sum them
    for (uint8_t i = 0; i < num_samples; i++)
    {
        adc_select_input(input);
        sum += adc_read();
        sleep_us(50); // Small delay between samples for stability
    }

    // Return the average
    return (uint16_t)(sum / num_samples);
}

LineColor read_line_sensor(void)
{
    adc_select_input(1);
    // return (adc_read() > LINE_THRESHOLD) ? BLACK : WHITE;
    // return (adc_read() > 1100) ? BLACK : WHITE;
    return (adc_read() > 2000) ? BLACK : WHITE;
}

LineColor read_line_sensor2(void)
{
    uint16_t adc_value = read_adc_avg(1, 5); // Read from ADC input 1 (Pin 27)

    return (adc_value > LINE_THRESHOLD) ? BLACK : WHITE;
}

void init_ir_sensors(void)
{
    adc_init();
    adc_gpio_init(BARCODE_SENSOR_PIN);
    adc_gpio_init(LINE_SENSOR_PIN);
}
