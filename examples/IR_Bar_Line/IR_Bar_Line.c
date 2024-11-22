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

// Define GPIO pins for motors
#define L_MOTOR_PWM_PIN 2
#define L_MOTOR_DIR_PIN1 0
#define L_MOTOR_DIR_PIN2 1
#define R_MOTOR_PWM_PIN 6
#define R_MOTOR_DIR_PIN1 4
#define R_MOTOR_DIR_PIN2 5

// PWM and speed settings
#define MAX_DUTY_CYCLE 12500

// IR Sensors
#define BARCODE_SENSOR_PIN 26   // GPIO 26 connected to ADC input 0 (Barcode)
#define LINE_SENSOR_PIN 27      // GPIO 27 connected to ADC input 1 (Line Following)

// Thresholds
#define BARCODE_THRESHOLD 1800  // Threshold for barcode detection
#define LINE_THRESHOLD 1800     // Threshold for line-following sensor

// Barcode Detection Settings
#define NUM_SAMPLES 2
#define MAX_BARS 29
#define NOISE_THRESHOLD 500
#define DEBOUNCE_DELAY 500
#define END_BAR_SPACE_THRESHOLD 3000000
volatile uint64_t last_white_time = 0;
#define MAX_WHITE_TIME 5000000

// Code 39 Encoding (Narrow = 1, Wide = 2)
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

// Function declarations
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight);
void init_motor_pins();
void move_forward(float speed);
void turn_left(float speed);
void turn_right(float speed);
void stop_motors();
void set_motor_speed(uint32_t gpio, float speed, bool is_left);

void detect_surface_contrast_task(void *pvParameters);
void line_following_task(void *pvParameters);

uint16_t read_adc(uint8_t input);
void reset_bar_data();
char decode_character();
void decode_barcode();
void display_captured_bars();
int levenshtein_distance(const char *s, const char *t);
char classify_bar_width(uint64_t width, uint64_t max_width);

// Global Variables for Barcode Detection
volatile uint64_t last_transition_time = 0;
const char *previous_color = "White";
bool decoding_active = false;
bool initial_black_detected = false;
uint64_t bar_widths[MAX_BARS];
const char *bar_colors[MAX_BARS];
int bar_count = 0;
uint64_t max_width = 0;

// Mutex for ADC access
SemaphoreHandle_t xAdcMutex;

// Motor Control Functions

void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight)
{
    // Left Motor PWM Setup
    gpio_set_function(gpioLeft, GPIO_FUNC_PWM);
    uint32_t slice_num_left = pwm_gpio_to_slice_num(gpioLeft);
    pwm_set_clkdiv(slice_num_left, 100);
    pwm_set_wrap(slice_num_left, MAX_DUTY_CYCLE);
    pwm_set_enabled(slice_num_left, true);
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, 0);

    // Right Motor PWM Setup
    gpio_set_function(gpioRight, GPIO_FUNC_PWM);
    uint32_t slice_num_right = pwm_gpio_to_slice_num(gpioRight);
    pwm_set_clkdiv(slice_num_right, 100);
    pwm_set_wrap(slice_num_right, MAX_DUTY_CYCLE);
    pwm_set_enabled(slice_num_right, true);
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, 0);
}

void init_motor_pins()
{
    // Initialize direction control pins for left motor
    gpio_init(L_MOTOR_DIR_PIN1);
    gpio_init(L_MOTOR_DIR_PIN2);
    gpio_set_dir(L_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_DIR_PIN2, GPIO_OUT);

    // Initialize direction control pins for right motor
    gpio_init(R_MOTOR_DIR_PIN1);
    gpio_init(R_MOTOR_DIR_PIN2);
    gpio_set_dir(R_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(R_MOTOR_DIR_PIN2, GPIO_OUT);
}

void set_motor_speed(uint32_t gpio, float speed, bool is_left)
{
    if (speed > 1.0)
        speed = 1.0;
    if (speed < 0.0)
        speed = 0.0;

    uint32_t duty_cycle = (uint32_t)(speed * MAX_DUTY_CYCLE);
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), is_left ? PWM_CHAN_A : PWM_CHAN_B, duty_cycle);
}

void move_forward(float speed)
{
    printf("Moving forward with speed: %f\n", speed);
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);
    set_motor_speed(L_MOTOR_PWM_PIN, speed, true);
    set_motor_speed(R_MOTOR_PWM_PIN, speed, true);
    //set_motor_speed(R_MOTOR_PWM_PIN, speed * 0.95, false); // Slight adjustment for balance
}

void turn_left(float speed)
{
    // Left motor backward, right motor forward
    gpio_put(L_MOTOR_DIR_PIN1, 0);
    gpio_put(L_MOTOR_DIR_PIN2, 1);
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);
    set_motor_speed(L_MOTOR_PWM_PIN, speed, true);
    set_motor_speed(R_MOTOR_PWM_PIN, speed, false);
}

void turn_right(float speed)
{
    // Left motor forward, right motor backward
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);
    gpio_put(R_MOTOR_DIR_PIN1, 0);
    gpio_put(R_MOTOR_DIR_PIN2, 1);
    set_motor_speed(L_MOTOR_PWM_PIN, speed, true);
    set_motor_speed(R_MOTOR_PWM_PIN, speed, false);
}

void stop_motors()
{
    printf("Stopping motors\n");
    set_motor_speed(L_MOTOR_PWM_PIN, 0.0, true);
    set_motor_speed(R_MOTOR_PWM_PIN, 0.0, false);
}

// Barcode Detection Functions

uint16_t read_adc(uint8_t input)
{
    adc_select_input(input);
    return adc_read();
}

void reset_bar_data()
{
    for (int i = 0; i < MAX_BARS; i++)
    {
        bar_widths[i] = 0;
        bar_colors[i] = "Unknown";
    }
    bar_count = 0;
    max_width = 0;
    initial_black_detected = false;
    previous_color = "White";
    last_white_time = 0;
}

void display_captured_bars()
{
    printf("Captured Bar Widths (during decoding):\n");
    for (int i = 0; i < bar_count; i++)
    {
        printf("Bar #%d: Width = %llu us, Color = %s\n", i, bar_widths[i], bar_colors[i]);
    }
}

char classify_bar_width(uint64_t width, uint64_t max_width)
{
    // Use a ratio to determine narrow vs wide
    if (width <= max_width * 0.6)
        return '1'; // Narrow
    else
        return '2'; // Wide
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

char decode_character()
{
    printf("Debug: Max Width = %llu\n", max_width);

    char pattern[MAX_BARS + 1] = {0};

    for (int i = 0; i < bar_count; i++)
    {
        pattern[i] = classify_bar_width(bar_widths[i], max_width);
    }
    pattern[bar_count] = '\0';

    printf("Debug: Captured Pattern: %s\n", pattern);

    int min_distance = INT_MAX;
    char best_matches[10];  // Adjust size as needed
    int best_matches_count = 0;
    int best_match_is_reverse[10];  // Array to track if match is reverse

    // Check both the pattern and its reverse
    char reverse_pattern[MAX_BARS + 1] = {0};
    for (int i = 0; i < bar_count; i++)
    {
        reverse_pattern[i] = pattern[bar_count - i - 1];
    }
    reverse_pattern[bar_count] = '\0';

    // Iterate over Code 39 patterns
    int num_patterns = sizeof(code39_patterns) / sizeof(code39_patterns[0]);
    for (int i = 0; i < num_patterns; i++)
    {
        const char *reference_pattern = code39_patterns[i];

        int distance = levenshtein_distance(pattern, reference_pattern);
        int reverse_distance = levenshtein_distance(reverse_pattern, reference_pattern);

        printf("Debug: Comparing with pattern %s (%c), Distance: %d, Reverse Distance: %d\n",
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
        printf("Debug: Best Match(es): ");
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
        printf("Debug: No acceptable match found (Min Distance: %d)\n", min_distance);
        return '?';
    }
}


// Function to decode the entire barcode sequence
void decode_barcode()
{
    printf("Decoding barcode...\n");
    display_captured_bars();
    char decoded_char = decode_character();
    printf("Decoded Character: %c\n", decoded_char);

    reset_bar_data();
    decoding_active = false;
}


// Task to detect surface contrast and capture barcode data
void detect_surface_contrast_task(void *pvParameters)
{
    adc_gpio_init(BARCODE_SENSOR_PIN); // Initialize ADC pin for barcode sensor

    while (true)
    {
        // Acquire ADC mutex
        xSemaphoreTake(xAdcMutex, portMAX_DELAY);

        uint16_t adc_value = read_adc(0); // Read from ADC input 0 (Pin 26)

        // Release ADC mutex
        xSemaphoreGive(xAdcMutex);

        const char *current_color = (adc_value > BARCODE_THRESHOLD) ? "Black" : "White";
        uint64_t current_time = time_us_64();

        if (!decoding_active && strcmp(current_color, "Black") == 0)
        {
            decoding_active = true;
            reset_bar_data();
            last_transition_time = current_time;
            previous_color = "Black"; // Initialize previous_color
            printf("Starting barcode detection\n");
        }

        if (decoding_active && !initial_black_detected && strcmp(current_color, "Black") == 0)
        {
            initial_black_detected = true;
            previous_color = "Black";
        }

        if (decoding_active && initial_black_detected)
        {
            if (strcmp(current_color, previous_color) != 0)
            {
                uint64_t pulse_duration_us = current_time - last_transition_time;

                if (pulse_duration_us > NOISE_THRESHOLD + DEBOUNCE_DELAY && bar_count < MAX_BARS)
                {
                    bar_widths[bar_count] = pulse_duration_us;
                    bar_colors[bar_count] = previous_color;

                    printf("Captured Bar Color: %s\n", previous_color);
                    printf("Captured bar #%d: Width = %llu us\n", bar_count, pulse_duration_us);

                    if (pulse_duration_us > max_width)
                    {
                        max_width = pulse_duration_us;
                        printf("Updated Max Width: %llu\n", max_width);
                    }
                    bar_count++;
                }

                if (bar_count >= MAX_BARS)
                {
                    decode_barcode();
                }

                previous_color = current_color;
                last_transition_time = current_time;
            }

            // Check for extended white period to force decode
            if (strcmp(current_color, "White") == 0)
            {
                if (last_white_time == 0)
                {
                    last_white_time = current_time; // Start timing white period
                }
                else if (current_time - last_white_time > MAX_WHITE_TIME)
                {
                    printf("Detected extended white space - triggering decode.\n");
                    decode_barcode();
                    last_white_time = 0; // Reset white timing
                }
            }
            else
            {
                last_white_time = 0; // Reset white timing if black is detected
            }
        }
        else
        {
            // If not decoding, ensure last_white_time is reset
            last_white_time = 0;
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

void line_following_task(void *pvParameters)
{
    enum { ON_LINE, SEARCHING } robot_state = ON_LINE;
    adc_gpio_init(LINE_SENSOR_PIN); // Initialize ADC pin for line sensor

    // Define the number of samples to average
    const uint8_t num_samples = 10;

    // Start moving forward initially
    move_forward(0.5); // Set initial speed to 50%

    while (1)
    {
        // Acquire ADC mutex
        xSemaphoreTake(xAdcMutex, portMAX_DELAY);

        // Get averaged ADC value
        uint16_t adc_value = read_adc_avg(1, num_samples); // Read from ADC input 1 (Pin 27)

        // Release ADC mutex
        xSemaphoreGive(xAdcMutex);

        printf("[Line Following: Averaged ADC Value: %d]\n", adc_value);

        if (adc_value > LINE_THRESHOLD)
        {
            // Black line detected
            if (robot_state == SEARCHING)
            {
                printf("Black line found. Resuming normal line-following.\n");
            }

            // Move forward at normal speed
            move_forward(0.5); // 50% speed
            robot_state = ON_LINE;
        }
        else
        {
            // White surface detected
            if (robot_state == ON_LINE)
            {
                printf("Line lost. Entering search mode.\n");
                robot_state = SEARCHING;
            }

            // Search for the black line by turning alternately
            static bool turn_left_first = true; // Alternate between left and right turns

            if (turn_left_first)
            {
                printf("Searching: Turning left.\n");
                turn_left(0.3); // Turn left slightly
                vTaskDelay(pdMS_TO_TICKS(200)); // Small turn duration
            }
            else
            {
                printf("Searching: Turning right.\n");
                turn_right(0.3); // Turn right slightly
                vTaskDelay(pdMS_TO_TICKS(200)); // Small turn duration
            }

            // Alternate direction for the next search attempt
            turn_left_first = !turn_left_first;

            // Stop after each adjustment to allow sensor re-check
            stop_motors();
            vTaskDelay(pdMS_TO_TICKS(100)); // Short delay before re-checking sensor

            // Re-check sensor value after adjustment
            xSemaphoreTake(xAdcMutex, portMAX_DELAY);
            adc_value = read_adc_avg(1, num_samples); // Read from ADC input 1 (Pin 27)
            xSemaphoreGive(xAdcMutex);

            if (adc_value > LINE_THRESHOLD)
            {
                // Black line re-detected
                printf("Black line re-detected during search.\n");
                move_forward(0.5); // Resume forward motion
                robot_state = ON_LINE;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay for stability
    }
}
