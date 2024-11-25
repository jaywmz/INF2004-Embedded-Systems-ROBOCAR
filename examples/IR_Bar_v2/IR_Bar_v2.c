

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <string.h>
#include <limits.h> // For INT_MAX
#include <stdlib.h>

// Define GPIO pins for motors
#define L_MOTOR_PWM_PIN 2
#define L_MOTOR_DIR_PIN1 0
#define L_MOTOR_DIR_PIN2 1
#define R_MOTOR_PWM_PIN 6
#define R_MOTOR_DIR_PIN1 4
#define R_MOTOR_DIR_PIN2 5

// PWM and speed settings
#define MAX_DUTY_CYCLE 12500

// IR Sensor
#define IR_SENSOR_PIN 26 // GPIO 26 connected to ADC
#define THRESHOLD 2480   // Black/White threshold
#define NUM_SAMPLES 2
#define MAX_BARS 29
#define NOISE_THRESHOLD 500             // Reduced for higher speed
#define DEBOUNCE_DELAY 500              // Reduced for higher speed
#define END_BAR_SPACE_THRESHOLD 3000000 // Threshold for detecting end-of-barcode space
volatile uint64_t white_line_timestamp = 0;  // Time when last white surface was detected
#define MAX_WHITE_TIME 5000000          // Maximum time to stay on white before forcing a decode

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
void move_forward(uint32_t gpioLeft, uint32_t gpioRight, float speed);
void straight_line_task(void *pvParameters);
void barcode_task(void *pvParameters);
uint16_t read_adc();
void reset_data();
char get_pattern();
void get_barcode();
void show_found_bars();
int levenshtein_distance(const char *s, const char *t);
char get_bar_width(uint64_t width, uint64_t max_width);

// Motor Control Code
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight)
{
    gpio_set_function(gpioLeft, GPIO_FUNC_PWM);
    uint32_t slice_num_left = pwm_gpio_to_slice_num(gpioLeft);
    pwm_set_clkdiv(slice_num_left, 100);
    pwm_set_wrap(slice_num_left, MAX_DUTY_CYCLE);
    pwm_set_enabled(slice_num_left, true);
    pwm_set_chan_level(slice_num_left, PWM_CHAN_A, 0);

    gpio_set_function(gpioRight, GPIO_FUNC_PWM);
    uint32_t slice_num_right = pwm_gpio_to_slice_num(gpioRight);
    pwm_set_clkdiv(slice_num_right, 100);
    pwm_set_wrap(slice_num_right, MAX_DUTY_CYCLE);
    pwm_set_enabled(slice_num_right, true);
    pwm_set_chan_level(slice_num_right, PWM_CHAN_B, 0);
}

void init_motor_pins()
{
    gpio_init(L_MOTOR_DIR_PIN1);
    gpio_init(L_MOTOR_DIR_PIN2);
    gpio_init(R_MOTOR_DIR_PIN1);
    gpio_init(R_MOTOR_DIR_PIN2);
    gpio_set_dir(L_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_DIR_PIN2, GPIO_OUT);
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

void move_forward(uint32_t gpioLeft, uint32_t gpioRight, float speed)
{
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);
    set_motor_speed(gpioLeft, speed, true);
    set_motor_speed(gpioRight, speed * 0.95, false); // Slight adjustment for balance
}

void straight_line_task(void *pvParameters)
{
    float move_speed = 0.0; // Adjust speed as needed
    while (true)
    {
        move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, move_speed);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Barcode Detection Code

// Variables for pulse measurement
volatile uint64_t last_event_time = 0; // Time of the last transition
const char *prev_color = "White";  // Track previous surface color ("Black" or "White")
bool is_decoding = false;              // Flag to indicate if decoding has started
bool first_black = false;              // Flag to confirm first Black bar

// Buffer for storing barcode data
uint64_t array_bar_widths[LINE_MAXIMUM_BARS];    // Array to store bar widths
const char *array_bar_colors[LINE_MAXIMUM_BARS]; // Array to store colors ("Black" or "White")
int total_bars = 0;                      // Counter for bars in a single character
uint64_t maximum_width = 0;                 // Track max width dynamically

uint16_t read_adc()
{
    return adc_read();
}

void reset_data()
{
    for (int i = 0; i < LINE_MAXIMUM_BARS; i++)
    {
        array_bar_widths[i] = 0;
        array_bar_colors[i] = "Unknown";
    }
    total_bars = 0;
    maximum_width = 0;
    first_black = false;
    prev_color = "White";
    white_line_timestamp = 0;
}

void show_found_bars()
{
    printf("Captured Bar Widths (during decoding):\n");
    for (int i = 0; i < total_bars; i++)
    {
        printf("Bar #%d: Width = %llu us, Color = %s\n", i, array_bar_widths[i], array_bar_colors[i]);
    }
}

char get_bar_width(uint64_t width, uint64_t max_width)
{
    // Use a ratio to determine narrow vs wide
    if (width <= max_width * 0.5)
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

char get_pattern()
{
    printf("Debug: Max Width = %llu\n", maximum_width);

    char pattern[LINE_MAXIMUM_BARS + 1] = {0};

    for (int i = 0; i < total_bars; i++)
    {
        pattern[i] = get_bar_width(array_bar_widths[i], maximum_width);
    }
    pattern[total_bars] = '\0';

    printf("Debug: Captured Pattern: %s\n", pattern);

    int min_distance = INT_MAX;
    char best_matches[10]; // Adjust size as needed
    int best_matches_count = 0;
    int best_match_is_reverse[10]; // Array to track if match is reverse

    // Check both the pattern and its reverse
    char reverse_pattern[LINE_MAXIMUM_BARS + 1] = {0};
    for (int i = 0; i < total_bars; i++)
    {
        reverse_pattern[i] = pattern[total_bars - i - 1];
    }
    reverse_pattern[total_bars] = '\0';

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
void get_barcode()
{
    printf("Decoding barcode...\n");
    show_found_bars();
    char decoded_char = get_pattern();
    printf("Decoded Character: %c\n", decoded_char);

    reset_data();
    is_decoding = false;
}

// Task to detect surface contrast and capture barcode data
void barcode_task(void *pvParameters)
{
    adc_init();
    adc_gpio_init(IR_SENSOR_PIN);
    adc_select_input(0);

    while (true)
    {
        uint16_t adc_value = read_adc();

        const char *current_color = (adc_value > THRESHOLD) ? "Black" : "White";
        uint64_t current_time = time_us_64();

        if (!is_decoding && strcmp(current_color, "Black") == 0)
        {
            is_decoding = true;
            reset_data();
            last_event_time = current_time;
            prev_color = "Black"; // Initialize previous_color
            printf("Starting barcode detection\n");
        }

        if (is_decoding && !first_black && strcmp(current_color, "Black") == 0)
        {
            first_black = true;
            prev_color = "Black";
        }

        if (is_decoding && first_black)
        {
            if (strcmp(current_color, prev_color) != 0)
            {
                uint64_t pulse_duration_us = current_time - last_event_time;

                if (pulse_duration_us > NOISE_THRESHOLD + DEBOUNCE_DELAY && total_bars < LINE_MAXIMUM_BARS)
                {
                    array_bar_widths[total_bars] = pulse_duration_us;
                    array_bar_colors[total_bars] = prev_color;

                    printf("Captured Bar Color: %s\n", prev_color);
                    printf("Captured bar #%d: Width = %llu us\n", total_bars, pulse_duration_us);

                    if (pulse_duration_us > maximum_width)
                    {
                        maximum_width = pulse_duration_us;
                        printf("Updated Max Width: %llu\n", maximum_width);
                    }
                    total_bars++;
                }

                if (total_bars >= LINE_MAXIMUM_BARS)
                {
                    get_barcode();
                }

                prev_color = current_color;
                last_event_time = current_time;
            }

            // Check for extended white period to force decode
            if (strcmp(current_color, "White") == 0)
            {
                if (white_line_timestamp == 0)
                {
                    white_line_timestamp = current_time; // Start timing white period
                }
                else if (current_time - white_line_timestamp > MAX_DURATION_WHITE_LINE)
                {
                    printf("Detected extended white space - triggering decode.\n");
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

// Main Function
int main()
{
    stdio_init_all();
    init_motor_pins();
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);

    TaskHandle_t straight_line_handle, detect_surface_contrast_handle;

    xTaskCreate(straight_line_task, "Straight Line Task", 256, NULL, 1, &straight_line_handle);
    xTaskCreate(barcode_task, "Barcode Detection Task", 1024, NULL, 1, &detect_surface_contrast_handle);

    vTaskStartScheduler();
    while (true)
    {
        // Should never reach here
    }
}

/*void straight_line_task(void *pvParameters)
{
    float move_speed = 0.5; // 70% speed for forward movement
    while (true)
    {
        // move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, move_speed);
        gpio_put(0, 1);
        gpio_put(1, 0);
        uint slice_num_motor1 = pwm_gpio_to_slice_num(2);
        pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A, 6000);

        gpio_put(4, 1);
        gpio_put(5, 0);
        uint slice_num_motor2 = pwm_gpio_to_slice_num(6);
        pwm_set_chan_level(slice_num_motor2, PWM_CHAN_A, 6000);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}*/
