#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <string.h>  

// Define GPIO pins for motors
#define L_MOTOR_PWM_PIN 11
#define L_MOTOR_DIR_PIN1 12
#define L_MOTOR_DIR_PIN2 13
#define R_MOTOR_PWM_PIN 10
#define R_MOTOR_DIR_PIN1 14
#define R_MOTOR_DIR_PIN2 15

// PWM and speed settings
#define MAX_DUTY_CYCLE 12500

// IR Sensor
#define IR_SENSOR_PIN 26          // GPIO 26 connected to ADC
#define THRESHOLD 4085           // Black/White threshold
#define NUM_SAMPLES 5
#define MAX_BARS 29
#define NOISE_THRESHOLD 5000      // Minimum pulse width to consider valid
#define DEBOUNCE_DELAY 5000       // Minimum delay between transitions in microseconds
#define END_BAR_SPACE_THRESHOLD 5000000  // Threshold for detecting end-of-barcode space

// Code 39 Encoding (Narrow = 1, Wide = 2)
const char* code39_patterns[] = {
    "12112121111112212111121121211", "12112121112112111121121121211", "12112121111122111121121121211",
    "12112121112122111111121121211", "12112121111112211121121121211", "12112121112112211111121121211",
    "12112121111122211111121121211", "12112121111112112121121121211", "12112121112112112111121121211",
    "12112121111122112111121121211", "12112121112111121121121121211", "12112121111121121121121121211",
    "12112121112121121111121121211", "12112121111111221121121121211", "12112121112111221111121121211",
    "12112121111121221111121121211", "12112121111111122121121121211", "12112121112111122111121121211",
    "12112121111121122111121121211", "12112121111111222111121121211", "12112121112111111221121121211",
    "12112121111121111221121121211", "12112121112121111211121121211", "12112121111111211221121121211",
    "12112121112111211211121121211", "12112121111121211211121121211", "12112121111111112221121121211",
    "12112121112111112211121121211", "12112121111121112211121121211", "12112121111111212211121121211",
    "12112121112211111121121121211", "12112121111221111121121121211", "12112121112221111111121121211",
    "12112121111211211121121121211", "12112121112211211111121121211", "12112121111221211111121121211",
    "12112121111211112121121121211", "12112121112211112111121121211", "12112121112222222211121121211" // A-Z, *
};

const char code39_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-*";

// Function declarations
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight);
void init_motor_pins();
void move_forward(uint32_t gpioLeft, uint32_t gpioRight, float speed);
void straight_line_task(void *pvParameters);
void detect_surface_contrast_task(void *pvParameters);
uint16_t read_adc();
void reset_bar_data();
char decode_character();
void decode_barcode();
void display_captured_bars();

// Motor Control Code
void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight) {
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

void init_motor_pins() {
    gpio_init(L_MOTOR_DIR_PIN1);
    gpio_init(L_MOTOR_DIR_PIN2);
    gpio_init(R_MOTOR_DIR_PIN1);
    gpio_init(R_MOTOR_DIR_PIN2);
    gpio_set_dir(L_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(L_MOTOR_DIR_PIN2, GPIO_OUT);
    gpio_set_dir(R_MOTOR_DIR_PIN1, GPIO_OUT);
    gpio_set_dir(R_MOTOR_DIR_PIN2, GPIO_OUT);
}

void set_motor_speed(uint32_t gpio, float speed, bool is_left) {
    if (speed > 1.0) speed = 1.0;
    if (speed < 0.0) speed = 0.0;

    uint32_t duty_cycle = (uint32_t)(speed * MAX_DUTY_CYCLE);
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), is_left ? PWM_CHAN_A : PWM_CHAN_B, duty_cycle);
}

void move_forward(uint32_t gpioLeft, uint32_t gpioRight, float speed) {
    gpio_put(L_MOTOR_DIR_PIN1, 1);
    gpio_put(L_MOTOR_DIR_PIN2, 0);
    gpio_put(R_MOTOR_DIR_PIN1, 1);
    gpio_put(R_MOTOR_DIR_PIN2, 0);
    set_motor_speed(gpioLeft, speed, true);
    set_motor_speed(gpioRight, speed * 0.95, false);  // Slight adjustment for balance
}

void straight_line_task(void *pvParameters) {
    float move_speed = 0.7;  // 70% speed for forward movement
    while (true) {
        move_forward(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, move_speed);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Barcode Detection Code
volatile uint64_t last_transition_time = 0;
const char* previous_color = "White";
bool decoding_active = false;
bool initial_black_detected = false;
uint64_t bar_widths[MAX_BARS];
const char* bar_colors[MAX_BARS];
int bar_count = 0;
uint64_t max_width = 0;

uint16_t read_adc() {
    return adc_read();
}

void reset_bar_data() {
    for (int i = 0; i < MAX_BARS; i++) {
        bar_widths[i] = 0;
        bar_colors[i] = "Unknown";
    }
    bar_count = 0;
    max_width = 0;
    initial_black_detected = false;
}

void display_captured_bars() {
    printf("Captured Bar Widths (during decoding):\n");
    for (int i = 0; i < bar_count; i++) {
        printf("Bar #%d: Width = %llu us, Color = %s\n", i, bar_widths[i], bar_colors[i]);
    }
}

char classify_bar_width(uint64_t width, uint64_t max_width) {
    return (width <= max_width / 3) ? '1' : '2';
}

char decode_character() {
    char pattern[MAX_BARS + 1];
    char reverse_pattern[MAX_BARS + 1];

    for (int i = 0; i < bar_count; i++) {
        pattern[i] = classify_bar_width(bar_widths[i], max_width);
        reverse_pattern[bar_count - i - 1] = pattern[i];  // Reverse pattern
    }
    pattern[bar_count] = '\0';
    reverse_pattern[bar_count] = '\0';

    printf("Debug: Pattern (L->R): %s\n", pattern);
    printf("Debug: Pattern (R->L): %s\n", reverse_pattern);

    for (int i = 0; i < sizeof(code39_patterns) / sizeof(code39_patterns[0]); i++) {
        if (strcmp(pattern, code39_patterns[i]) == 0) {
            printf("Debug: Matched pattern L->R\n");
            return code39_chars[i];
        }
        if (strcmp(reverse_pattern, code39_patterns[i]) == 0) {
            printf("Debug: Matched pattern R->L\n");
            return code39_chars[i];
        }
    }
    printf("Debug: No matching pattern found\n");
    return '?';
}

void decode_barcode() {
    printf("Decoding barcode...\n");
    display_captured_bars();
    char decoded_char = decode_character();
    printf("Decoded Character: %c\n", decoded_char);

    reset_bar_data();
    decoding_active = false;
}

void detect_surface_contrast_task(void *pvParameters) {
    adc_init();
    adc_gpio_init(IR_SENSOR_PIN);
    adc_select_input(0);

    while (true) {
        uint32_t adc_sum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            adc_sum += read_adc();
            sleep_us(100);
        }
        uint16_t adc_value = adc_sum / NUM_SAMPLES;

        const char* current_color = (adc_value > THRESHOLD) ? "Black" : "White";

        if (!decoding_active && strcmp(current_color, "Black") == 0) {
            decoding_active = true;
            reset_bar_data();
            last_transition_time = time_us_64();  // Reset transition time to start fresh timing
            printf("Starting barcode detection\n");
        }

        if (decoding_active && !initial_black_detected && strcmp(current_color, "Black") == 0) {
            initial_black_detected = true;
            previous_color = "Black";
        }

        if (decoding_active && initial_black_detected) {
            if (strcmp(current_color, previous_color) != 0) {
                uint64_t current_time = time_us_64();
                uint64_t pulse_duration_us = current_time - last_transition_time;

                if (pulse_duration_us > NOISE_THRESHOLD + DEBOUNCE_DELAY && bar_count < MAX_BARS) {
                    bar_widths[bar_count] = pulse_duration_us;
                    bar_colors[bar_count] = previous_color;

                    printf("Captured Bar Color: %s\n", previous_color);
                    printf("Captured bar #%d: Width = %llu us\n", bar_count, pulse_duration_us);

                    if (pulse_duration_us > max_width) {
                        max_width = pulse_duration_us;
                        printf("Updated Max Width: %llu\n", max_width);
                    }
                    bar_count++;
                }

                if (bar_count == MAX_BARS) {
                    decode_barcode();
                }

                previous_color = current_color;
                last_transition_time = current_time;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay for next ADC reading
    }
}

// Main Function
int main() {
    stdio_init_all();
    setup_pwm(L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN);
    init_motor_pins();

    xTaskCreate(straight_line_task, "Straight Line Task", 256, NULL, 1, NULL);
    xTaskCreate(detect_surface_contrast_task, "Barcode Detection Task", 512, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true) {}
} 