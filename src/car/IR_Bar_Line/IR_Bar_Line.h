#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>

// Define GPIO pins for motors
#define L_MOTOR_PWM_PIN 2
#define L_MOTOR_DIR_PIN1 0
#define L_MOTOR_DIR_PIN2 1
#define R_MOTOR_PWM_PIN 6
#define R_MOTOR_DIR_PIN1 4
#define R_MOTOR_DIR_PIN2 5

// PWM and speed settings
// #define MAX_DUTY_CYCLE 12500

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
#define MAX_WHITE_TIME 5000000

extern SemaphoreHandle_t xAdcMutex;

// Function declarations
// void setup_pwm(uint32_t gpioLeft, uint32_t gpioRight);
// void init_motor_pins();
// void move_forward(float speed);
// void turn_left(float speed);
// void turn_right(float speed);
// void stop_motors();
// void set_motor_speed(uint32_t gpio, float speed, bool is_left);

void detect_surface_contrast_task(void *pvParameters);
void line_following_task(void *pvParameters);

uint16_t read_adc(uint8_t input);
void reset_bar_data();
char decode_character();
void decode_barcode();
void display_captured_bars();
int levenshtein_distance(const char *s, const char *t);
char classify_bar_width(uint64_t width, uint64_t max_width);

#endif // LINE_FOLLOWER_H
