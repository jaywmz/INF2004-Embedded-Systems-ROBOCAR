#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include <stdio.h>

// -------------------- Encoder and Motor Configuration --------------------
#define ENCODER_PIN 2       // Encoder signal input
#define MOTOR_PWM_PIN 15    // Motor PWM output pin
#define WHEEL_DIAMETER_MM 65  // Diameter of the wheel in mm
#define ENCODER_SLOTS 20    // Number of slots in the encoder

// Variables to track pulse count and speed
volatile uint32_t pulse_count = 0;
volatile uint64_t last_pulse_time = 0;
volatile float speed_mm_per_s = 0.0;
volatile float total_distance_m = 0.0;

// Constants for encoder
const float distance_per_pulse_mm = (WHEEL_DIAMETER_MM * 3.1416) / ENCODER_SLOTS;  // Distance per pulse in mm

// Timer for speed calculation
struct repeating_timer speed_timer;

// -------------------- Ultrasonic Sensor Configuration --------------------
const int timeout = 26100;  // Timeout for ultrasonic sensor (~4.5 meters)
const uint trigPin = 1;  // GP1 for Trigger pin
const uint echoPin = 0;  // GP0 for Echo pin

// -------------------- Encoder Callback Function --------------------
void encoder_callback(uint gpio, uint32_t events) {
    if (gpio == ENCODER_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        uint64_t current_time = time_us_64();
        uint64_t pulse_duration_us = current_time - last_pulse_time;
        last_pulse_time = current_time;

        // Increment pulse count
        pulse_count++;

        // Calculate speed in mm/s
        if (pulse_duration_us > 0) {
            float pulse_duration_s = pulse_duration_us / 1000000.0f;
            speed_mm_per_s = distance_per_pulse_mm / pulse_duration_s;  // Speed in mm/s
        }

        // Update total distance traveled in meters
        total_distance_m += distance_per_pulse_mm / 1000.0f;  // Convert mm to meters

        printf("Pulse Duration: %.2f ms, Pulses: %d, Speed: %.2f mm/s, Total Distance: %.2f meters\n", 
                pulse_duration_us / 1000.0f, pulse_count, speed_mm_per_s, total_distance_m);
    }
}

// -------------------- Timer Callback for Speed Updates --------------------
bool speed_timer_callback(struct repeating_timer *t) {
    printf("Speed Update -> Speed: %.2f mm/s, Total Distance: %.2f meters\n", speed_mm_per_s, total_distance_m);
    return true;
}

// -------------------- PWM Setup for Motor Control --------------------
void setup_pwm() {
    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    pwm_set_wrap(slice_num, 12500);  // 10ms period (100Hz PWM frequency)
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 6250);  // 50% duty cycle
    pwm_set_enabled(slice_num, true);
}

// -------------------- Ultrasonic Sensor Functions --------------------
void setupUltrasonicPins() {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);  // Trigger pin as output
    gpio_set_dir(echoPin, GPIO_IN);   // Echo pin as input
    gpio_put(trigPin, 0);  // Ensure trigger starts low
}

void sendTriggerPulse() {
    gpio_put(trigPin, 1);
    sleep_us(10);  // 10 microsecond pulse
    gpio_put(trigPin, 0);
    sleep_us(200);  // Small delay between pulses
}

uint64_t getPulse() {
    sendTriggerPulse();
    absolute_time_t startWaitTime = get_absolute_time();
    while (gpio_get(echoPin) == 0) {
        if (absolute_time_diff_us(startWaitTime, get_absolute_time()) > timeout) {
            return 0;  // Timeout
        }
    }
    absolute_time_t pulseStart = get_absolute_time();
    while (gpio_get(echoPin) == 1) {
        if (absolute_time_diff_us(pulseStart, get_absolute_time()) > timeout) {
            return 0;  // Timeout
        }
    }
    absolute_time_t pulseEnd = get_absolute_time();
    return absolute_time_diff_us(pulseStart, pulseEnd);
}

uint64_t getCm() {
    uint64_t pulseLength = getPulse();
    if (pulseLength == 0) {
        return 0;
    }
    double distance_cm = (pulseLength * 0.0343) / 2;
    if (distance_cm > 450 || distance_cm < 2) {
        return 0;  // Out of range
    }
    return (uint64_t)distance_cm;
}

// -------------------- Main Function --------------------
int main() {
    stdio_init_all();

    // Initialize encoder
    gpio_init(ENCODER_PIN);
    gpio_set_dir(ENCODER_PIN, GPIO_IN);
    gpio_pull_down(ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);

    // Initialize ultrasonic sensor
    setupUltrasonicPins();

    // Set up PWM for motor control
    setup_pwm();

    // Start a repeating timer to print speed and distance every second
    add_repeating_timer_ms(1000, speed_timer_callback, NULL, &speed_timer);

    // Main loop to measure ultrasonic distance and display it
    while (true) {
        uint64_t distance_cm = getCm();
        if (distance_cm > 0) {
            printf("Ultrasonic Distance: %llu cm\n", distance_cm);
        } else {
            printf("Ultrasonic: Out of range or timeout\n");
        }
        sleep_ms(1000);  // Wait before the next ultrasonic measurement
    }

    return 0;
}
