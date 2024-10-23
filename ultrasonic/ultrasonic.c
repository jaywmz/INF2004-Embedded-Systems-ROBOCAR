#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Timeout value in microseconds (~26100us is equivalent to ~4.5 meters)
const int timeout = 26100;

// Function to initialize the trigger and echo pins for the ultrasonic sensor
void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);  // Set trigger pin as output
    gpio_set_dir(echoPin, GPIO_IN);   // Set echo pin as input
    gpio_put(trigPin, 0);  // Ensure trigger starts low
    printf("Pins initialized: Trigger = GPIO %d, Echo = GPIO %d\n", trigPin, echoPin);
}

// Function to send a 10µs pulse to the trigger pin
void sendTriggerPulse(uint trigPin) {
    gpio_put(trigPin, 1);
    sleep_us(10);  // 10 microsecond pulse
    gpio_put(trigPin, 0);
    sleep_us(200);  // Ensure a small delay between pulses (debouncing)
    printf("Trigger pulse sent on GPIO %d\n", trigPin);
}

// Function to get the echo pulse width in microseconds
uint64_t getPulse(uint trigPin, uint echoPin) {
    sendTriggerPulse(trigPin);  // Send the trigger pulse

    // Wait for the echo to go HIGH (pulse start)
    absolute_time_t startWaitTime = get_absolute_time();
    while (gpio_get(echoPin) == 0) {
        if (absolute_time_diff_us(startWaitTime, get_absolute_time()) > timeout) {
            printf("Timeout waiting for echo to start on GPIO %d\n", echoPin);
            return 0;  // Timeout, no pulse detected
        }
    }

    printf("Echo started on GPIO %d\n", echoPin);

    // Record the start time of the echo pulse
    absolute_time_t pulseStart = get_absolute_time();

    // Wait for the echo to go LOW (pulse end)
    while (gpio_get(echoPin) == 1) {
        if (absolute_time_diff_us(pulseStart, get_absolute_time()) > timeout) {
            printf("Timeout waiting for echo to end on GPIO %d\n", echoPin);
            return 0;  // Timeout, pulse lasted too long
        }
    }

    // Record the end time of the echo pulse
    absolute_time_t pulseEnd = get_absolute_time();
    printf("Echo ended on GPIO %d\n", echoPin);

    // Return the duration of the pulse in microseconds
    return absolute_time_diff_us(pulseStart, pulseEnd);
}

// Function to convert the pulse width to distance in centimeters
uint64_t getCm(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    if (pulseLength == 0) {
        printf("Error: Echo pulse length is 0 (timeout or out of range)\n");
        return 0;  // Handle timeout case
    }
    // Speed of sound in air is approximately 343 m/s or 0.0343 cm/µs
    // Distance = (pulse_duration / 2) * speed_of_sound
    double distance_cm = (pulseLength * 0.0343) / 2;

    // Optional: Validate the distance (e.g., to handle too-close or too-far cases)
    if (distance_cm > 450 || distance_cm < 2) {  // Example: discard out-of-range readings
        printf("Warning: Measured distance (%.2f cm) is out of range.\n", distance_cm);
        return 0;
    }

    return (uint64_t)distance_cm;
}

// Function to convert the pulse width to distance in inches
uint64_t getInch(uint trigPin, uint echoPin) {
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    if (pulseLength == 0) {
        printf("Error: Echo pulse length is 0 (timeout or out of range)\n");
        return 0;  // Handle timeout case
    }
    // Speed of sound in air in inches per microsecond is ~0.0133 in/µs
    return (pulseLength * 0.0133) / 2;
}

int main() {
    // Initialize stdio for serial output
    stdio_init_all();

    // Define the GPIO pins for the ultrasonic sensor
    const uint trigPin = 1;  // GP1 for Trigger pin
    const uint echoPin = 0;  // GP0 for Echo pin

    // Set up the ultrasonic sensor pins
    setupUltrasonicPins(trigPin, echoPin);

    // Main loop to continuously measure distance
    while (true) {
        uint64_t distance_cm = getCm(trigPin, echoPin);
        if (distance_cm > 0) {
            printf("Distance: %llu cm\n", distance_cm);
        } else {
            printf("Out of range or timeout\n");
        }

        // Wait 1 second before the next measurement
        sleep_ms(1000);
    }

    return 0;
}
