#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wheel_encoder.h"
#include "ultrasonic.h"  // Include ultrasonic header
#include "pid_motor.h"   // Include PID motor header

#define STOPPING_DISTANCE 20.0   // Target stopping distance
#define MOVE_DISTANCE_CM 90.0    // Distance to move forward after turning
#define MOVE_TIME_MS 5000        // Time to move forward in milliseconds

// Global variable to control motor state
bool ultrasonic_override = false;

// Function to rotate the car 90 degrees to the right
void rotate_90_degrees_right() {
    // Assuming the car can rotate 90 degrees to the right by running the motors in opposite directions for a specific time
    rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), duty_cycle_motor1);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), duty_cycle_motor2);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Adjust this delay to achieve a 90-degree turn
    stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
    stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
}

// Function to move the car forward for a specified distance over a specified time
void move_forward_distance(double distance_cm, int time_ms) {
    // Calculate the duty cycle needed to move the specified distance over the specified time
    int duty_cycle = (int)((distance_cm / MOVE_DISTANCE_CM) * 12500);  // Adjust this calculation as needed

    rotate_both_motors("forward", duty_cycle, duty_cycle);
    vTaskDelay(pdMS_TO_TICKS(time_ms));
    stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
    stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
}

// Function to check distance and override motor control if object is close
void check_ultrasonic_distance(void *pvParameters) {
    kalman_state *state = (kalman_state *)pvParameters;  // Retrieve Kalman filter state

    while (true) {
        double distance_cm = getCm(state);  // Get distance from ultrasonic sensor

        // Print the current distance for debugging
        printf("Measured Distance: %.2f cm\n", distance_cm);

        if (distance_cm > 0 && distance_cm <= STOPPING_DISTANCE) {
            // Object detected within stopping distance; set override to true and stop motors
            ultrasonic_override = true;
            stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
            stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
            printf("Object detected within %.2f cm. Stopping car.\n", distance_cm);

            // Rotate 90 degrees to the right
            rotate_90_degrees_right();

            // Move forward for 90 cm over 5 seconds
            move_forward_distance(MOVE_DISTANCE_CM, MOVE_TIME_MS);
        } else {
            // No object within range; disable override
            ultrasonic_override = false;
            rotate_both_motors("forward", duty_cycle_motor1, duty_cycle_motor2);
            printf("Moving forward. Distance: %.2f cm\n", distance_cm);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Delay to avoid constant checking (adjust as needed)
    }
}

int main() {
    stdio_init_all();  // Initialize serial communication

    // Initialize ultrasonic sensor and Kalman filter
    setupUltrasonicPins();
    double process_noise_covariance = 0.05;
    double measurement_noise_covariance = 50.0;
    double estimation_error_covariance = 1.0;
    double initial_value = STOPPING_DISTANCE;
    kalman_state *state = kalman_init(process_noise_covariance, measurement_noise_covariance, estimation_error_covariance, initial_value);

    // Initialize motor control pins and PWM for both motors
    setup_motor_pins(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    setup_pwm(MOTOR1_PWM_PIN);
    setup_motor_pins(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    setup_pwm(MOTOR2_PWM_PIN);

    // Create the ultrasonic check task
    xTaskCreate(check_ultrasonic_distance, "Ultrasonic Check Task", 1024, (void *)state, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    printf("Motors are ready for control.\n");

    // Main loop for manual control through keyboard input
    while (true) {
        // Read user input from the serial monitor
        char input = getchar_timeout_us(500000);  // Wait for input for 500ms

        // If ultrasonic override is active, skip manual control
        if (ultrasonic_override) {
            printf("Ultrasonic override active, motors stopped.\n");
            continue;
        }

        // Control for Motor 1
        if (input == 'f') {
            rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), duty_cycle_motor1);
            printf("Motor 1 rotating forward.\n");
        } else if (input == 'b') {
            rotate_motor("backward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), duty_cycle_motor1);
            printf("Motor 1 rotating backward.\n");
        } else if (input == '+') {
            if (duty_cycle_motor1 < 12500) {
                duty_cycle_motor1 += 1250;  // Increase speed by 10% for Motor 1
                set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), duty_cycle_motor1);
                printf("Increased speed for Motor 1. Current duty cycle: %d\n", duty_cycle_motor1);
            }
        } else if (input == '-') {
            if (duty_cycle_motor1 > 0) {
                duty_cycle_motor1 -= 1250;  // Decrease speed by 10% for Motor 1
                set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), duty_cycle_motor1);
                printf("Decreased speed for Motor 1. Current duty cycle: %d\n", duty_cycle_motor1);
            }
        } else if (input == 's') {
            stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
            printf("Motor 1 stopped.\n");
        }

        // Control for Motor 2
        if (input == 'F') {
            rotate_motor("forward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), duty_cycle_motor2);
            printf("Motor 2 rotating forward.\n");
        } else if (input == 'B') {
            rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), duty_cycle_motor2);
            printf("Motor 2 rotating backward.\n");
        } else if (input == '>') {
            if (duty_cycle_motor2 < 12500) {
                duty_cycle_motor2 += 1250;  // Increase speed by 10% for Motor 2
                set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), duty_cycle_motor2);
                printf("Increased speed for Motor 2. Current duty cycle: %d\n", duty_cycle_motor2);
            }
        } else if (input == '<') {
            if (duty_cycle_motor2 > 0) {
                duty_cycle_motor2 -= 1250;  // Decrease speed by 10% for Motor 2
                set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), duty_cycle_motor2);
                printf("Decreased speed for Motor 2. Current duty cycle: %d\n", duty_cycle_motor2);
            }
        } else if (input == 'S') {
            stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
            printf("Motor 2 stopped.\n");
        }

        // Control for Both Motors Together
        if (input == 'w') {
            rotate_both_motors("forward", duty_cycle_motor1, duty_cycle_motor2);
            printf("Both motors rotating forward.\n");
        } else if (input == 'x') {
            rotate_both_motors("backward", duty_cycle_motor1, duty_cycle_motor2);
            printf("Both motors rotating backward.\n");
        } else if (input == 'p') {
            stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
            stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
            printf("Both motors stopped.\n");
        }
    }

    return 0;
}