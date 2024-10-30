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

// Update rotate_90_degrees_right to use the 35% speed:
void rotate_90_degrees_right() {
    printf("Executing 90-degree right turn\n");
    rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), NORMAL_DUTY_CYCLE);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), NORMAL_DUTY_CYCLE);
    vTaskDelay(pdMS_TO_TICKS(1000));
    stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
    stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
    printf("Turn complete\n");
}

// Update move_forward_distance to use the 35% speed:
void move_forward_distance(double distance_cm, int time_ms) {
    printf("Moving forward %.2f cm\n", distance_cm);
    rotate_both_motors("forward", NORMAL_DUTY_CYCLE, NORMAL_DUTY_CYCLE);
    vTaskDelay(pdMS_TO_TICKS(time_ms));
    stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
    stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
    printf("Forward movement complete\n");
}

void check_ultrasonic_distance(void *pvParameters) {
    kalman_state *state = (kalman_state *)pvParameters;
    
    // Initialize PID controller with gentler gains
    pid_init(&distance_pid, 
            225.0,   // Proportional gain
            6.0,     // Integral gain
            35.0,    // Derivative gain
            TARGET_DISTANCE,
            MIN_DUTY_CYCLE,
            NORMAL_DUTY_CYCLE
    );

    bool turning_flag = false;  // Add flag to prevent multiple turns

    while (true) {
        double distance_cm = getCm(state);
        printf("Measured Distance: %.2f cm\n", distance_cm);

        if (distance_cm > 0) {  // Valid distance reading
            if (distance_cm <= START_CONTROL_DISTANCE) {  // Within control distance
                if (distance_cm <= TARGET_DISTANCE + STOP_TOLERANCE && !turning_flag) {
                    // Stop motors first
                    stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
                    stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
                    printf("TARGET REACHED - STOPPING. Distance: %.2f cm\n", distance_cm);
                    
                    // Set turning flag and small delay to ensure complete stop
                    turning_flag = true;
                    vTaskDelay(pdMS_TO_TICKS(500));
                    
                    printf("Starting turn sequence\n");
                    pid_reset(&distance_pid);
                    rotate_90_degrees_right();
                    move_forward_distance(MOVE_DISTANCE_CM, MOVE_TIME_MS);
                    
                    // Reset turning flag after movement complete
                    turning_flag = false;
                } else if (!turning_flag) {
                    // Calculate motor speed using PID
                    double error = distance_cm - TARGET_DISTANCE;
                    double pid_output = pid_update(&distance_pid, distance_cm);
                    int new_duty_cycle = (int)pid_output;
                    
                    // Ensure duty cycle is within bounds
                    if (new_duty_cycle < MIN_DUTY_CYCLE) {
                        new_duty_cycle = MIN_DUTY_CYCLE;
                    } else if (new_duty_cycle > NORMAL_DUTY_CYCLE) {
                        new_duty_cycle = NORMAL_DUTY_CYCLE;
                    }
                    
                    duty_cycle_motor1 = new_duty_cycle;
                    duty_cycle_motor2 = new_duty_cycle;
                    
                    rotate_both_motors("forward", duty_cycle_motor1, duty_cycle_motor2);
                    printf("PID control active. Distance: %.2f cm, Error: %.2f, Duty Cycle: %d\n", 
                           distance_cm, error, new_duty_cycle);
                }
            } else if (!turning_flag) {
                // When beyond control distance, move at normal speed (35%)
                rotate_both_motors("forward", NORMAL_DUTY_CYCLE, NORMAL_DUTY_CYCLE);
                printf("Normal speed (35%%). Distance: %.2f cm, Duty Cycle: %d\n", 
                       distance_cm, NORMAL_DUTY_CYCLE);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
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