#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid_motor.h"
#include "wheel_ultra.h"  // Unified wheel encoder and ultrasonic functions

#define STOPPING_DISTANCE 10.0   // Distance in cm to stop when obstacle detected
#define MOVE_DISTANCE_CM 90.0    // Distance to move forward after turning
#define MOVE_TIME_MS 15000       // Time in ms to move forward after turn
#define TARGET_PULSES 10       // Adjust based on calibration for a 90-degree turn

// Function to rotate the car 90 degrees to the right with encoder feedback
void rotate_90_degrees_right() {
    printf("Starting 90-degree turn with encoder feedback\n");

    // Set motor directions for turning right
    rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);

    // Reset encoder message buffers
    reset_encoders();

    // Initial high power burst to start the turn
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), BURST_DUTY_CYCLE);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), BURST_DUTY_CYCLE);
    vTaskDelay(pdMS_TO_TICKS(BURST_DURATION_MS));

    // Reduce motor power for controlled turn
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), HIGH_POWER_DUTY);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), HIGH_POWER_DUTY);

    int total_left_pulses = 0;
    int total_right_pulses = 0;
    EncoderData left_data;
    EncoderData right_data;

    // Monitor encoder pulses until the target is reached
    while (total_left_pulses < TARGET_PULSES || total_right_pulses < TARGET_PULSES) {
        // Check for new data from the left encoder
        if (xMessageBufferReceive(left_buffer, &left_data, sizeof(left_data), pdMS_TO_TICKS(10)) > 0) {
            total_left_pulses = left_data.pulse_count;
            printf("Left pulses: %d\n", total_left_pulses);
        }

        // Check for new data from the right encoder
        if (xMessageBufferReceive(right_buffer, &right_data, sizeof(right_data), pdMS_TO_TICKS(10)) > 0) {
            total_right_pulses = right_data.pulse_count;
            printf("Right pulses: %d\n", total_right_pulses);
        }

        // Check if we have reached the target on both encoders
        if (total_left_pulses >= TARGET_PULSES && total_right_pulses >= TARGET_PULSES) {
            break;
        }

        // Brief delay to allow encoder readings to update
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Stop both motors once target pulses are reached
    stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
    stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
    printf("90-degree turn complete\n");
}



// Function to move the car forward a set distance
void move_forward_distance(double distance_cm, int time_ms) {
    printf("Moving forward %.2f cm\n", distance_cm);

    strong_start("forward");

    // Move at normal speed for remaining time
    int remaining_time = time_ms - BURST_DURATION_MS - HIGH_POWER_DURATION_MS;
    if (remaining_time > 0) {
        vTaskDelay(pdMS_TO_TICKS(remaining_time));
    }
    
    stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
    stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
    printf("Forward movement complete\n");
}

// Task to check ultrasonic distance and control movement
void check_ultrasonic_distance(void *pvParameters) {
    kalman_state *state = (kalman_state *)pvParameters;
    bool turning_flag = false;
    bool first_start = true;

    pid_init(&distance_pid, 225.0, 6.0, 35.0, TARGET_DISTANCE, MIN_DUTY_CYCLE, NORMAL_DUTY_CYCLE);

    while (true) {
        double distance_cm = getCm(state);  // Get distance from ultrasonic sensor
        printf("Measured Distance: %.2f cm\n", distance_cm);

        if (distance_cm > 0) {
            if (distance_cm <= STOPPING_DISTANCE) {
                if (!turning_flag) {
                    stop_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN));
                    stop_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN));
                    printf("Target reached. Distance: %.2f cm\n", distance_cm);

                    turning_flag = true;
                    vTaskDelay(pdMS_TO_TICKS(500));

                    pid_reset(&distance_pid);
                    rotate_90_degrees_right();
                    move_forward_distance(MOVE_DISTANCE_CM, MOVE_TIME_MS);

                    turning_flag = false;
                    first_start = true;
                }
            } else if (!turning_flag) {
                if (first_start) {
                    strong_start("forward");
                    first_start = false;
                } else {
                    rotate_both_motors("forward", NORMAL_DUTY_CYCLE, NORMAL_DUTY_CYCLE);
                }
                printf("Normal movement. Distance: %.2f cm\n", distance_cm);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
    stdio_init_all();

    // Initialize sensors and tasks
    system_init();  // Initializes ultrasonic and encoder sensors

    // Motor setup
    setup_motor_pins(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    setup_pwm(MOTOR1_PWM_PIN);
    setup_motor_pins(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    setup_pwm(MOTOR2_PWM_PIN);

    kalman_state *state = kalman_init(0.05, 50.0, 1.0, STOPPING_DISTANCE);

    // Create the task for checking ultrasonic distance and controlling the car
    xTaskCreate(check_ultrasonic_distance, "Ultrasonic Check Task", 1024, (void *)state, 1, NULL);

    vTaskStartScheduler();  // Start the FreeRTOS scheduler

    while (true) {
        tight_loop_contents();  // Keeps the main loop active
    }

    return 0;
}
