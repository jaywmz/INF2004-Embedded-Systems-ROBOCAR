#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wheel_encoder.h"
#include "ultrasonic.h"  // Include ultrasonic header (adjust path if necessary)
#include "pid_motor.h"   // Include PID motor header (adjust path if necessary)


#define STOPPING_DISTANCE 10.0   // Target stopping distance
#define BUFFER_ZONE 2.0          // Buffer zone around the target distance (e.g., ±2 cm)

// Task to test and maintain ultrasonic sensor feedback at 10 cm
void ultrasonic_test_task(void *pvParameters) {
    kalman_state *state = (kalman_state *)pvParameters;  // Retrieve Kalman filter state

    while (true) {
        double distance_cm = getCm(state);  // Get distance from ultrasonic sensor

        // Print the current distance for debugging
        printf("Measured Distance: %.2f cm\n", distance_cm);

        // Check if distance is within the buffer zone around the target distance
        if (distance_cm > 0 && distance_cm <= (STOPPING_DISTANCE + BUFFER_ZONE) &&
            distance_cm >= (STOPPING_DISTANCE - BUFFER_ZONE)) {
            printf("Object is within buffer zone around %.2f cm. Continuing detection...\n", STOPPING_DISTANCE);
        } else if (distance_cm > 0 && distance_cm < (STOPPING_DISTANCE - BUFFER_ZONE)) {
            printf("Object too close! Distance: %.2f cm\n", distance_cm);
        } else if (distance_cm > (STOPPING_DISTANCE + BUFFER_ZONE)) {
            printf("Object too far! Distance: %.2f cm\n", distance_cm);
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Loop delay set to 50 ms for frequent updates
    }
}

int main() {
    stdio_init_all();  // Initialize serial communication
    // ultrasonic test
    // printf("Starting ultrasonic sensor feedback test for 10 cm detection.\n");

    // setupUltrasonicPins();  // Configure pins for the ultrasonic sensor

    // // Tune Kalman filter parameters
    // double process_noise_covariance = 0.05;  // Reduced process noise for smoother filtering
    // double measurement_noise_covariance = 50.0;  // Adjusted for expected measurement noise
    // double estimation_error_covariance = 1.0;
    // double initial_value = 10;

    // // Initialize Kalman filter with tuned parameters
    // kalman_state *state = kalman_init(process_noise_covariance, measurement_noise_covariance, estimation_error_covariance, initial_value);

    // // Create the ultrasonic test task
    // xTaskCreate(ultrasonic_test_task, "Ultrasonic Test Task", 1024, (void *)state, 1, NULL);

    // // Start the FreeRTOS scheduler
    // vTaskStartScheduler();

    // // Infinite loop (not reached due to FreeRTOS scheduler)
    // while (1);
    // return 0;

    // wheel_encoder test
    // Initialize the encoder pins
    // gpio_init(LEFT_ENCODER_PIN);
    // gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    // gpio_pull_down(LEFT_ENCODER_PIN);

    // gpio_init(RIGHT_ENCODER_PIN);
    // gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    // gpio_pull_down(RIGHT_ENCODER_PIN);

    // // Create message buffers
    // left_buffer = xMessageBufferCreate(10 * sizeof(EncoderData));
    // right_buffer = xMessageBufferCreate(10 * sizeof(EncoderData));

    // if (left_buffer == NULL || right_buffer == NULL) {
    //     printf("Failed to create message buffers\n");
    //     return 1;
    // }

    // // Set interrupts for both encoders
    // gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &left_encoder_callback);
    // gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &right_encoder_callback);

    // // Create tasks for each wheel's processing
    // xTaskCreate(process_left_encoder_task, "LeftEncoderTask", 1024, NULL, 1, &left_encoder_task_handle);
    // xTaskCreate(process_right_encoder_task, "RightEncoderTask", 1024, NULL, 1, &right_encoder_task_handle);

    // // Start the FreeRTOS scheduler
    // vTaskStartScheduler();

    // while (true) {
    //     tight_loop_contents();
    // }

    // return 0;

    // pid_motor test
    sleep_ms(1000);  // Allow time for USB to initialize

    // Set up motor control pins and PWM for both motors
    setup_motor_pins(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    setup_pwm(MOTOR1_PWM_PIN);
    setup_motor_pins(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    setup_pwm(MOTOR2_PWM_PIN);

    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);

    printf("Motors are ready for control.\n");

    while (true) {
        // Read user input from the serial monitor
        char input = getchar_timeout_us(500000);  // Wait for input for 500ms

        // Control for Motor 1
        if (input == 'f') {
            rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(slice_num_motor1, duty_cycle_motor1);
            printf("Motor 1 rotating forward.\n");
        } else if (input == 'b') {
            rotate_motor("backward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(slice_num_motor1, duty_cycle_motor1);
            printf("Motor 1 rotating backward.\n");
        } else if (input == '+') {
            if (duty_cycle_motor1 < 12500) {
                duty_cycle_motor1 += 1250;  // Increase speed by 10% for Motor 1
                set_motor_speed(slice_num_motor1, duty_cycle_motor1);
                printf("Increased speed for Motor 1. Current duty cycle: %d\n", duty_cycle_motor1);
            }
        } else if (input == '-') {
            if (duty_cycle_motor1 > 0) {
                duty_cycle_motor1 -= 1250;  // Decrease speed by 10% for Motor 1
                set_motor_speed(slice_num_motor1, duty_cycle_motor1);
                printf("Decreased speed for Motor 1. Current duty cycle: %d\n", duty_cycle_motor1);
            }
        } else if (input == 's') {
            stop_motor(slice_num_motor1);
            printf("Motor 1 stopped.\n");
        }

        // Control for Motor 2
        if (input == 'F') {
            rotate_motor("forward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(slice_num_motor2, duty_cycle_motor2);
            printf("Motor 2 rotating forward.\n");
        } else if (input == 'B') {
            rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(slice_num_motor2, duty_cycle_motor2);
            printf("Motor 2 rotating backward.\n");
        } else if (input == '>') {
            if (duty_cycle_motor2 < 12500) {
                duty_cycle_motor2 += 1250;  // Increase speed by 10% for Motor 2
                set_motor_speed(slice_num_motor2, duty_cycle_motor2);
                printf("Increased speed for Motor 2. Current duty cycle: %d\n", duty_cycle_motor2);
            }
        } else if (input == '<') {
            if (duty_cycle_motor2 > 0) {
                duty_cycle_motor2 -= 1250;  // Decrease speed by 10% for Motor 2
                set_motor_speed(slice_num_motor2, duty_cycle_motor2);
                printf("Decreased speed for Motor 2. Current duty cycle: %d\n", duty_cycle_motor2);
            }
        } else if (input == 'S') {
            stop_motor(slice_num_motor2);
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
            stop_motor(slice_num_motor1);
            stop_motor(slice_num_motor2);
            printf("Both motors stopped.\n");
        }

        sleep_ms(100);  // Small delay to ensure smooth control
    }

}
