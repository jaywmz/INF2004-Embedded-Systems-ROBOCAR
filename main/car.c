#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "wheel_encoder.h"
#include "ultrasonic.h"  // Include ultrasonic header (adjust path if necessary)

#define STOPPING_DISTANCE 10.0   // Target stopping distance
#define BUFFER_ZONE 2.0          // Buffer zone around the target distance (e.g., ±2 cm)

// Task to test ultrasonic sensor feedback
void ultrasonic_test_task(void *pvParameters) {
    kalman_state *state = (kalman_state *)pvParameters;  // Retrieve Kalman filter state

    while (true) {
        double distance_cm = getCm(state);  // Get distance from ultrasonic sensor

        // Print the current distance for debugging
        printf("Measured Distance: %.2f cm\n", distance_cm);

        // Check if distance is within the buffer zone of the stopping distance
        if (distance_cm > 0 && distance_cm <= (STOPPING_DISTANCE + BUFFER_ZONE) &&
            distance_cm >= (STOPPING_DISTANCE - BUFFER_ZONE)) {
            printf("Object detected within buffer zone around %.2f cm. Test complete.\n", STOPPING_DISTANCE);
            break;  // Exit the loop once the object is detected within the buffer range
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // Loop delay set to 50 ms for frequent updates
    }
}

int main() {
    stdio_init_all();  // Initialize serial communication
    // printf("Starting ultrasonic sensor feedback test for 10 cm detection.\n");

    // setupUltrasonicPins();  // Configure pins for the ultrasonic sensor

    // // Tune Kalman filter parameters
    // double process_noise_covariance = 0.05;  // Reduced process noise for smoother filtering
    // double measurement_noise_covariance = 50.0;  // Adjusted for expected measurement noise
    // double estimation_error_covariance = 1.0;
    // double initial_value = 0.0;

    // // Initialize Kalman filter with tuned parameters
    // kalman_state *state = kalman_init(process_noise_covariance, measurement_noise_covariance, estimation_error_covariance, initial_value);

    // // Create the ultrasonic test task
    // xTaskCreate(ultrasonic_test_task, "Ultrasonic Test Task", 1024, (void *)state, 1, NULL);

    // // Start the FreeRTOS scheduler
    // vTaskStartScheduler();

    // // Infinite loop (not reached due to FreeRTOS scheduler)
    // while (1);
    // return 0;

    // Initialize the encoder pins
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_down(LEFT_ENCODER_PIN);

    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_down(RIGHT_ENCODER_PIN);

    // Create message buffers
    left_buffer = xMessageBufferCreate(10 * sizeof(EncoderData));
    right_buffer = xMessageBufferCreate(10 * sizeof(EncoderData));

    if (left_buffer == NULL || right_buffer == NULL) {
        printf("Failed to create message buffers\n");
        return 1;
    }

    // Set interrupts for both encoders
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &left_encoder_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &right_encoder_callback);

    // Create tasks for each wheel's processing
    xTaskCreate(process_left_encoder_task, "LeftEncoderTask", 1024, NULL, 1, &left_encoder_task_handle);
    xTaskCreate(process_right_encoder_task, "RightEncoderTask", 1024, NULL, 1, &right_encoder_task_handle);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    while (true) {
        tight_loop_contents();
    }

    return 0;
}
