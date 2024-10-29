#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "../ultrasonic/ultrasonic.h"  // Include ultrasonic header (adjust path if necessary)

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
    printf("Starting ultrasonic sensor feedback test for 10 cm detection.\n");

    setupUltrasonicPins();  // Configure pins for the ultrasonic sensor

    // Tune Kalman filter parameters
    double process_noise_covariance = 0.05;  // Reduced process noise for smoother filtering
    double measurement_noise_covariance = 50.0;  // Adjusted for expected measurement noise
    double estimation_error_covariance = 1.0;
    double initial_value = 0.0;

    // Initialize Kalman filter with tuned parameters
    kalman_state *state = kalman_init(process_noise_covariance, measurement_noise_covariance, estimation_error_covariance, initial_value);

    // Create the ultrasonic test task
    xTaskCreate(ultrasonic_test_task, "Ultrasonic Test Task", 1024, (void *)state, 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Infinite loop (not reached due to FreeRTOS scheduler)
    while (1);
    return 0;
}
