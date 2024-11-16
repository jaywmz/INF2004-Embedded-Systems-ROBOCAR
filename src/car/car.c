#include "FreeRTOS.h"
#include "encoder.h"
#include "motor.h"
#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>
#include <math.h>

#include "hardware/pwm.h"

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

// Global encoder data structures for left and right wheels
EncoderData motor1_encoder_data, motor2_encoder_data;
// Kalman filter state for ultrasonic distance measurements
KalmanState kalman_state;

// PID controllers for left and right motors
PIDController pid_motor_1, pid_motor_2;

// Global flag to indicate when the car should stop based on ultrasonic readings
int PLS_STOP = 0;

// Constants for movement and stopping distances
#define STOPPING_DISTANCE 10.0 // Distance in cm to stop when obstacle detected
#define MOVE_DISTANCE_CM 82.0  // Distance to move forward after turning
// #define MOVE_TIME_MS 1000      // Time in ms to move forward after turn
#define TARGET_PULSES 21           // Adjust based on calibration for a 90-degree turn
#define DISTANCE_PER_NOTCH 0.0102 // in meters, or 1.02 cm

// Function to rotate the car 90 degrees to the right with encoder feedback
void rotate_90_degrees(int target_pulses)
{
    // Initial high power burst to start the turn
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), BURST_DUTY_CYCLE);

    motor1_encoder_data.pulse_count = 0;

    // Monitor encoder pulses until the target is reached
    while (motor1_encoder_data.pulse_count < target_pulses)
    {
        // Check if we have reached the target on both encoders
        if (motor1_encoder_data.pulse_count >= target_pulses)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(8)); // Small delay for stability
    }

    // Stop both motors once target pulses are reached
    stop_motors();
}

// Helper function to set up a 90-degree right turn by configuring direction and rotating
void rotate_90_degrees_right()
{
    // Set motor directions for turning right
    set_direction(GO_BACKWARD, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    rotate_90_degrees(10); // Rotate with target pulses to achieve 90-degree turn
}

// Function to move the car forward a specified distance in centimeters
void move_forward_distance(float target_distance_cm)
{
    // Calculate target pulses based on target distance and encoder settings
    int target_pulses = (int)(((target_distance_cm / 100) / DISTANCE_PER_NOTCH) + 0.5);

    // Initialize encoder pulse counts
    motor2_encoder_data.pulse_count = 0;
    motor1_encoder_data.pulse_count = 0;

    // Start moving forward with an initial strong start
    strong_start(GO_FORWARD);

    while (motor2_encoder_data.pulse_count < target_pulses || motor1_encoder_data.pulse_count < target_pulses)
    {
        // Calculate an average target pulse count to keep both motors aligned
        int average_pulse_count = (motor2_encoder_data.pulse_count + motor1_encoder_data.pulse_count) / 2;
        pid_motor_1.setpoint = average_pulse_count;
        pid_motor_2.setpoint = average_pulse_count;

        // Compute adjustments based on the current pulse count vs. the target
        int motor2_dutyCycle = pid_update(&pid_motor_1, motor2_encoder_data.pulse_count);
        int motor1_dutyCycle = pid_update(&pid_motor_2, motor1_encoder_data.pulse_count);

        // Apply PID adjustments to each motor, plus any necessary bias
        motor1_forward(NORMAL_DUTY_CYCLE + motor2_dutyCycle - 400);
        motor2_forward(NORMAL_DUTY_CYCLE + motor1_dutyCycle); // Adjust bias if needed

        // Print debug information about target and actual pulse counts
        printf("Target pulses needed: %d\n", target_pulses);
        printf("Left encoder data pulse count: %d\n", motor2_encoder_data.pulse_count);
        printf("Right encoder data pulse count: %d\n", motor1_encoder_data.pulse_count);

        // Break if both motors have reached or exceeded the target pulses
        if (motor2_encoder_data.pulse_count >= target_pulses && motor1_encoder_data.pulse_count >= target_pulses)
        {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay for stability
    }

    // Stop motors after reaching the target distance
    stop_motors();
}

// Function to adjust motor speeds dynamically to maintain straight movement using PID
void adjust_motor_speeds_with_pid()
{
    // Try to implement this, should start fast then slow down (Method in motor.c)
    // static bool initialized = false; // Flag to track if strong start has been used
    // if (!initialized)
    // {
    //     strong_start(GO_FORWARD); // Perform strong start if not done yet
    //     initialized = true;          // Mark as initialized to avoid repeating strong start
    // }

    // Use a set speed as setpoint for PIDs
    const float setSpeed = 100.0; // cm/s
    printf("motor1 encoder speed: %f, motor2 encoder speed: %f", motor1_encoder_data.speed_cm_per_s, motor2_encoder_data.speed_cm_per_s);
    pid_motor_1.setpoint = setSpeed;
    pid_motor_2.setpoint = setSpeed;

    // Compute adjustments based on the current pulse count vs. the target
    int motor1_dutyCycle = pid_update(&pid_motor_1, motor1_encoder_data.speed_cm_per_s);
    int motor2_dutyCycle = pid_update(&pid_motor_2, motor2_encoder_data.speed_cm_per_s);

    // printf("L-epc: %d | R-epc: %d | L-dca: %d | R-dca: %d\n", left_encoder_data.pulse_count, right_encoder_data.pulse_count, left_duty_adjustment, right_duty_adjustment);
    printf("    motor1-dc: %d | motor2-dc: %d\n", motor1_dutyCycle, motor2_dutyCycle);

    // Apply adjustments to maintain straight movement
    motor1_forward(motor1_dutyCycle);
    motor2_forward(motor2_dutyCycle); 
}

// Encoder task for reading encoder data and updating encoder structures
void vTaskEncoder(__unused void *pvParameters)
{
    encoder_kalman_init(&motor2_encoder_data.kalman_state, 0.1, 0.1, 1.0, 0.0);
    encoder_kalman_init(&motor1_encoder_data.kalman_state, 0.1, 0.1, 1.0, 0.0);
    while (1)
    {
        read_encoder_data(LEFT_ENCODER_PIN, &motor2_encoder_data);
        read_encoder_data(RIGHT_ENCODER_PIN, &motor1_encoder_data);
    }
}

// Motor control task to adjust motor speeds based on ultrasonic sensor feedback
void vTaskMotor(__unused void *pvParameters)
{
    int state = 0;
    while (1)   
    {
        if (PLS_STOP == 1) // If an obstacle is detected
        {
            stop_motors();
            vTaskDelay(pdMS_TO_TICKS(2000));
            // turn right
            rotate_90_degrees_right();
            vTaskDelay(pdMS_TO_TICKS(2000));

            // code to move forward 90 cm
            move_forward_distance(MOVE_DISTANCE_CM);
            stop_motors();
            break;
        }
        else
        {
            adjust_motor_speeds_with_pid(); // Continue with PID adjustments
        }
    }
}

// Ultrasonic sensor task to measure distance and update `PLS_STOP` flag if obstacle is within stopping distance
void vTaskUltrasonic(__unused void *pvParameters)
{
    kalman_state.q = 0.1;
    kalman_state.r = 0.1;
    kalman_state.p = 1.0;
    kalman_state.x = 0.0;

    while (1)
    {
        // Trigger ultrasonic sensor
        set_trigger_pin(1);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_trigger_pin(0);

        // Measure pulse length and calculate distance
        uint64_t pulse_length = get_pulse_length();
        double distance = get_cm(&kalman_state, pulse_length);

        // Set or clear stop flag based on distance
        if (distance < STOPPING_DISTANCE)
        {
            PLS_STOP = 1;
        }
        else
        {
            PLS_STOP = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay for next measurement
    }
}

// Function to launch all tasks and start the FreeRTOS scheduler
void vLaunch()
{
    TaskHandle_t motorTask, encoderTask, distanceTask;
    xTaskCreate(vTaskEncoder, "EncoderTask", 2048, NULL, tskIDLE_PRIORITY + 1UL, &encoderTask);
    xTaskCreate(vTaskMotor, "MotorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &motorTask);
    // xTaskCreate(vTaskUltrasonic, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, &distanceTask);
    vTaskStartScheduler(); // Start FreeRTOS scheduler
}

// Main function for system initialization and launching tasks
int main(void)
{
    stdio_init_all();  // Initialize standard I/O for debugging
    init_encoder();    // Initialize encoders
    init_motors();     // Initialize motors
    init_ultrasonic(); // Initialize ultrasonic sensor

    // Initialize PID controllers for both motors
    pid_init(&pid_motor_1, 2, 1, 0.5, 0.0, 0, MAX_DUTY_CYCLE); // Left motor PID
    pid_init(&pid_motor_2, 5, 1, 1, 0.0, 0, MAX_DUTY_CYCLE); // Right motor PID

    sleep_ms(1000); // Small delay to ensure system is ready
    vLaunch();      // Launch tasks and start FreeRTOS scheduler
    while (1)
    {
        // Should never reach here
    }
    return 0;
}
