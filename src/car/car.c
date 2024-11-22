#include "FreeRTOS.h"
#include "encoder.h"
#include "motor.h"
#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>
#include <math.h>
#include "udp.h"
#include "barcode.h"
#include "linefollow.h"
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
    // Compute new duty cycle based on the current pulse count vs. the target
    float motor1_dutyCycle = pid_update(&pid_motor_1, motor1_encoder_data.speed);
    float motor2_dutyCycle = pid_update(&pid_motor_2, motor2_encoder_data.speed);

    // printf("motor1-dc: %f | motor2-dc: %f\n", motor1_dutyCycle, motor2_dutyCycle);

    // Apply new duty cycle to maintain straight movement
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
        read_encoder_data(MOTOR2_ENCODER_PIN, &motor2_encoder_data);
        read_encoder_data(MOTOR1_ENCODER_PIN, &motor1_encoder_data);
    }
}

// Moving average function to get average speed
float movingAvg(bool motor1, float newSpeed) {
    const int len = 5;
    static float speedsMotor1[10] = {0.0};
    static float speedsMotor2[10] = {0.0};
    static int index1 = 0;
    static int index2 = 0;
    static float sum1 = 0;
    static float sum2 = 0;

    if (motor1) {
        sum1 = sum1 - speedsMotor1[index1] + newSpeed;

        speedsMotor1[index1] = newSpeed;

        index1++;
        if (index1 >= len - 1) {index1 = 0;}

        return sum1 / len;
    }
    else {
        sum2 = sum2 - speedsMotor2[index2] + newSpeed;

        speedsMotor2[index2] = newSpeed;

        index2++;
        if (index2 >= len - 1) {index2 = 0;}

        return sum2 / len;
    }
}

// Task to measure speed (pulses per second)
void encoder_task(void *pvParameters) {
    int prev_pulses_motor1 = 0, prev_pulses_motor2 = 0;

    while (1) {
        // Find number of pulses since last interval
        int pulses_motor1 = motor1_encoder_data.pulse_count - prev_pulses_motor1;
        int pulses_motor2 = motor2_encoder_data.pulse_count - prev_pulses_motor2;
        prev_pulses_motor1 = motor1_encoder_data.pulse_count;
        prev_pulses_motor2 = motor2_encoder_data.pulse_count;

        // Calculate speed, since interval is 100ms, multiply 10 to get pulses per 1000ms/1s
        float speed_motor1 = (float)pulses_motor1 * 5;
        float speed_motor2 = (float)pulses_motor2 * 5;

        // Get moving average of speed and save to encoder structs
        motor1_encoder_data.speed = movingAvg(true, speed_motor1);
        motor2_encoder_data.speed = movingAvg(false, speed_motor2);

        printf("motor1 speed: %f | motor2 speed: %f | ", motor1_encoder_data.speed, motor2_encoder_data.speed);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS)); // Wait for next sample
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
            // Continually move car and maintain speed with PID adjustments
            // adjust_motor_speeds_with_pid();

            // Compute new duty cycle based on the current pulse count vs. the target
            float motor1_dutyCycle = pid_update(&pid_motor_1, motor1_encoder_data.speed);
            float motor2_dutyCycle = pid_update(&pid_motor_2, motor2_encoder_data.speed);
            // float motor1_dutyCycle = 0.8;
            // float motor2_dutyCycle = 0.8;

            // Apply new duty cycle to maintain straight movement
            motor1_forward(motor1_dutyCycle);
            motor2_forward(motor2_dutyCycle);

            printf("motor1-dc: %f | motor2-dc: %f\n", motor1_dutyCycle, motor2_dutyCycle);

            vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS / 2)); // Wait for next sample
        }

//         if (compass.direction == 'f')
//         {
//             printf("Moving forward\n");
//             move_forward();
//         }
//         else if (compass.direction == 'b')
//         {
//             printf("Moving backward\n");
//             move_backward();
//         }
//         else if (compass.direction == 'l')
//         {
//             printf("Turning left\n");
//             motor1_backward(compass.left_duty);
//             motor2_forward(compass.right_duty);
//         }
//         else if (compass.direction == 'r')
//         {
//             printf("Turning right\n");
//             motor1_forward(compass.left_duty);
//             motor2_backward(compass.right_duty);
//         }
//         else if (compass.direction == 's')
//         {
//             printf("Stopping\n");
//             stop_motors();
//         }
//         else
//         {
//             printf("Invalid direction\n");
//             stop_motors();
//         }
    }
}

void vTaskCompass(__unused void *pvParameters)
{
    while (1)
    {
        get_compass_data(&compass);
        vTaskDelay(pdMS_TO_TICKS(100));
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
    TaskHandle_t motorTask, compassTask, encoderTask, distanceTask;
    xTaskCreate(vTaskCompass, "CompassTask", 2048, NULL, tskIDLE_PRIORITY + 1UL, &compassTask);
    xTaskCreate(encoder_task, "EncoderTask", 2048, NULL, tskIDLE_PRIORITY + 1UL, &encoderTask);
    xTaskCreate(vTaskMotor, "MotorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &motorTask);
    xTaskCreate(vTaskUltrasonic, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, &distanceTask);
    vTaskStartScheduler();
}

// Main function for system initialization and launching tasks
int main(void)
{
    stdio_init_all();  // Initialize standard I/O for debugging
    init_udp();
    init_encoder();    // Initialize encoders
    init_motors();     // Initialize motors
    init_ultrasonic(); // Initialize ultrasonic sensor

    // Initialize PID controllers for both motors
    float kp1 = 0.006; float kp2 = 0.01;
    float ki1 = 0.01; float ki2 = 0.01;
    float kd1 = 0.014; float kd2 = 0.010;
    float setpoint = 25.0;  // speed of 34 pulses per second
    pid_init(&pid_motor_1, kp1, ki1, kd1, setpoint, 0.0, 1.0); // Left motor PID
    pid_init(&pid_motor_2, kp2, ki2, kd2, setpoint, 0.0, 1.0); // Right motor PID

    sleep_ms(1000); // Small delay to ensure system is ready
    vLaunch();      // Launch tasks and start FreeRTOS scheduler

    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}
