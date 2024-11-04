#include "FreeRTOS.h"
#include "encoder.h"
#include "motor.h"
#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>

#include "hardware/pwm.h"

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

EncoderData left_encoder_data, right_encoder_data;
KalmanState kalman_state;

PIDController pid_motor_1, pid_motor_2;

int PLS_STOP = 0;

#define STOPPING_DISTANCE 10.0 // Distance in cm to stop when obstacle detected
#define MOVE_DISTANCE_CM 90.0  // Distance to move forward after turning
// #define MOVE_TIME_MS 1000      // Time in ms to move forward after turn
#define TARGET_PULSES 8           // Adjust based on calibration for a 90-degree turn
#define DISTANCE_PER_NOTCH 0.0102 // in meters, or 1.02 cm

// Function to rotate the car 90 degrees to the right with encoder feedback
void rotate_90_degrees(int target_pulses)
{
    // Initial high power burst to start the turn
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), BURST_DUTY_CYCLE);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), BURST_DUTY_CYCLE);

    int total_left_pulses = 0;
    int total_right_pulses = 0;

    left_encoder_data.pulse_count = 0;
    right_encoder_data.pulse_count = 0;

    // Monitor encoder pulses until the target is reached
    while (total_left_pulses < target_pulses || total_right_pulses < target_pulses)
    {
        // Check for new data from the left encoder
        total_left_pulses = left_encoder_data.pulse_count;

        // Check for new data from the right encoder
        total_right_pulses = right_encoder_data.pulse_count;

        // Check if we have reached the target on both encoders
        if (total_left_pulses >= target_pulses && total_right_pulses >= target_pulses)
        {
            break;
        }
        // printf("Left Pulses: %d\n", total_left_pulses);
        // printf("Right Pulses: %d\n", total_right_pulses);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // // Stop both motors once target pulses are reached
    stop_motors();
}

void rotate_90_degrees_right()
{
    // Set motor directions for turning right
    set_direction(GO_BACKWARD, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    set_direction(GO_FORWARD, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    rotate_90_degrees(8);
}

void move_forward_distance(float target_distance_cm)
{
    // Calculate target pulses based on target distance and encoder settings
    int target_pulses = (int)((target_distance_cm / DISTANCE_PER_NOTCH) + 0.5);

    // Initialize encoder pulse counts
    left_encoder_data.pulse_count = 0;
    right_encoder_data.pulse_count = 0;

    // Start moving forward
    move_forward();

    // Monitor the encoder pulses until the target is reached
    while (left_encoder_data.pulse_count < target_pulses || right_encoder_data.pulse_count < target_pulses)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Stop motors after reaching the target distance
    stop_motors();
}

void adjust_motor_speeds_with_pid()
{
    // Calculate an average target to keep both motors aligned
    int average_pulse_count = (left_encoder_data.pulse_count + right_encoder_data.pulse_count) / 2;
    pid_motor_1.setpoint = average_pulse_count;
    pid_motor_2.setpoint = average_pulse_count;

    // Compute adjustments based on the current pulse count vs. the target
    int left_duty_adjustment = pid_update(&pid_motor_1, left_encoder_data.pulse_count);
    int right_duty_adjustment = pid_update(&pid_motor_2, right_encoder_data.pulse_count);

    // Apply adjustments to maintain straight movement
    // set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), NORMAL_DUTY_CYCLE + left_duty_adjustment);
    //  set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), 3500);
    //  set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), 4000);

    // Apply adjustments to maintain straight movement with a 250 bias on motor 2
    motor1_forward(NORMAL_DUTY_CYCLE + left_duty_adjustment);
    motor2_forward(NORMAL_DUTY_CYCLE + right_duty_adjustment); // Adding bias here
}

void vTaskEncoder(__unused void *pvParameters)
{

    encoder_kalman_init(&left_encoder_data.kalman_state, 0.1, 0.1, 1.0, 0.0);
    encoder_kalman_init(&right_encoder_data.kalman_state, 0.1, 0.1, 1.0, 0.0);
    while (1)
    {
        read_encoder_data(LEFT_ENCODER_PIN, &left_encoder_data);
        read_encoder_data(RIGHT_ENCODER_PIN, &right_encoder_data);
    }
}

void vTaskMotor(__unused void *pvParameters)
{
    int state = 0;
    while (1)
    {

        if (PLS_STOP == 1)
        {
            stop_motors();
            // turn right
            rotate_90_degrees_right();

            // code to move forward 90 cm
            move_forward_distance(MOVE_DISTANCE_CM);
        }
        else
        {
            if (state == 0)
            {
                strong_start(GO_FORWARD); // Initiate burst start
                state = 1;                // Mark as started to avoid multiple bursts
            }
            else
            {
                adjust_motor_speeds_with_pid(); // Continue with PID adjustments
            }
        }
    }
}

void vTaskUltrasonic(__unused void *pvParameters)
{
    kalman_state.q = 0.1;
    kalman_state.r = 0.1;
    kalman_state.p = 1.0;
    kalman_state.x = 0.0;

    while (1)
    {
        set_trigger_pin(1);
        vTaskDelay(pdMS_TO_TICKS(10));
        set_trigger_pin(0);
        uint64_t pulse_length = get_pulse_length();
        double distance = get_cm(&kalman_state, pulse_length);
        if (distance < STOPPING_DISTANCE)
        {
            PLS_STOP = 1;
        }
        else
        {
            PLS_STOP = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vLaunch()
{
    TaskHandle_t motorTask, encoderTask, distanceTask;
    xTaskCreate(vTaskEncoder, "EncoderTask", 2048, NULL, tskIDLE_PRIORITY + 1UL,
                &encoderTask);
    xTaskCreate(vTaskMotor, "MotorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &motorTask);
    xTaskCreate(vTaskUltrasonic, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, &distanceTask);
    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();
    init_encoder();
    init_motors();
    init_ultrasonic();

    // In car.c
    pid_init(&pid_motor_1, 0.8, 0.0, 0.1, 0.0, -MAX_DUTY_CYCLE, MAX_DUTY_CYCLE); // Left motor PID
    pid_init(&pid_motor_2, 0.8, 0.0, 0.1, 0.0, -MAX_DUTY_CYCLE, MAX_DUTY_CYCLE); // Right motor PID

    sleep_ms(1000);
    vLaunch();
    while (1)
    {
        // Should never reach here
    }
    return 0;
}
