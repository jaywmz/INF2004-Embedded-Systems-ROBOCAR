#include "FreeRTOS.h"
#include "encoder.h"
#include "motor.h"
#include "ultrasonic.h"
#include "mqtt.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>

#include "hardware/pwm.h"

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

Compass compass;
EncoderData left_encoder_data, right_encoder_data;
KalmanState kalman_state;

PIDController pid_motor_1, pid_motor_2;

int PLS_STOP = 0;

#define STOPPING_DISTANCE 10.0 // Distance in cm to stop when obstacle detected
#define MOVE_DISTANCE_CM 90.0  // Distance to move forward after turning
// #define MOVE_TIME_MS 1000      // Time in ms to move forward after turn
#define TARGET_PULSES 8 // Adjust based on calibration for a 90-degree turn

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
    set_direction(GO_FORWARD, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    set_direction(GO_BACKWARD, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    rotate_90_degrees(8);
}

void rotate_90_degrees_left()
{
    // Set motor directions for turning left
    set_direction(GO_BACKWARD, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    set_direction(GO_FORWARD, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    rotate_90_degrees(7);
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
    pid_init(&pid_motor_1, 1.0, 0.1, 0.05, 0.7, 0, 12500);
    pid_init(&pid_motor_2, 1.0, 0.1, 0.05, 0.7, 0, 12500);

    int state = 0;
    while (1)
    {
        //     float left_control = pid_compute(&pid_motor_1, left_encoder_data.speed_m_per_s);
        //     float right_control = pid_compute(&pid_motor_2, right_encoder_data.speed_m_per_s);

        //     set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), left_control);
        //     set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), right_control);

        //     motor1_forward(left_control);
        //     motor2_forward(right_control);
        // printf("P: %d, R: %d\n", compass.p, compass.r);
        if (compass.p > 20)
        {
            move_forward();
            // if (PLS_STOP == 1)
            // {
            //     stop_motors();
            // }
            // else
            // {
            //     // printf("Moving forward\n");
            //     // move_forward();
            //     float left_control = pid_compute(&pid_motor_1, left_encoder_data.speed_m_per_s);
            //     float right_control = pid_compute(&pid_motor_2, right_encoder_data.speed_m_per_s);

            //     set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), left_control);
            //     set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), right_control);

            //     motor1_forward(left_control);
            //     motor2_forward(right_control);
            // }
            // strong_start(GO_FORWARD);
        }
        else if (compass.p < -20)
        {
            // printf("Moving backward\n");
            motor1_forward(11000);
            motor2_forward(11000);
        }
        else if (compass.r > 20)
        {
            // printf("Turning right\n");
            turn_right();
        }
        else if (compass.r < -20)
        {
            // printf("Turning left\n");
            turn_left();
        }
        else if (compass.y > 20)
        {
            if (state == 0)
            {
                rotate_90_degrees_right();
                state = 1;
            }
        }
        else if (compass.y < -20)
        {
            if (state == 1)
            {
                rotate_90_degrees_left();
                state = 0;
            }
        }
        else
        {
            // printf("Stopping\n");
            stop_motors();
        }
    }
}

void vTaskCompass(__unused void *pvParameters)
{
    while (1)
    {
        get_compass_data(&compass);
        // printf("p=%d, r=%d, y=%d\n", compass.p, compass.r, compass.y);
        vTaskDelay(pdMS_TO_TICKS(50));
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
        // double distance = getCm(&kalman_state);
        if (distance < STOPPING_DISTANCE)
        {
            PLS_STOP = 1;
            // stop_motors();
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
    TaskHandle_t motorTask, compassTask, encoderTask, distanceTask;
    xTaskCreate(vTaskCompass, "CompassTask", 2048, NULL, tskIDLE_PRIORITY + 1UL,
                &compassTask);
    xTaskCreate(vTaskEncoder, "EncoderTask", 2048, NULL, tskIDLE_PRIORITY + 1UL,
                &encoderTask);
    xTaskCreate(vTaskMotor, "MotorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &motorTask);
    // xTaskCreate(vTaskUltrasonic, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, &distanceTask);
    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();
    init_mqtt();
    init_encoder();
    init_motors();
    init_ultrasonic();

    sleep_ms(1000);
    vLaunch();
    while (1)
    {
        // Should never reach here
    }
    return 0;
}