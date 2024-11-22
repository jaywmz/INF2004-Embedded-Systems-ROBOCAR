#include "FreeRTOS.h"
#include "encoder.h"
#include "motor.h"
#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>
#include <math.h>
#include "udp.h"
#include "hardware/pwm.h"
// #include "barcode.h"
// #include "linefollow.h"

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

// Global encoder data structures for left and right wheels
EncoderData motor1_encoder_data, motor2_encoder_data;
// Kalman filter state for ultrasonic distance measurements
KalmanState kalman_state;
// Global compass struct for storing remote controls
Compass compass;
// PID controllers for left and right motors
PIDController pid_motor_1, pid_motor_2;
// Global flag to indicate when the car should stop based on ultrasonic readings
int PLS_STOP = 0;
// Constants for movement and stopping distances
#define STOPPING_DISTANCE 20.0 // Distance in cm to stop when obstacle detected
#define MOVE_DISTANCE_CM 82.0  // Distance to move forward after turning

// Helpfer function to map speed percentage from remote controller to speed in pulses per second for PID controller
// float percent_to_pulsesPerSec(uint16_t speedPercentage) {
//     // 25 pulses per sec is roughly the maximum speed of car at 100% duty cycle on the ground
//     float speed = ((float)speedPercentage / 100.0f);
//     float pulsesPerSec;
//     if (speed >= 0.6 && speed < 0.7) {
//         pulsesPerSec = 22.0f;
//     }
//     else if (speed >= 0.7 && speed < 0.8) {
//         pulsesPerSec = 23.0f;
//     }
//     else if (speed >= 0.8 && speed < 0.9) {
//         pulsesPerSec = 24.0f;
//     }
//     else if (speed >= 0.9 && speed < 1.0) {
//         pulsesPerSec = 25.0f;
//     }
//     return pulsesPerSec;
// }

// Function to adjust motor speeds dynamically to maintain straight movement using PID
// void adjust_motor_speeds_with_pid()
// {
//     // Get desired speed from remote control message
//     float speed = percent_to_pulsesPerSec(compass.speed);
//     pid_motor_1.setpoint = speed;
//     pid_motor_2.setpoint = speed;

//     // Compute new duty cycle based on the current pulse count vs. the target
//     float motor1_dutyCycle = pid_update(&pid_motor_1, motor1_encoder_data.speed);
//     float motor2_dutyCycle = pid_update(&pid_motor_2, motor2_encoder_data.speed);

//     // Store calculated duty cycle
//     pid_motor_1.duty_cycle = motor1_dutyCycle;
//     pid_motor_2.duty_cycle = motor2_dutyCycle;
// }

// Encoder task for reading encoder data and updating encoder structures
// void vTaskEncoder(__unused void *pvParameters)
// {
//     encoder_kalman_init(&motor2_encoder_data.kalman_state, 0.1, 0.1, 1.0, 0.0);
//     encoder_kalman_init(&motor1_encoder_data.kalman_state, 0.1, 0.1, 1.0, 0.0);

//     while (1)
//     {
//         read_encoder_data(MOTOR2_ENCODER_PIN, &motor2_encoder_data);
//         read_encoder_data(MOTOR1_ENCODER_PIN, &motor1_encoder_data);
//     }
// }

// Moving average function to get average speed
float movingAvg(bool motor1, float newSpeed)
{
    const int len = 5;
    static float speedsMotor1[10] = {0.0};
    static float speedsMotor2[10] = {0.0};
    static int index1 = 0;
    static int index2 = 0;
    static float sum1 = 0;
    static float sum2 = 0;

    if (motor1)
    {
        sum1 = sum1 - speedsMotor1[index1] + newSpeed;

        speedsMotor1[index1] = newSpeed;

        index1++;
        if (index1 >= len - 1)
        {
            index1 = 0;
        }

        return sum1 / len;
    }
    else
    {
        sum2 = sum2 - speedsMotor2[index2] + newSpeed;

        speedsMotor2[index2] = newSpeed;

        index2++;
        if (index2 >= len - 1)
        {
            index2 = 0;
        }

        return sum2 / len;
    }
}

// Task to measure speed (pulses per second)
void encoder_task(void *pvParameters)
{
    int prev_pulses_motor1 = 0, prev_pulses_motor2 = 0;

    while (1)
    {
        // Find number of pulses since last interval
        int pulses_motor1 = motor1_encoder_data.pulse_count - prev_pulses_motor1;
        int pulses_motor2 = motor2_encoder_data.pulse_count - prev_pulses_motor2;
        prev_pulses_motor1 = motor1_encoder_data.pulse_count;
        prev_pulses_motor2 = motor2_encoder_data.pulse_count;

        // Calculate speed, since interval is 100ms, multiply 10 to get pulses per 1000ms/1s
        float speed_motor1 = (float)pulses_motor1 * 10;
        float speed_motor2 = (float)pulses_motor2 * 10;

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
        float speed = ((float)compass.speed / 100.0f) * 2;
        if (speed > 1.0) 
        {
            speed = 1.0;
        }
        if (compass.direction == 1)
        {
            if (PLS_STOP == 1)
            {
                stop_motors();
            } 
            else 
            {
                printf("Moving forward\n");
                // adjust_motor_speeds_with_pid();
                move_forward(speed - 0.02, speed);
            }

        }
        else if (compass.direction == 2)
        {
            printf("Moving backward\n");
            // adjust_motor_speeds_with_pid();
            move_backward(speed - 0.02, speed);
        }
        else if (compass.direction == 3)
        {
            printf("Turning left\n");
            // adjust_motor_speeds_with_pid();
            turn_left(speed - 0.02, speed);
        }
        else if (compass.direction == 4)
        {
            printf("Turning right\n");
            // adjust_motor_speeds_with_pid();
            turn_right(speed - 0.02, speed);
        }
        else if (compass.direction == 0)
        {
            printf("Stopping\n");
            stop_motors();
        }
        else
        {
            printf("Invalid direction\n");
            stop_motors();
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS)); // Wait for next sample
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

void vTaskDashboard() {
    
}

// Function to launch all tasks and start the FreeRTOS scheduler
void vLaunch()
{
    TaskHandle_t motorTask, compassTask, encoderTask, distanceTask, dashboardTask;
    xTaskCreate(vTaskCompass, "CompassTask", 2048, NULL, tskIDLE_PRIORITY + 1UL, &compassTask);
    xTaskCreate(encoder_task, "EncoderTask", 2048, NULL, tskIDLE_PRIORITY + 1UL, &encoderTask);
    xTaskCreate(vTaskMotor, "MotorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &motorTask);
    xTaskCreate(vTaskUltrasonic, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, &distanceTask);
    xTaskCreate(vTaskDashboard, "DashboardTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &dashboardTask);
    vTaskStartScheduler();
}

// Main function for system initialization and launching tasks
int main(void)
{
    stdio_init_all(); // Initialize standard I/O for debugging
    init_udp();
    init_encoder();    // Initialize encoders
    init_motors();     // Initialize motors
    init_ultrasonic(); // Initialize ultrasonic sensor

    // Initialize PID controllers for both motors
    float kp1 = 0.004; float kp2 = 0.01;
    float ki1 = 0.01; float ki2 = 0.01;
    float kd1 = 0.004; float kd2 = 0.004;
    float setpoint = 0.0;
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
