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
#include "hardware/adc.h"
#include "line.h"
#include "pico/time.h"
#include <string.h>

// Constants for movement and stopping distances
#define STOPPING_DISTANCE 20.0 // Distance in cm to stop when obstacle detected
#define MOVE_DISTANCE_CM 82.0  // Distance to move forward after turning
#define LINE_SENSOR_PIN 27     // GPIO 27 connected to ADC input 1 (Line Following)
#define BARCODE_THRESHOLD 1800 // Threshold for barcode detection

// Global encoder data structures for left and right wheels
EncoderData motor1_encoder_data, motor2_encoder_data;
// Kalman filter state for ultrasonic distance measurements
KalmanState kalman_state;
// Global compass struct for storing remote controls
Compass compass;
// PID controllers for left and right motors
PIDController pid_motor_1, pid_motor_2;
// Task handles for motor and line tasks
TaskHandle_t motorTask, lineFollowTask, encoderTask;

// Global flag to indicate when the car should stop based on ultrasonic readings
int PLS_STOP = 0;

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
void vTaskEncoder(void *pvParameters)
{
    int prev_pulses_motor1 = 0, prev_pulses_motor2 = 0;

    while (1)
    {
        // Find number of pulses since last interval
        int pulses_motor1 = motor1_encoder_data.pulse_count - prev_pulses_motor1;
        int pulses_motor2 = motor2_encoder_data.pulse_count - prev_pulses_motor2;
        prev_pulses_motor1 = motor1_encoder_data.pulse_count;
        prev_pulses_motor2 = motor2_encoder_data.pulse_count;

        // Calculate speed, since interval is 100ms, multiply 10 to get pulses per second
        float speed_motor1 = (float)pulses_motor1 * 10;
        float speed_motor2 = (float)pulses_motor2 * 10;

        // Get moving average of speed and save to encoder structs
        motor1_encoder_data.speed = movingAvg(true, speed_motor1);
        motor2_encoder_data.speed = movingAvg(false, speed_motor2);

        // printf("motor1 speed: %f | motor2 speed: %f | ", motor1_encoder_data.speed, motor2_encoder_data.speed);

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
                // printf("Moving forward\n");
                // adjust_motor_speeds_with_pid();
                move_forward(speed - 0.02, speed);
            }
        }
        else if (compass.direction == 2)
        {
            // printf("Moving backward\n");
            // adjust_motor_speeds_with_pid();
            move_backward(speed - 0.02, speed);
        }
        else if (compass.direction == 3)
        {
            // printf("Turning left\n");
            // adjust_motor_speeds_with_pid();
            turn_left(speed - 0.02, speed);
        }
        else if (compass.direction == 4)
        {
            // printf("Turning right\n");
            // adjust_motor_speeds_with_pid();
            turn_right(speed - 0.02, speed);
        }
        else if (compass.direction == 0)
        {
            // printf("Stopping\n");
            stop_motors();
        }
        else
        {
            printf("Invalid direction\n");
            stop_motors();
        }
        // vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS)); // Wait for next sample
    }
}

void vTaskLineFollow(__unused void *pvParameters)
{
    // Start moving forward initially

    move_forward(0.8, 0.8);         // Set initial speed
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for motors to start

    uint64_t current_time = time_us_64();
    int should_go_left = 1;

    while (1)
    {
        if (read_line_sensor() == BLACK)
        {
            current_time = time_us_64();
            should_go_left = 1;
            move_forward(0.75, 0.7);
        }
        else
        {
            if (should_go_left)
            {
                turn_left(0.60, 0.70);
            }
            // turn_left(0.55, 0.65);
        }

        uint64_t new_time = time_us_64();
        if (new_time - current_time > 400000)
        {
            // move_forward(0, 0.6);
            should_go_left = 0;
            turn_right(0.8, 0.7);
        }
    }
}

void vTaskCompass(__unused void *pvParameters)
{
    uint16_t manual_mode_prev = 1; // True

    while (1)
    {
        get_compass_data(&compass);
        // printf("Direction: %d | Speed: %d | Manual Mode: %d\n", compass.direction, compass.speed, compass.manual_mode);
        if (compass.manual_mode != manual_mode_prev)
        {
            if (compass.manual_mode == 1)
            {
                if (lineFollowTask != NULL)
                {
                    vTaskDelete(lineFollowTask);
                }
                vTaskResume(motorTask);
                vTaskResume(encoderTask);
            }
            else
            {
                vTaskSuspend(motorTask);
                vTaskSuspend(encoderTask);
                xTaskCreate(vTaskLineFollow, "LineFollowTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &lineFollowTask);
            }
            manual_mode_prev = compass.manual_mode;
        }
        // send_telemetry(NULL);
        // vTaskDelay(pdMS_TO_TICKS(100));
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
    }
}

// Dashboard task to send car sensor data to serial monitor
void vTaskTelemetry(__unused void *pvParameters)
{
    while (1)
    {
        send_telemetry(NULL);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Function to launch all tasks and start the FreeRTOS scheduler
void vLaunch()
{
    xTaskCreate(vTaskMotor, "MotorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &motorTask);
    vTaskSuspend(motorTask);
    xTaskCreate(vTaskEncoder, "EncoderTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &encoderTask);
    vTaskSuspend(encoderTask);
    // xTaskCreate(vTaskLineFollow, "LineFollowTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &lineFollowTask);
    // vTaskSuspend(lineFollowTask);

    TaskHandle_t compassTask, ultrasonicTask;
    xTaskCreate(vTaskCompass, "CompassTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &compassTask);
    xTaskCreate(vTaskUltrasonic, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, &ultrasonicTask);
    // xTaskCreate(vTaskTelemetry, "DashboardTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3UL, &dashboardTask);
    vTaskStartScheduler();
}

// Main function for system initialization and launching tasks
int main(void)
{
    stdio_init_all();  // Initialize standard I/O for debugging
    init_udp();        // Initialize UDP communication
    init_encoder();    // Initialize encoders
    init_motors();     // Initialize motors
    init_ultrasonic(); // Initialize ultrasonic sensor
    init_ir_sensors(); // Initialize IR sensors

    // Initialize PID controllers for both motors
    float kp1 = 0.004;
    float kp2 = 0.01;
    float ki1 = 0.01;
    float ki2 = 0.01;
    float kd1 = 0.004;
    float kd2 = 0.004;
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
