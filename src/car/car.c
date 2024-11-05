#include "FreeRTOS.h"
#include "encoder.h"
#include "motor.h"
#include "ultrasonic.h"
#include "mqtt.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>
//#include "barcode.h"
//#include "linefollow.h"

#include "hardware/pwm.h"

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

Compass compass;
EncoderData left_encoder_data, right_encoder_data;
KalmanState kalman_state;

#define STOPPING_DISTANCE 10.0 // Distance in cm to stop when obstacle detected
#define MOVE_DISTANCE_CM 90.0  // Distance to move forward after turning
#define TARGET_PULSES 8        // Adjust based on calibration for a 90-degree turn

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
        if (compass.direction == 'f')
        {
            printf("Moving forward\n");
            move_forward();
        }
        else if (compass.direction == 'b')
        {
            printf("Moving backward\n");
            move_backward();
        }
        else if (compass.direction == 'l')
        {
            printf("Turning left\n");
            motor1_backward(compass.left_duty);
            motor2_forward(compass.right_duty);
        }
        else if (compass.direction == 'r')
        {
            printf("Turning right\n");
            motor1_forward(compass.left_duty);
            motor2_backward(compass.right_duty);
        }
        else if (compass.direction == 's')
        {
            printf("Stopping\n");
            stop_motors();
        }
        else
        {
            printf("Invalid direction\n");
            stop_motors();
        }
    }
}

void vTaskCompass(__unused void *pvParameters)
{
    while (1)
    {
        get_compass_data(&compass);
        vTaskDelay(pdMS_TO_TICKS(1000));
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
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vLaunch()
{
    TaskHandle_t motorTask, compassTask, encoderTask, distanceTask;
    xTaskCreate(vTaskCompass, "CompassTask", 2048, NULL, tskIDLE_PRIORITY + 1UL, &compassTask);
    // xTaskCreate(vTaskEncoder, "EncoderTask", 2048, NULL, tskIDLE_PRIORITY + 1UL, &encoderTask);
    // xTaskCreate(vTaskMotor, "MotorTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, &motorTask);
    // xTaskCreate(vTaskUltrasonic, "UltrasonicTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, &distanceTask);
    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();
    init_mqtt();
    init_motors();
    // init_encoder();
    // init_ultrasonic();
    sleep_ms(1000);
    vLaunch();

    while (1)
    {
        tight_loop_contents();
    }

    return 0;
}
