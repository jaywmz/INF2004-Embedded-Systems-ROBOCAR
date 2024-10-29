#include "FreeRTOS.h"
#include "motor.h"
#include "mqtt.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

Compass compass;

void vTaskMotor(__unused void *pvParameters) {
    while (1) {
        if (compass.p > 20) {
            // printf("Moving forward\n");
            move_forward();
        } else if (compass.p < -20) {
            // printf("Moving backward\n");
            move_backward();
        } else if (compass.r > 20) {
            // printf("Turning right\n");
            turn_right();
        } else if (compass.r < -20) {
            // printf("Turning left\n");
            turn_left();
        } else {
            // printf("Stopping\n");
            stop_motors();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vTaskCompass(__unused void *pvParameters) {
    while (1) {
        get_compass_data(&compass);
        // printf("p=%d, r=%d, y=%d\n", compass.p, compass.r, compass.y);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vLaunch() {
    TaskHandle_t motorTask, compassTask;
    xTaskCreate(vTaskCompass, "CompassTask", 2048, NULL, tskIDLE_PRIORITY + 1UL,
                &compassTask);
    xTaskCreate(vTaskMotor, "MotorTask", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 2UL, &motorTask);
    vTaskStartScheduler();
}

int main(void) {
    stdio_init_all();
    init_mqtt();
    init_motors();
    sleep_ms(2000);
    vLaunch();
    while (1) {
        // Should never reach here
    }
    return 0;
}
