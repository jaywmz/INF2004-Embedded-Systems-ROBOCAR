#include "FreeRTOS.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>

// Define pins for Motor 1
#define MOTOR1_PWM_PIN 2
#define MOTOR1_IN1_PIN 0
#define MOTOR1_IN2_PIN 1

// Define pins for Motor 2
#define MOTOR2_PWM_PIN 6
#define MOTOR2_IN1_PIN 4
#define MOTOR2_IN2_PIN 5

// Variables for motor speeds
int duty_cycle_motor1 = 6250;
int duty_cycle_motor2 = 6250;

// Define a queue to receive commands from user input
QueueHandle_t commandQueue;

// Setup PWM for speed control on a given motor
void setup_pwm(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_clkdiv(slice_num, 100);
    pwm_set_wrap(slice_num, 12500);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
    pwm_set_enabled(slice_num, true);
}

// Setup motor direction control pins
void setup_motor_pins(uint in1_pin, uint in2_pin) {
    gpio_init(in1_pin);
    gpio_set_dir(in1_pin, GPIO_OUT);
    gpio_init(in2_pin);
    gpio_set_dir(in2_pin, GPIO_OUT);
}

// Function to rotate motor in a specific direction
void rotate_motor(const char *direction, uint in1_pin, uint in2_pin) {
    if (strcmp(direction, "forward") == 0) {
        gpio_put(in1_pin, 1);
        gpio_put(in2_pin, 0);
    } else if (strcmp(direction, "backward") == 0) {
        gpio_put(in1_pin, 0);
        gpio_put(in2_pin, 1);
    }
}

// Start or stop a motor at a given duty cycle
void start_motor(uint slice_num, int duty_cycle) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

void stop_motor(uint slice_num) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
}

// Task to handle motor 1 commands
void motor1_task(void *params) {
    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);

    while (true) {
        char command;
        if (xQueueReceive(commandQueue, &command, portMAX_DELAY) == pdPASS) {
            if (command == 'f') {
                rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
                start_motor(slice_num_motor1, duty_cycle_motor1);
                printf("Motor 1 rotating forward.\n");
            } else if (command == 'b') {
                rotate_motor("backward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
                start_motor(slice_num_motor1, duty_cycle_motor1);
                printf("Motor 1 rotating backward.\n");
            } else if (command == '+') {
                duty_cycle_motor1 = (duty_cycle_motor1 < 12500)
                                        ? duty_cycle_motor1 + 1250
                                        : 12500;
                pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A,
                                   duty_cycle_motor1);
                printf("Increased Motor 1 speed. Duty cycle: %d\n",
                       duty_cycle_motor1);
            } else if (command == '-') {
                duty_cycle_motor1 =
                    (duty_cycle_motor1 > 0) ? duty_cycle_motor1 - 1250 : 0;
                pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A,
                                   duty_cycle_motor1);
                printf("Decreased Motor 1 speed. Duty cycle: %d\n",
                       duty_cycle_motor1);
            } else if (command == 's') {
                stop_motor(slice_num_motor1);
                printf("Motor 1 stopped.\n");
            }
        }
    }
}

// Task to handle motor 2 commands
void motor2_task(void *params) {
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);

    while (true) {
        char command;
        if (xQueueReceive(commandQueue, &command, portMAX_DELAY) == pdPASS) {
            if (command == 'F') {
                rotate_motor("forward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
                start_motor(slice_num_motor2, duty_cycle_motor2);
                printf("Motor 2 rotating forward.\n");
            } else if (command == 'B') {
                rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
                start_motor(slice_num_motor2, duty_cycle_motor2);
                printf("Motor 2 rotating backward.\n");
            } else if (command == '>') {
                duty_cycle_motor2 = (duty_cycle_motor2 < 12500)
                                        ? duty_cycle_motor2 + 1250
                                        : 12500;
                pwm_set_chan_level(slice_num_motor2, PWM_CHAN_A,
                                   duty_cycle_motor2);
                printf("Increased Motor 2 speed. Duty cycle: %d\n",
                       duty_cycle_motor2);
            } else if (command == '<') {
                duty_cycle_motor2 =
                    (duty_cycle_motor2 > 0) ? duty_cycle_motor2 - 1250 : 0;
                pwm_set_chan_level(slice_num_motor2, PWM_CHAN_A,
                                   duty_cycle_motor2);
                printf("Decreased Motor 2 speed. Duty cycle: %d\n",
                       duty_cycle_motor2);
            } else if (command == 'S') {
                stop_motor(slice_num_motor2);
                printf("Motor 2 stopped.\n");
            }
        }
    }
}

// Task to read input from serial and send it to the queue
void input_task(void *params) {
    while (true) {
        char input = getchar_timeout_us(500000); // 500ms timeout
        if (input != PICO_ERROR_TIMEOUT) {
            xQueueSend(commandQueue, &input, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Main setup function
void setup() {
    stdio_init_all();
    sleep_ms(1000);

    // Setup motor control pins and PWM for both motors
    setup_motor_pins(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    setup_pwm(MOTOR1_PWM_PIN);
    setup_motor_pins(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    setup_pwm(MOTOR2_PWM_PIN);

    // Create a command queue
    commandQueue = xQueueCreate(10, sizeof(char));

    // Create tasks for each motor and input handling
    xTaskCreate(motor1_task, "Motor 1 Task", 256, NULL, 1, NULL);
    xTaskCreate(motor2_task, "Motor 2 Task", 256, NULL, 1, NULL);
    xTaskCreate(input_task, "Input Task", 256, NULL, 1, NULL);

    printf("Motors are ready for control.\n");

    // Start the scheduler
    vTaskStartScheduler();
}

// Main function
int main() {
    setup();
    while (true) {
        // The main loop will not be reached; FreeRTOS tasks will run instead
    }
}
