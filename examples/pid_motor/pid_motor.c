#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Define pins for Motor 1 (same as the current setup)
#define MOTOR1_PWM_PIN 2 // PWM-capable GPIO pin for motor 1 speed control (ENA)
#define MOTOR1_IN1_PIN 0 // GPIO pin for motor 1 direction control (IN1)
#define MOTOR1_IN2_PIN 1 // GPIO pin for motor 1 direction control (IN2)

// Define pins for Motor 2 (new setup)
#define MOTOR2_PWM_PIN 6 // PWM-capable GPIO pin for motor 2 speed control (ENA)
#define MOTOR2_IN1_PIN 4 // GPIO pin for motor 2 direction control (IN1)
#define MOTOR2_IN2_PIN 5 // GPIO pin for motor 2 direction control (IN2)

// Variables to control motor speed
int duty_cycle_motor1 = 6250; // Start with a 50% duty cycle for motor 1 (range: 0-12500)
int duty_cycle_motor2 = 6250; // Start with a 50% duty cycle for motor 2 (range: 0-12500)

// Function to set up PWM for speed control on a given motor
void setup_pwm(uint gpio_pin)
{
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_clkdiv(slice_num, 100);               // Set clock divider for 1.25 MHz PWM clock
    pwm_set_wrap(slice_num, 12500);               // Set wrap value for a 100 Hz PWM signal
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0); // Start with 0 duty cycle
    pwm_set_enabled(slice_num, true);
}

// Function to set up motor direction control pins
void setup_motor_pins(uint in1_pin, uint in2_pin)
{
    gpio_init(in1_pin);
    gpio_set_dir(in1_pin, GPIO_OUT);
    gpio_init(in2_pin);
    gpio_set_dir(in2_pin, GPIO_OUT);
}

// Function to rotate motor in a specific direction
void rotate_motor(char *direction, uint in1_pin, uint in2_pin)
{
    if (direction == "forward")
    {
        gpio_put(in1_pin, 1);
        gpio_put(in2_pin, 0);
    }
    else if (direction == "backward")
    {
        gpio_put(in1_pin, 0);
        gpio_put(in2_pin, 1);
    }
}

// Function to start a motor at the specified duty cycle
void start_motor(uint slice_num, int duty_cycle)
{
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

// Function to stop a motor
void stop_motor(uint slice_num)
{
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0); // Set duty cycle to 0 to stop
}

// Function to update motor speed
void set_motor_speed(uint slice_num, int duty_cycle)
{
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

// Function to control both motors in the same direction and speed
void rotate_both_motors(char *direction, int duty_cycle_motor1, int duty_cycle_motor2)
{
    rotate_motor(direction, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    rotate_motor(direction, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), duty_cycle_motor1);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), duty_cycle_motor2);
}

int main()
{
    // Initialize serial communication
    stdio_init_all();
    sleep_ms(1000); // Allow time for USB to initialize

    // Set up motor control pins and PWM for both motors
    setup_motor_pins(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    setup_pwm(MOTOR1_PWM_PIN);
    setup_motor_pins(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    setup_pwm(MOTOR2_PWM_PIN);

    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);

    printf("Motors are ready for control.\n");

    while (true)
    {
        // Read user input from the serial monitor
        char input = getchar_timeout_us(500000); // Wait for input for 500ms

        // Control for Motor 1
        if (input == 'f')
        {
            rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(slice_num_motor1, duty_cycle_motor1);
            printf("Motor 1 rotating forward.\n");
        }
        else if (input == 'b')
        {
            rotate_motor("backward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(slice_num_motor1, duty_cycle_motor1);
            printf("Motor 1 rotating backward.\n");
        }
        else if (input == '+')
        {
            if (duty_cycle_motor1 < 12500)
            {
                duty_cycle_motor1 += 1250; // Increase speed by 10% for Motor 1
                set_motor_speed(slice_num_motor1, duty_cycle_motor1);
                printf("Increased speed for Motor 1. Current duty cycle: %d\n", duty_cycle_motor1);
            }
        }
        else if (input == '-')
        {
            if (duty_cycle_motor1 > 0)
            {
                duty_cycle_motor1 -= 1250; // Decrease speed by 10% for Motor 1
                set_motor_speed(slice_num_motor1, duty_cycle_motor1);
                printf("Decreased speed for Motor 1. Current duty cycle: %d\n", duty_cycle_motor1);
            }
        }
        else if (input == 's')
        {
            stop_motor(slice_num_motor1);
            printf("Motor 1 stopped.\n");
        }

        // Control for Motor 2
        if (input == 'F')
        {
            rotate_motor("forward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(slice_num_motor2, duty_cycle_motor2);
            printf("Motor 2 rotating forward.\n");
        }
        else if (input == 'B')
        {
            rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(slice_num_motor2, duty_cycle_motor2);
            printf("Motor 2 rotating backward.\n");
        }
        else if (input == '>')
        {
            if (duty_cycle_motor2 < 12500)
            {
                duty_cycle_motor2 += 1250; // Increase speed by 10% for Motor 2
                set_motor_speed(slice_num_motor2, duty_cycle_motor2);
                printf("Increased speed for Motor 2. Current duty cycle: %d\n", duty_cycle_motor2);
            }
        }
        else if (input == '<')
        {
            if (duty_cycle_motor2 > 0)
            {
                duty_cycle_motor2 -= 1250; // Decrease speed by 10% for Motor 2
                set_motor_speed(slice_num_motor2, duty_cycle_motor2);
                printf("Decreased speed for Motor 2. Current duty cycle: %d\n", duty_cycle_motor2);
            }
        }
        else if (input == 'S')
        {
            stop_motor(slice_num_motor2);
            printf("Motor 2 stopped.\n");
        }

        // Control for Both Motors Together
        if (input == 'w')
        {
            rotate_both_motors("forward", duty_cycle_motor1, duty_cycle_motor2);
            printf("Both motors rotating forward.\n");
        }
        else if (input == 'x')
        {
            rotate_both_motors("backward", duty_cycle_motor1, duty_cycle_motor2);
            printf("Both motors rotating backward.\n");
        }
        else if (input == 'p')
        {
            stop_motor(slice_num_motor1);
            stop_motor(slice_num_motor2);
            printf("Both motors stopped.\n");
        }

        sleep_ms(100); // Small delay to ensure smooth control
    }
}
