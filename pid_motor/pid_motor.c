#include "pid_motor.h"

// Initialize duty cycle variables
int duty_cycle_motor1 = 11250;  // Start with a 50% duty cycle for motor 1
int duty_cycle_motor2 = 6250;  // Start with a 50% duty cycle for motor 2

// Function to set up PWM for speed control on a given motor
void setup_pwm(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_clkdiv(slice_num, 100); // Set clock divider for 1.25 MHz PWM clock
    pwm_set_wrap(slice_num, 12500); // Set wrap value for a 100 Hz PWM signal
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0); // Start with 0 duty cycle
    pwm_set_enabled(slice_num, true);
}

// Function to set up motor direction control pins
void setup_motor_pins(uint in1_pin, uint in2_pin) {
    gpio_init(in1_pin);
    gpio_set_dir(in1_pin, GPIO_OUT);
    gpio_init(in2_pin);
    gpio_set_dir(in2_pin, GPIO_OUT);
}

// Function to rotate motor in a specific direction
void rotate_motor(char *direction, uint in1_pin, uint in2_pin) {
    if (direction == "forward") {
        gpio_put(in1_pin, 1);
        gpio_put(in2_pin, 0);
    } else if (direction == "backward") {
        gpio_put(in1_pin, 0);
        gpio_put(in2_pin, 1);
    }
}

// Function to start a motor at the specified duty cycle
void start_motor(uint slice_num, int duty_cycle) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

// Function to stop a motor
void stop_motor(uint slice_num) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0); // Set duty cycle to 0 to stop
}

// Function to update motor speed
void set_motor_speed(uint slice_num, int duty_cycle) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

// Function to control both motors in the same direction and speed
void rotate_both_motors(char *direction, int duty_cycle_motor1, int duty_cycle_motor2) {
    rotate_motor(direction, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    rotate_motor(direction, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), duty_cycle_motor1);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), duty_cycle_motor2);
}
