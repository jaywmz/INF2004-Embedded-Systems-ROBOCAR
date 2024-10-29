#include "motor.h"
#include <stdio.h>

// Variables to control motor speed
static int duty_cycle_motor1 =
    6250; // Start with a 50% duty cycle for motor 1 (range: 0-12500)
static int duty_cycle_motor2 =
    6250; // Start with a 50% duty cycle for motor 2 (range: 0-12500)

// Function to set up PWM for speed control on a given motor
static void setup_pwm(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_clkdiv(slice_num, 100); // Set clock divider for 1.25 MHz PWM clock
    pwm_set_wrap(slice_num, 12500); // Set wrap value for a 100 Hz PWM signal
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0); // Start with 0 duty cycle
    pwm_set_enabled(slice_num, true);
}

// Function to start a motor at the specified duty cycle
static void start_motor(uint slice_num, int duty_cycle) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

static void motor1_forward(int duty_cycle) {
    gpio_put(MOTOR1_IN1_PIN, 1);
    gpio_put(MOTOR1_IN2_PIN, 0);
    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    start_motor(slice_num_motor1, duty_cycle);
}

static void motor1_backward(int duty_cycle) {
    gpio_put(MOTOR1_IN1_PIN, 0);
    gpio_put(MOTOR1_IN2_PIN, 1);
    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    start_motor(slice_num_motor1, duty_cycle);
}

static void motor2_forward(int duty_cycle) {
    gpio_put(MOTOR2_IN1_PIN, 1);
    gpio_put(MOTOR2_IN2_PIN, 0);
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    start_motor(slice_num_motor2, duty_cycle);
}

static void motor2_backward(int duty_cycle) {
    gpio_put(MOTOR2_IN1_PIN, 0);
    gpio_put(MOTOR2_IN2_PIN, 1);
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    start_motor(slice_num_motor2, duty_cycle);
}

// Function to set up motor direction control pins
static void setup_motor_pins(uint in1_pin, uint in2_pin) {
    gpio_init(in1_pin);
    gpio_set_dir(in1_pin, GPIO_OUT);
    gpio_init(in2_pin);
    gpio_set_dir(in2_pin, GPIO_OUT);
}

void move_forward() {
    motor1_forward(duty_cycle_motor1);
    motor2_forward(duty_cycle_motor2);
}

void move_backward() {
    motor1_backward(duty_cycle_motor1);
    motor2_backward(duty_cycle_motor2);
}

void turn_left() {
    motor1_forward(9000);
    motor2_backward(9000);
}

void turn_right() {
    motor1_backward(9000);
    motor2_forward(9000);
}

// Function to stop a motor
void stop_motors() {
    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A,
                       0); // Set duty cycle to 0 to stop
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    pwm_set_chan_level(slice_num_motor2, PWM_CHAN_A,
                       0); // Set duty cycle to 0 to stop
}

// Function to update motor speed
void set_motor_speed(uint slice_num, int duty_cycle) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

void init_motors() {
    // Set up motor control pins and PWM for both motors
    setup_motor_pins(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    setup_pwm(MOTOR1_PWM_PIN);
    setup_motor_pins(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    setup_pwm(MOTOR2_PWM_PIN);

    printf("Motors are ready for control.\n");
}