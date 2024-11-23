#include "motor.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Variables to control motor speed
// static int duty_cycle_motor1 = 6250; // Start with a 50% duty cycle for motor 1 (range: 0-12500)
// static int duty_cycle_motor2 = 6250; // Start with a 50% duty cycle for motor 2 (range: 0-12500)

// Function to set up PWM for speed control on a given motor
static void setup_pwm(uint gpio_pin)
{
    // Set up GPIO function to PWM
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    
    // Find PWM Slice connected to given GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);

    // Calculate the PWM frequency and set the PWM wrap value
    float clock_freq = 125000000.0f;  // Default clock frequency of the Pico in Hz, 125MHz
    float freq = 1000.0f;    // Desired frequency
    float divider = clock_freq / (freq * 65536.0f);  // Compute clock divider
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, MAX_DUTY_CYCLE);
    // pwm_set_clkdiv(slice_num, 100.f);               // Set clock divider for 1.25 MHz PWM clock
    // pwm_set_wrap(slice_num, 12500);               // Set wrap value for a 100 Hz PWM signal

    // pwm_set_chan_level(slice_num, PWM_CHAN_A, 0); // Start with 0 duty cycle
    pwm_set_gpio_level(gpio_pin, (uint16_t)(0 * MAX_DUTY_CYCLE));
    pwm_set_enabled(slice_num, true);
}
// NOT USED directly in car.c - This function is used internally in `init_motors` to configure PWM for motor speed control.

// Function to start a motor at the specified duty cycle
static void start_motor(uint gpio_pin, float duty_cycle)
{
    // pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
    pwm_set_gpio_level(gpio_pin, (uint16_t)(duty_cycle * MAX_DUTY_CYCLE));
}
// NOT USED directly in car.c - This function is used internally in `strong_start` to start the motors with a specified duty cycle.

void motor1_forward(float duty_cycle)
{
    gpio_put(MOTOR1_IN1_PIN, 1);
    gpio_put(MOTOR1_IN2_PIN, 0);
    // uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    // start_motor(slice_num_motor1, duty_cycle);
    pwm_set_gpio_level(MOTOR1_PWM_PIN, (uint16_t)(duty_cycle * MAX_DUTY_CYCLE));
}
// USED in car.c - This function is called in `move_forward_distance` and `adjust_motor_speeds_with_pid` to set motor 1 to move forward.

void motor1_backward(float duty_cycle)
{
    gpio_put(MOTOR1_IN1_PIN, 0);
    gpio_put(MOTOR1_IN2_PIN, 1);
    // uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    // start_motor(MOTOR1_PWM_PIN, duty_cycle);
    pwm_set_gpio_level(MOTOR1_PWM_PIN, (uint16_t)(duty_cycle * MAX_DUTY_CYCLE));
}
// NOT USED in car.c - This function sets motor 1 to move backward but is not called directly in car.c.

void motor2_forward(float duty_cycle)
{
    gpio_put(MOTOR2_IN1_PIN, 1);
    gpio_put(MOTOR2_IN2_PIN, 0);
    // uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    // start_motor(MOTOR2_PWM_PIN, duty_cycle);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, (uint16_t)(duty_cycle * MAX_DUTY_CYCLE));
}
// USED in car.c - This function is called in `move_forward_distance` and `adjust_motor_speeds_with_pid` to set motor 2 to move forward.

void motor2_backward(float duty_cycle)
{
    gpio_put(MOTOR2_IN1_PIN, 0);
    gpio_put(MOTOR2_IN2_PIN, 1);
    // uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    // start_motor(MOTOR2_PWM_PIN, duty_cycle);
    pwm_set_gpio_level(MOTOR2_PWM_PIN, (uint16_t)(duty_cycle * MAX_DUTY_CYCLE));
}
// NOT USED in car.c - This function sets motor 2 to move backward but is not called directly in car.c.

// Function to set up motor direction control pins
static void setup_motor_pins(uint in1_pin, uint in2_pin)
{
    gpio_init(in1_pin);
    gpio_set_dir(in1_pin, GPIO_OUT);
    gpio_init(in2_pin);
    gpio_set_dir(in2_pin, GPIO_OUT);
}
// NOT USED directly in car.c - This function is called within `init_motors` to set up the direction control pins for each motor.

void move_forward(float duty_cycle_motor1, float duty_cycle_motor2)
{
    motor1_forward(duty_cycle_motor1);
    motor2_forward(duty_cycle_motor2);
}
// NOT USED in car.c - This function moves both motors forward, but car.c calls `motor1_forward` and `motor2_forward` directly instead.

void move_backward(float duty_cycle_motor1, float duty_cycle_motor2)
{
    motor1_backward(duty_cycle_motor1);
    motor2_backward(duty_cycle_motor2);
}
// NOT USED in car.c - This function moves both motors backward but is not called in car.c.

void turn_right(float duty_cycle_motor1, float duty_cycle_motor2)
{
    motor1_forward(duty_cycle_motor1);
    motor2_backward(duty_cycle_motor2);
}
// NOT USED in car.c - This function makes the car turn left by setting motor 1 to forward and motor 2 to backward.

void turn_left(float duty_cycle_motor1, float duty_cycle_motor2)
{
    motor1_backward(duty_cycle_motor1);
    motor2_forward(duty_cycle_motor2);
}
// NOT USED in car.c - This function makes the car turn right by setting motor 1 to backward and motor 2 to forward.

void stop_motors()
{
    // uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    // pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A, 0); // Set duty cycle to 0 to stop
    // uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    // pwm_set_chan_level(slice_num_motor2, PWM_CHAN_A, 0); // Set duty cycle to 0 to stop
    pwm_set_gpio_level(MOTOR1_PWM_PIN, (uint16_t)(0.0 * MAX_DUTY_CYCLE));
    pwm_set_gpio_level(MOTOR2_PWM_PIN, (uint16_t)(0.0 * MAX_DUTY_CYCLE));
}
// USED in car.c - This function is called in several places in car.c to stop both motors.

void set_motor_speed(uint slice_num, float duty_cycle)
{
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}
// NOT USED directly in car.c - This function is used within `strong_start` to set the motor speed with a specified duty cycle.

void set_direction(int direction, uint in1_pin, uint in2_pin)
{
    if (direction == GO_FORWARD)
    {
        gpio_put(in1_pin, 1);
        gpio_put(in2_pin, 0);
    }
    else if (direction == GO_BACKWARD)
    {
        gpio_put(in1_pin, 0);
        gpio_put(in2_pin, 1);
    }
}
// USED in car.c - This function is called in `rotate_90_degrees_right` and `strong_start` to set the motor direction for turning and movement.

float pid_compute(PIDController *pid, float measurement)
{
    float error = pid->setpoint - measurement;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}
// NOT USED in car.c - This function computes the PID output, but car.c uses `pid_update` instead.

void strong_start(int direction)
{
    // Set direction for both motors
    set_direction(direction, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    set_direction(direction, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);

    // Phase 1: High power burst
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), BURST_DUTY_CYCLE);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), BURST_DUTY_CYCLE);
    sleep_ms(BURST_DURATION_MS);

    // Phase 2: Medium power
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), HIGH_POWER_DUTY);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), HIGH_POWER_DUTY);
    sleep_ms(HIGH_POWER_DURATION_MS);

    // Phase 3: Normal speed
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), NORMAL_DUTY_CYCLE);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), NORMAL_DUTY_CYCLE);
}
// USED in car.c - This function is called in `move_forward_distance` and `adjust_motor_speeds_with_pid` to provide an initial high power start for the motors.

void pid_init(PIDController *pid, float kp, float ki, float kd,
              float setpoint, float output_min, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->output_min = output_min;
    pid->output_max = output_max;
    // pid->last_time = to_ms_since_boot(get_absolute_time());
}
// USED in car.c - This function initializes the PID controllers for both motors in `main`.

float pid_update(PIDController *pid, float measurement)
{
    // Calculate delta of time
    // uint32_t current_time = to_ms_since_boot(get_absolute_time());
    // double dt = (current_time - pid->last_time) / 1000.0;
    
    float error = pid->setpoint - measurement;
    float p_term = pid->kp * error;

    // pid->integral += error * dt;
    pid->integral += error;
    float i_term = pid->ki * pid->integral;

    // double derivative = (error - pid->prev_error) / dt;
    float derivative = (error - pid->prev_error);
    float d_term = pid->kd * derivative;

    // Clamp output to within limits
    float output = p_term + i_term + d_term;
    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    pid->prev_error = error;
    // pid->last_time = current_time;

    return output;
}
// USED in car.c - This function is called in `adjust_motor_speeds_with_pid` to compute PID control adjustments for the motors.

void pid_reset(PIDController *pid)
{
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    // pid->last_time = to_ms_since_boot(get_absolute_time());
}
// NOT USED in car.c - This function resets the PID controller's integral and error terms but is not called in car.c.

void init_motors()
{
    // Set up motor control pins and PWM for both motors
    setup_motor_pins(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    setup_pwm(MOTOR1_PWM_PIN);
    setup_motor_pins(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    setup_pwm(MOTOR2_PWM_PIN);
}
// USED in car.c - This function is called in `main` to initialize the motors and set up PWM.


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