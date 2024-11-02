#include "motor.h"
#include "hardware/pwm.h"
#include <stdio.h>

// Variables to control motor speed
static int duty_cycle_motor1 =
    6250; // Start with a 50% duty cycle for motor 1 (range: 0-12500)
static int duty_cycle_motor2 =
    6250; // Start with a 50% duty cycle for motor 2 (range: 0-12500)

// Function to set up PWM for speed control on a given motor
static void setup_pwm(uint gpio_pin)
{
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_clkdiv(slice_num, 100);               // Set clock divider for 1.25 MHz PWM clock
    pwm_set_wrap(slice_num, 12500);               // Set wrap value for a 100 Hz PWM signal
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0); // Start with 0 duty cycle
    pwm_set_enabled(slice_num, true);
}

// Function to start a motor at the specified duty cycle
static void start_motor(uint slice_num, int duty_cycle)
{
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

static void motor1_forward(int duty_cycle)
{
    gpio_put(MOTOR1_IN1_PIN, 1);
    gpio_put(MOTOR1_IN2_PIN, 0);
    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    start_motor(slice_num_motor1, duty_cycle);
}

static void motor1_backward(int duty_cycle)
{
    gpio_put(MOTOR1_IN1_PIN, 0);
    gpio_put(MOTOR1_IN2_PIN, 1);
    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    start_motor(slice_num_motor1, duty_cycle);
}

static void motor2_forward(int duty_cycle)
{
    gpio_put(MOTOR2_IN1_PIN, 1);
    gpio_put(MOTOR2_IN2_PIN, 0);
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    start_motor(slice_num_motor2, duty_cycle);
}

static void motor2_backward(int duty_cycle)
{
    gpio_put(MOTOR2_IN1_PIN, 0);
    gpio_put(MOTOR2_IN2_PIN, 1);
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    start_motor(slice_num_motor2, duty_cycle);
}

// Function to set up motor direction control pins
static void setup_motor_pins(uint in1_pin, uint in2_pin)
{
    gpio_init(in1_pin);
    gpio_set_dir(in1_pin, GPIO_OUT);
    gpio_init(in2_pin);
    gpio_set_dir(in2_pin, GPIO_OUT);
}

void move_forward()
{
    motor1_forward(duty_cycle_motor1);
    motor2_forward(duty_cycle_motor2);
}

void move_backward()
{
    motor1_backward(duty_cycle_motor1);
    motor2_backward(duty_cycle_motor2);
}

void turn_left()
{
    motor1_forward(12000);
    motor2_backward(12000);
}

void turn_right()
{
    motor1_backward(12000);
    motor2_forward(12000);
}

// Function to stop a motor
void stop_motors()
{
    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A,
                       0); // Set duty cycle to 0 to stop
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    pwm_set_chan_level(slice_num_motor2, PWM_CHAN_A,
                       0); // Set duty cycle to 0 to stop
}

// Function to update motor speed
void set_motor_speed(uint slice_num, int duty_cycle)
{
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

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

void strong_start(int direction)
{
    // printf("Starting with high power burst\n");

    // Set direction
    set_direction(direction, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    set_direction(direction, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);

    // Initial high power burst
    // printf("Phase 1: High power burst at %d\n", BURST_DUTY_CYCLE);
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), BURST_DUTY_CYCLE);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), BURST_DUTY_CYCLE);

    sleep_ms(BURST_DURATION_MS);

    // Second phase - medium power
    // printf("Phase 2: Medium power at %d\n", HIGH_POWER_DUTY);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), HIGH_POWER_DUTY);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), HIGH_POWER_DUTY);

    sleep_ms(HIGH_POWER_DURATION_MS);

    // Final phase - normal operating speed
    // printf("Phase 3: Normal speed at %d\n", NORMAL_DUTY_CYCLE);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), NORMAL_DUTY_CYCLE);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), NORMAL_DUTY_CYCLE);
}

void pid_init(PIDController *pid, double kp, double ki, double kd,
              double setpoint, double output_min, double output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->last_time = to_ms_since_boot(get_absolute_time());
}

double pid_update(PIDController *pid, double measurement)
{
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    double dt = (current_time - pid->last_time) / 1000.0;

    double error = pid->setpoint - measurement;
    double p_term = pid->kp * error;

    pid->integral += error * dt;
    double i_term = pid->ki * pid->integral;

    double derivative = (error - pid->prev_error) / dt;
    double d_term = pid->kd * derivative;

    double output = p_term + i_term + d_term;

    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    pid->prev_error = error;
    pid->last_time = current_time;

    return output;
}

void pid_reset(PIDController *pid)
{
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->last_time = to_ms_since_boot(get_absolute_time());
}

void init_motors()
{
    // Set up motor control pins and PWM for both motors
    setup_motor_pins(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    setup_pwm(MOTOR1_PWM_PIN);
    setup_motor_pins(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    setup_pwm(MOTOR2_PWM_PIN);

    printf("Motors are ready for control.\n");
}