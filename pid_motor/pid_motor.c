#include "pid_motor.h"

// Initialize variables
int duty_cycle_motor1 = 5000;
int duty_cycle_motor2 = 5000;
PIDController distance_pid;

void setup_pwm(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_clkdiv(slice_num, 100);
    pwm_set_wrap(slice_num, 12500);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
    pwm_set_enabled(slice_num, true);
    printf("PWM setup on pin %d\n", gpio_pin);
}

void setup_motor_pins(uint in1_pin, uint in2_pin) {
    gpio_init(in1_pin);
    gpio_set_dir(in1_pin, GPIO_OUT);
    gpio_init(in2_pin);
    gpio_set_dir(in2_pin, GPIO_OUT);
    printf("Motor pins setup: IN1=%d, IN2=%d\n", in1_pin, in2_pin);
}

void rotate_motor(const char *direction, uint in1_pin, uint in2_pin) {
    if (strcmp(direction, "forward") == 0) {
        gpio_put(in1_pin, 1);
        gpio_put(in2_pin, 0);
    } else if (strcmp(direction, "backward") == 0) {
        gpio_put(in1_pin, 0);
        gpio_put(in2_pin, 1);
    }
}

void start_motor(uint slice_num, int duty_cycle) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

void stop_motor(uint slice_num) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
}

void set_motor_speed(uint slice_num, int duty_cycle) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

void rotate_both_motors(const char *direction, int duty_cycle_motor1, int duty_cycle_motor2) {
    rotate_motor(direction, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    rotate_motor(direction, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), duty_cycle_motor1);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), duty_cycle_motor2);
}

void strong_start(const char *direction) {
    printf("Starting with high power burst\n");
    
    // Set direction
    rotate_motor(direction, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    rotate_motor(direction, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    
    // Initial high power burst
    printf("Phase 1: High power burst at %d\n", BURST_DUTY_CYCLE);
    start_motor(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), BURST_DUTY_CYCLE);
    start_motor(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), BURST_DUTY_CYCLE);
    
    sleep_ms(BURST_DURATION_MS);
    
    // Second phase - medium power
    printf("Phase 2: Medium power at %d\n", HIGH_POWER_DUTY);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), HIGH_POWER_DUTY);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), HIGH_POWER_DUTY);
    
    sleep_ms(HIGH_POWER_DURATION_MS);
    
    // Final phase - normal operating speed
    printf("Phase 3: Normal speed at %d\n", NORMAL_DUTY_CYCLE);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR1_PWM_PIN), NORMAL_DUTY_CYCLE);
    set_motor_speed(pwm_gpio_to_slice_num(MOTOR2_PWM_PIN), NORMAL_DUTY_CYCLE);
}

void pid_init(PIDController *pid, double kp, double ki, double kd, 
              double setpoint, double output_min, double output_max) {
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

double pid_update(PIDController *pid, double measurement) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    double dt = (current_time - pid->last_time) / 1000.0;
    
    double error = pid->setpoint - measurement;
    double p_term = pid->kp * error;
    
    pid->integral += error * dt;
    double i_term = pid->ki * pid->integral;
    
    double derivative = (error - pid->prev_error) / dt;
    double d_term = pid->kd * derivative;
    
    double output = p_term + i_term + d_term;
    
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    pid->prev_error = error;
    pid->last_time = current_time;
    
    return output;
}

void pid_reset(PIDController *pid) {
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->last_time = to_ms_since_boot(get_absolute_time());
}