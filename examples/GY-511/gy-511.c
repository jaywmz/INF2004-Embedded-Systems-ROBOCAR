#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// I2C Defines
#define SDA_PIN 4
#define SCL_PIN 5

// Accelerometer Address
#define ACC_ADDR 0x19

// Magnetometer Address
#define MAG_ADDR 0x1E

// Register Addresses for Accelerometer and Magnetometer
#define ACC_OUT_X_L 0x28
#define MAG_OUT_X_H 0x03
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define MR_REG_M 0x02

// set up i2c interface
void i2c_init_setup() {
    i2c_init(i2c_default, 400 * 1000); // 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// helper function to write to given register
void write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;
    i2c_write_blocking(i2c_default, addr, buffer, 2, false);
}

// helper function to read from given register
void read_registers(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len) {
    i2c_write_blocking(i2c_default, addr, &reg, 1, true); // Reg to read from
    i2c_read_blocking(i2c_default, addr, buffer, len, false);
}

// configures accelerometer by writing to the control registers
void accel_init() {
    // Enable accelerometer, 100Hz data rate, enable all axes
    write_register(ACC_ADDR, CTRL_REG1_A, 0x57);

    // Set sensitivity scale factor, enable high res mode
    write_register(ACC_ADDR, CTRL_REG4_A, 0x08);
}

// configures magnetometer by writing to the mode register
void mag_init() {
    // Continuous conversion mode
    write_register(MAG_ADDR, MR_REG_M, 0x00);
}

// read data from accelerometer registers
void read_accel(int16_t *accel_data) {
    uint8_t buffer[6];
    read_registers(ACC_ADDR, ACC_OUT_X_L | 0x80, buffer, 6);

    accel_data[0] = (int16_t)(buffer[1] << 8 | buffer[0]); // X-axis
    accel_data[1] = (int16_t)(buffer[3] << 8 | buffer[2]); // Y-axis
    accel_data[2] = (int16_t)(buffer[5] << 8 | buffer[4]); // Z-axis
}

// read data from magnetometer
void read_mag(int16_t *mag_data) {
    uint8_t buffer[6];
    read_registers(MAG_ADDR, MAG_OUT_X_H, buffer, 6);

    mag_data[0] = (int16_t)(buffer[0] << 8 | buffer[1]); // X-axis
    mag_data[1] = (int16_t)(buffer[4] << 8 | buffer[5]); // Y-axis
    mag_data[2] = (int16_t)(buffer[2] << 8 | buffer[3]); // Z-axis
}

void update_orientation(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                        int16_t mag_x, int16_t mag_y, int16_t mag_z,
                        int16_t *pitch, int16_t *roll, int16_t *yaw) {
    // Calculate pitch and roll from accelerometer
    *pitch = (int16_t)round(
        atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) *
        (180.0 / M_PI));
    *roll = (int16_t)round(
        atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) *
        (180.0 / M_PI));

    // Tilt-compensated yaw from magnetometer
    *yaw = (int16_t)round(atan2(mag_y, mag_x) * (180.0 / M_PI));
    if (*yaw < 0) {
        *yaw += 360;
    }
}

#define MAX_PITCH 30       // Max pitch angle for full speed
#define MAX_ROLL 30        // Max roll angle for full turn
#define MAX_DUTY_CYCLE 100 // Max PWM duty cycle percentage
#define MIN_DUTY_CYCLE 50  // Minimum to ensure motor response
#define MAX_TURN_DUTY_CYCLE 50
#define ALPHA 0.8 // Smoothing factor

uint16_t previous_left_duty = 0;
uint16_t previous_right_duty = 0;

void update_motor_duty(int16_t *pitch, int16_t *roll) {
    bool forward;

    if (*pitch < 0) {
        forward = false;
    } else {
        forward = true;
    }

    if (abs(*pitch) > 10) {
        // Finds the proportionately equivalent duty cycle of the given pitch
        float pitch_to_dutyCycle = (abs(*pitch) / MAX_PITCH) * MAX_DUTY_CYCLE;

        // Limits forward duty cycle within min and max
        float forward_dutyCycle =
            fmin(fmax(pitch_to_dutyCycle, MIN_DUTY_CYCLE), MAX_DUTY_CYCLE);

        // Calculate turning_speed from roll, allowing for left and right turns
        float turning_speed = (*roll / MAX_ROLL) * MAX_TURN_DUTY_CYCLE;

        // Clamp turning_speed to within [-MAX_TURN_DUTY_CYCLE,
        // MAX_TURN_DUTY_CYCLE]
        turning_speed = fmin(fmax(turning_speed, -MAX_TURN_DUTY_CYCLE),
                             MAX_TURN_DUTY_CYCLE);

        // Calculate left and right motor duties based on forward and turning
        // speeds
        float left_motor_duty = forward_dutyCycle + turning_speed;
        float right_motor_duty = forward_dutyCycle - turning_speed;

        // Clamp final motor duty cycles to stay within [MIN_DUTY_CYCLE,
        // MAX_DUTY_CYCLE]
        left_motor_duty =
            fmin(fmax(left_motor_duty, MIN_DUTY_CYCLE), MAX_DUTY_CYCLE);
        right_motor_duty =
            fmin(fmax(right_motor_duty, MIN_DUTY_CYCLE), MAX_DUTY_CYCLE);

        // Apply smoothing filter
        left_motor_duty =
            ALPHA * left_motor_duty + (1 - ALPHA) * previous_left_duty;
        right_motor_duty =
            ALPHA * right_motor_duty + (1 - ALPHA) * previous_right_duty;

        // Round to integer
        uint16_t left_dutyCycle = (uint16_t)round(left_motor_duty);
        uint16_t right_dutyCycle = (uint16_t)round(right_motor_duty);

        previous_left_duty = left_motor_duty;
        previous_right_duty = right_motor_duty;

        printf("Forwards: %d, Left Motor Duty: %u, Right Motor Duty: %u\n",
               forward, left_dutyCycle, right_dutyCycle);
    } else {
        return;
    }
}

int main() {
    stdio_init_all();

    i2c_init_setup();

    accel_init();
    mag_init();

    int16_t accel_data[3];
    int16_t mag_data[3];

    int16_t pitch = 0;
    int16_t roll = 0;
    int16_t yaw = 0;

    while (1) {
        read_accel(accel_data);
        read_mag(mag_data);

        // calculate pitch and roll angles of controller
        update_orientation(accel_data[0], accel_data[1], accel_data[2],
                           mag_data[0], mag_data[1], mag_data[2], &pitch, &roll,
                           &yaw);

        // translate pitch and roll to duty cycle for wheels
        update_motor_duty(&pitch, &roll);

        sleep_ms(100);
    }

    return 0;
}