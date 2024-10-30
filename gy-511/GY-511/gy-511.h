#ifndef GY_511_H
#define GY_511_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// GPIO pins for I2C
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

#define MAX_PITCH 50 // Max pitch angle for full speed
#define MAX_ROLL 50 // Max roll angle for full turn
#define MAX_DUTY_CYCLE 100 // Max PWM duty cycle percentage
#define MIN_DUTY_CYCLE 10 // Minimum to ensure motor response
#define MAX_TURN_DUTY_CYCLE 50
#define ALPHA 0.8 // Smoothing factor

void i2c_init_setup();
void write_register(uint8_t addr, uint8_t reg, uint8_t value);
void read_registers(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len);
void accel_init();
void mag_init();
void read_accel(int16_t* accel_data);
void read_mag(int16_t* mag_data);
void update_orientation(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                        int16_t mag_x, int16_t mag_y, int16_t mag_z, int16_t *pitch, int16_t *roll, int16_t *yaw);
void update_motor_duty(int16_t *pitch, int16_t *roll, uint16_t *prev_left_duty, uint16_t *prev_right_duty, 
                        bool *forward, uint16_t *left_dutyCycle, uint16_t *right_dutyCycle);
void print_controls(bool *forward, uint16_t *left_dutyCycle, uint16_t *right_dutyCycle);

#endif