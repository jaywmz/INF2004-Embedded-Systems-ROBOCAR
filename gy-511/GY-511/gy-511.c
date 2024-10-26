#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

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
void read_registers(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t len) {
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
void read_accel(int16_t* accel_data) {
    uint8_t buffer[6];
    read_registers(ACC_ADDR, ACC_OUT_X_L | 0x80, buffer, 6);

    accel_data[0] = (int16_t)(buffer[1] << 8 | buffer[0]); // X-axis
    accel_data[1] = (int16_t)(buffer[3] << 8 | buffer[2]); // Y-axis
    accel_data[2] = (int16_t)(buffer[5] << 8 | buffer[4]); // Z-axis
}

// read data from magnetometer
void read_mag(int16_t* mag_data) {
    uint8_t buffer[6];
    read_registers(MAG_ADDR, MAG_OUT_X_H, buffer, 6);

    mag_data[0] = (int16_t)(buffer[0] << 8 | buffer[1]); // X-axis
    mag_data[1] = (int16_t)(buffer[4] << 8 | buffer[5]); // Y-axis
    mag_data[2] = (int16_t)(buffer[2] << 8 | buffer[3]); // Z-axis
}

// Calculates the heading of the magnetometer from sensor data
float calculate_heading(int16_t mag_x, int16_t mag_y) {
    float heading = atan2((float)mag_y, (float)mag_x) * (180.0 / M_PI);
    if (heading < 0) {
        heading += 360;
    }
    return heading;
}

void process_accelerometer_data(float accel_x, float accel_y, float accel_z) {
    float gravity_x = 0.9, gravity_y = 0.9, gravity_z = 0.9;
    float linear_accel_x = 0, linear_accel_y = 0, linear_accel_z = 0;
    float alpha = 0.8;

    // Low-pass filter to estimate gravity
    gravity_x = alpha * gravity_x + (1 - alpha) * accel_x;
    gravity_y = alpha * gravity_y + (1 - alpha) * accel_y;
    gravity_z = alpha * gravity_z + (1 - alpha) * accel_z;

    // Subtract gravity to get linear acceleration
    linear_accel_x = accel_x - gravity_x;
    linear_accel_y = accel_y - gravity_y;
    linear_accel_z = accel_z - gravity_z;

    printf("    (Accel) X: %f Y: %f Z: %f", linear_accel_x, linear_accel_y, linear_accel_z);

    // Integrate acceleration to get velocity
    // velocity_x += linear_accel_x * delta_time;
    // velocity_y += linear_accel_y * delta_time;
    // velocity_z += linear_accel_z * delta_time;

    // printf("Velocity X: %.2f m/s, Y: %.2f m/s, Z: %.2f m/s\n", velocity_x, velocity_y, velocity_z);
}

void pitchAndRoll(float *accel_x, float *accel_y, float *accel_z, float *pitch, float *roll) {
    *pitch = atan2(*accel_y, sqrt(*accel_x * *accel_x + *accel_z * *accel_z)) * (180.0 / M_PI);
    *roll = atan2(*accel_x, sqrt(*accel_y * *accel_y + *accel_z * *accel_z)) * (180.0 / M_PI);
}

int main() {
    stdio_init_all();
    
    i2c_init_setup();
    
    accel_init();
    mag_init();

    int16_t accel_data[3];
    int16_t mag_data[3];

    while (1) {
        read_accel(accel_data);
        read_mag(mag_data);

        // get the heading from magnetometer data
        float heading = calculate_heading(mag_data[0], mag_data[1]);
        printf("Compass Heading: %.2f°", heading);

        // convert accelerometer data to g's
        float accel_x = accel_data[0] * 0.000061;   
        float accel_y = accel_data[1] * 0.000061;
        float accel_z = accel_data[2] * 0.000061;
        process_accelerometer_data(accel_x, accel_y, accel_z);

        // calculate pitch and roll angles of controller
        float pitch = 0.0;
        float roll = 0.0;

        pitchAndRoll(&accel_x, &accel_y, &accel_z, &pitch, &roll);

        printf("    Pitch: %.2f Roll: %.2f\n", pitch, roll);

        sleep_ms(200);
    }        
    
    return 0;
}