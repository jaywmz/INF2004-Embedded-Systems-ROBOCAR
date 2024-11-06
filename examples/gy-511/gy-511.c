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

#define THRESHHOLD 15
#define MAX_PITCH 90.0f // Max pitch angle for full speed
#define MAX_ROLL 90.0f // Max roll angle for full turn
#define MAX_SPEED 100.0f // Max speed percentage
#define ALPHA 0.5 // Smoothing factor

// #define MAX_PITCH 90.0f         // Max pitch angle for full speed
// #define MAX_ROLL 90.0f          // Max roll angle for full turn
// #define MAX_DUTY_CYCLE 12500.0f // Max PWM duty cycle percentage
// #define MIN_DUTY_CYCLE 6250.5f  // Minimum to ensure motor response
// #define MAX_TURN_DUTY_CYCLE 6250.0f
// #define ALPHA 0.5 // Smoothing factor

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

    // Set sensitivity scale factor to +-2gs, enable high res mode
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

    // Raw data
    int16_t raw_x = (int16_t)(buffer[1] << 8 | buffer[0]); // X-axis
    int16_t raw_y = (int16_t)(buffer[3] << 8 | buffer[2]); // Y-axis
    int16_t raw_z = (int16_t)(buffer[5] << 8 | buffer[4]); // Z-axis

    // Static variables to store previous filtered values
    static float filtered_x = 0.0f;
    static float filtered_y = 0.0f;
    static float filtered_z = 0.0f;

    // Apply low-pass filter to each axis
    filtered_x = ALPHA * raw_x + (1 - ALPHA) * filtered_x;
    filtered_y = ALPHA * raw_y + (1 - ALPHA) * filtered_y;
    filtered_z = ALPHA * raw_z + (1 - ALPHA) * filtered_z;

    // Store the filtered results in accel_data output array
    accel_data[0] = (int16_t)filtered_x;
    accel_data[1] = (int16_t)filtered_y;
    accel_data[2] = (int16_t)filtered_z;
}

// read data from magnetometer, and low-pass filter using exponential moving average
void read_mag(int16_t* mag_data) {
    uint8_t buffer[6];
    read_registers(MAG_ADDR, MAG_OUT_X_H, buffer, 6);

    mag_data[0] = (int16_t)(buffer[1] << 8 | buffer[0]); // X-axis
    mag_data[1] = (int16_t)(buffer[3] << 8 | buffer[2]); // Y-axis
    mag_data[2] = (int16_t)(buffer[5] << 8 | buffer[4]); // Z-axis
}

void update_orientation(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                        int16_t mag_x, int16_t mag_y, int16_t mag_z, int16_t *pitch, int16_t *roll, int16_t *yaw) {
    // Calculate pitch and roll from accelerometer
    *pitch = (int16_t)round( atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * (180.0 / M_PI) );
    *roll = (int16_t)round( atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * (180.0 / M_PI) );

    // Tilt-compensated yaw from magnetometer
    *yaw = (int16_t)round( atan2(mag_y, mag_x) * (180.0 / M_PI) );
    if (*yaw < 0) {
        *yaw += 360;
    }

    // printf("Pitch: %d, Roll: %d, Yaw: %d\n", *pitch, *roll, *yaw);
}

// void update_motor_duty(int16_t *pitch, int16_t *roll, uint16_t *prev_left_duty, uint16_t *prev_right_duty, 
//                         char *direction, uint16_t *left_dutyCycle, uint16_t *right_dutyCycle) {
//     if (*pitch > 10) {
//         *direction = 'f';
//     }
//     else if (*pitch < -10) {
//         *direction = 'b';
//     }
//     else {
//         *direction = 's';
//     }

//     // Pitch angle threshold of 10 degrees
//     if (abs(*pitch) > 10) {
//         // Finds the proportionately equivalent duty cycle of the given pitch
//         float pitch_to_dutyCycle = ((float)abs(*pitch) / MAX_PITCH) * MAX_DUTY_CYCLE;

//         // Limits base duty cycle within min and max
//         float base_dutyCycle = fmin(fmax(pitch_to_dutyCycle, MIN_DUTY_CYCLE), MAX_DUTY_CYCLE);

//         // same 10 angle threshold for roll angle
//         float turning_speed;
//         if (abs(*roll) > 10) {
//             // Calculate turning_speed from roll, allowing for left and right turns
//             turning_speed = (*roll / MAX_ROLL) * MAX_TURN_DUTY_CYCLE;
//             // Clamp turning_speed to within [-MAX_TURN_DUTY_CYCLE, MAX_TURN_DUTY_CYCLE]
//             turning_speed = fmin(fmax(turning_speed, -MAX_TURN_DUTY_CYCLE), MAX_TURN_DUTY_CYCLE);
//         }
//         else {
//             turning_speed = 0.0f;
//         }

//         // Calculate left and right motor duties based on forward and turning speeds
//         float left_motor_duty = base_dutyCycle + turning_speed;
//         float right_motor_duty = base_dutyCycle - turning_speed;

//         // Clamp final motor duty cycles to stay within [MIN_DUTY_CYCLE, MAX_DUTY_CYCLE]
//         left_motor_duty = fmin(fmax(left_motor_duty, MIN_DUTY_CYCLE), MAX_DUTY_CYCLE);
//         right_motor_duty = fmin(fmax(right_motor_duty, MIN_DUTY_CYCLE), MAX_DUTY_CYCLE);

//         // Apply smoothing filter
//         left_motor_duty = ALPHA * left_motor_duty + (1 - ALPHA) * *prev_left_duty;
//         right_motor_duty = ALPHA * right_motor_duty + (1 - ALPHA) * *prev_right_duty;

//         // Round to integer
//         *left_dutyCycle = (uint16_t)round(left_motor_duty);
//         *right_dutyCycle = (uint16_t)round(right_motor_duty);

//         *prev_left_duty = left_motor_duty;
//         *prev_right_duty = right_motor_duty;
//     }
//     else {
//         // pitch angle does not pass threshold so give 0 dutycycle
//         *left_dutyCycle = 0;
//         *right_dutyCycle = 0;
//     }
// }

void update_speed(int16_t *pitch, int16_t *roll, char *direction, uint8_t *speed) {
    static uint8_t prev_speed;

    // Determining direction
    if (*pitch > THRESHHOLD && abs(*roll) < THRESHHOLD) {
        *direction = 'f';
    }
    else if (*pitch < -THRESHHOLD && abs(*roll) < THRESHHOLD) {
        *direction = 'b';
    }
    else if (*roll > THRESHHOLD && abs(*pitch) < THRESHHOLD) {
        *direction = 'r';
    }
    else if (*roll < -THRESHHOLD && abs(*pitch) < THRESHHOLD){
        *direction = 'l';
    }
    else {
        *direction = 's';
    }
    
    // Determining forward speed
    if (abs(*pitch) > THRESHHOLD && abs(*roll) < THRESHHOLD) {
        // find the speed proportionate to pitch angle
        uint8_t pitch_to_speed = (round)((abs((float)*pitch) / MAX_PITCH) * MAX_SPEED);
        
        // Apply exponential smoothing to speed
        *speed = ALPHA * pitch_to_speed + (1 - ALPHA) * prev_speed;

        // Update previous speed value
        prev_speed = *speed;

        return;
    }
    // // If no change in pitch, then determine left/right speed
    else if (abs(*roll) > THRESHHOLD && abs(*pitch) < THRESHHOLD) {
        // find the speed proportionate to roll angle
        uint8_t roll_to_speed = (round)((abs((float)*roll) / MAX_ROLL) * MAX_SPEED);
        
        // Apply exponential smoothing to speed
        *speed = ALPHA * roll_to_speed + (1 - ALPHA) * prev_speed;

        // Update previous speed value
        prev_speed = *speed;
        
        return;
    }
    else {
        // When no pitch or roll, speed will be zero
        *speed = 0;

        return;
    }
}

void print_controls(char *direction, uint8_t *speed) {
    printf("Direction: %c, Speed: %u%%\n", *direction, *speed);
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

    // uint16_t prev_left_duty = 0;
    // uint16_t prev_right_duty = 0;

    // uint16_t left_dutyCycle;
    // uint16_t right_dutyCycle;

    char direction;
    uint8_t speed;

    while (1) {
        read_accel(accel_data);
        read_mag(mag_data);

        // calculate pitch and roll angles of controller
        update_orientation(accel_data[0], accel_data[1], accel_data[2], mag_data[0], mag_data[1], mag_data[2], &pitch, &roll, &yaw);

        // translate pitch and roll to duty cycle for wheels
        // update_motor_duty(&pitch, &roll, &prev_left_duty, &prev_right_duty, &direction, &left_dutyCycle, &right_dutyCycle);

        // translate pitch and roll to percentage speed for PID
        update_speed(&pitch, &roll, &direction, &speed); 

        // Print controls
        print_controls(&direction, &speed);

        sleep_ms(100);
    }        
    
    return 0;
}