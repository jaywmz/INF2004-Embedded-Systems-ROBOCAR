#include "FreeRTOS.h"
#include "hardware/i2c.h"

#include "task.h"
#include <math.h>

#include <string.h>
#include <stdlib.h>

#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/apps/mdns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define UDP_PORT 4444
#define BEACON_MSG_LEN_MAX 127
#define BEACON_TARGET "255.255.255.255"
#define BEACON_INTERVAL_MS 50

// == Compass Configuration ==

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

#define THRESHHOLD 20
#define MAX_PITCH 90.0f  // Max pitch angle for full speed
#define MAX_ROLL 90.0f   // Max roll angle for full turn
#define MAX_SPEED 100.0f // Max speed percentage
#define ALPHA 0.5        // Smoothing factor

// Global variables
uint8_t g_speed;
char g_direction;

// set up i2c interface
void i2c_init_setup()
{
    i2c_init(i2c_default, 400 * 1000); // 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// helper function to write to given register
void write_register(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = value;
    i2c_write_blocking(i2c_default, addr, buffer, 2, false);
}

// helper function to read from given register
void read_registers(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    i2c_write_blocking(i2c_default, addr, &reg, 1, true); // Reg to read from
    i2c_read_blocking(i2c_default, addr, buffer, len, false);
}

// configures accelerometer by writing to the control registers
void accel_init()
{
    // Enable accelerometer, 100Hz data rate, enable all axes
    write_register(ACC_ADDR, CTRL_REG1_A, 0x57);
    // Set sensitivity scale factor, enable high res mode
    write_register(ACC_ADDR, CTRL_REG4_A, 0x08);
}

// configures magnetometer by writing to the mode register
void mag_init()
{
    // Continuous conversion mode
    write_register(MAG_ADDR, MR_REG_M, 0x00);
}

// read data from accelerometer registers
void read_accel(int16_t *accel_data)
{
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

// read data from magnetometer
void read_mag(int16_t *mag_data)
{
    uint8_t buffer[6];
    read_registers(MAG_ADDR, MAG_OUT_X_H, buffer, 6);

    mag_data[0] = (int16_t)(buffer[0] << 8 | buffer[1]); // X-axis
    mag_data[1] = (int16_t)(buffer[4] << 8 | buffer[5]); // Y-axis
    mag_data[2] = (int16_t)(buffer[2] << 8 | buffer[3]); // Z-axis
}

void update_orientation(int16_t accel_x, int16_t accel_y, int16_t accel_z,
                        int16_t mag_x, int16_t mag_y, int16_t mag_z, int16_t *pitch, int16_t *roll, int16_t *yaw)
{
    // Calculate pitch and roll from accelerometer
    *pitch = (int16_t)round(atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * (180.0 / M_PI));
    *roll = (int16_t)round(atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * (180.0 / M_PI));

    // Calculate yaw from magnetometer
    *yaw = (int16_t)round(atan2(mag_y, mag_x) * (180.0 / M_PI));
    if (*yaw < 0)
    {
        *yaw += 360;
    }
}

void update_speed(int16_t *pitch, int16_t *roll, char *g_direction, uint8_t *g_speed)
{
    static uint8_t prev_speed;

    // Determining direction
    if (*pitch > THRESHHOLD && abs(*roll) < THRESHHOLD)
    {
        *g_direction = 'f';
    }
    else if (*pitch < -THRESHHOLD && abs(*roll) < THRESHHOLD)
    {
        *g_direction = 'b';
    }
    else if (*roll > THRESHHOLD && abs(*pitch) < THRESHHOLD)
    {
        *g_direction = 'r';
    }
    else if (*roll < -THRESHHOLD && abs(*pitch) < THRESHHOLD)
    {
        *g_direction = 'l';
    }
    else
    {
        *g_direction = 's';
    }

    // Determining forward speed
    if (abs(*pitch) > THRESHHOLD && abs(*roll) < THRESHHOLD)
    {
        // find the speed proportionate to pitch angle
        uint8_t pitch_to_speed = (round)((abs((float)*pitch) / MAX_PITCH) * MAX_SPEED);

        // Apply exponential smoothing to speed
        *g_speed = ALPHA * pitch_to_speed + (1 - ALPHA) * prev_speed;

        // Update previous speed value
        prev_speed = *g_speed;

        return;
    }
    // // If no change in pitch, then determine left/right speed
    else if (abs(*roll) > THRESHHOLD && abs(*pitch) < THRESHHOLD)
    {
        // find the speed proportionate to roll angle
        uint8_t roll_to_speed = (round)((abs((float)*roll) / MAX_ROLL) * MAX_SPEED);

        // Apply exponential smoothing to speed
        *g_speed = ALPHA * roll_to_speed + (1 - ALPHA) * prev_speed;

        // Update previous speed value
        prev_speed = *g_speed;

        return;
    }
    else
    {
        // When no pitch or roll, speed will be zero
        *g_speed = 0;

        return;
    }
}

void read_motor_task(__unused void *params)
{
    int16_t accel_data[3];
    int16_t mag_data[3];
    int16_t pitch, roll, yaw;

    uint16_t prev_left_duty = 0;
    uint16_t prev_right_duty = 0;

    while (1)
    {
        read_accel(accel_data);
        read_mag(mag_data);

        // calculate pitch and roll angles of controller
        update_orientation(accel_data[0], accel_data[1], accel_data[2], mag_data[0], mag_data[1], mag_data[2], &pitch, &roll, &yaw);

        // translate pitch and roll to duty cycle for wheels
        // update_motor_duty(&pitch, &roll, &prev_left_duty, &prev_right_duty, &g_direction, &g_left_dutyCycle, &g_right_dutyCycle);

        // translate pitch and roll to percentage speed for PID
        update_speed(&pitch, &roll, &g_direction, &g_speed);
    }
}

// == UDP Configuration ==

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define DEBUG_printf printf

// Return some characters from the ascii representation of the mac address
// e.g. 112233445566
// chr_off is index of character in mac to start
// chr_len is length of result
// chr_off=8 and chr_len=4 would return "5566"
// Return number of characters put into destination
static size_t get_mac_ascii(int idx, size_t chr_off, size_t chr_len,
                            char *dest_in)
{
    static const char hexchr[16] = "0123456789ABCDEF";
    uint8_t mac[6];
    char *dest = dest_in;
    assert(chr_off + chr_len <= (2 * sizeof(mac)));
    cyw43_hal_get_mac(idx, mac);
    for (; chr_len && (chr_off >> 1) < sizeof(mac); ++chr_off, --chr_len)
    {
        *dest++ = hexchr[mac[chr_off >> 1] >> (4 * (1 - (chr_off & 1))) & 0xf];
    }
    return dest - dest_in;
}

// Function to map direction character to number
int map_direction_to_number(char direction)
{
    switch (direction)
    {
    case 'f':
        return 1; // Forward
    case 'b':
        return 2; // Backward
    case 'l':
        return 3; // Left
    case 'r':
        return 4; // Right
    default:
        return 0; // Unknown
    }
}

void udp_task(__unused void *params)
{
    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    char hostname[sizeof(CYW43_HOST_NAME) + 4];
    memcpy(&hostname[0], CYW43_HOST_NAME, sizeof(CYW43_HOST_NAME) - 1);
    get_mac_ascii(CYW43_HAL_MAC_WLAN0, 8, 4,
                  &hostname[sizeof(CYW43_HOST_NAME) - 1]);
    hostname[sizeof(hostname) - 1] = '\0';
    netif_set_hostname(&cyw43_state.netif[CYW43_ITF_STA], hostname);

    printf("Connecting to WiFi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("SSID: %s\n" WIFI_SSID);
        printf("\nPW: %s\n", WIFI_PASSWORD);
        printf("failed to connect.\n");
    }
    printf("Connected.\n");

    mdns_resp_init();
    printf("mdns host name %s.local\n", hostname);
    mdns_resp_add_netif(&cyw43_state.netif[CYW43_ITF_STA], hostname);

    printf("Ready, running on host %s\n",
           ip4addr_ntoa(netif_ip4_addr(netif_list)));

    struct udp_pcb *pcb = udp_new();
    printf("Created UDP connection\n");

    if (!pcb)
    {
        printf("Error creating PCB\n");
        return;
    }
    printf("Created PCB\n");

    ip_addr_t addr;
    ipaddr_aton(BEACON_TARGET, &addr);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, BEACON_MSG_LEN_MAX + 1, PBUF_RAM);
    if (!p)
    {
        printf("Error allocating pbuf\n");
        return;
    }
    printf("Allocated pbuf\n");

    while (1)
    {
        char *req = (char *)p->payload;
        memset(req, 0, BEACON_MSG_LEN_MAX + 1);
        snprintf(req, BEACON_MSG_LEN_MAX, "{d:%d,s:%d}\n", map_direction_to_number(g_direction), g_speed);
        err_t er = udp_sendto(pcb, p, &addr, UDP_PORT);
        if (er != ERR_OK)
        {
            printf("Failed to send UDP packet! error=%d", er);
        }
        else
        {
            printf("{d:%d,s:%d}\n", map_direction_to_number(g_direction), g_speed);
        }

        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(BEACON_INTERVAL_MS));
    }
    pbuf_free(p);

    cyw43_arch_deinit();
}

// Main Function
int main()
{
    stdio_init_all();
    i2c_init_setup();
    accel_init();
    mag_init();

    TaskHandle_t read_motor_task_handle, mqtt_task_handle;
    xTaskCreate(read_motor_task, "ReadMotorTask", configMINIMAL_STACK_SIZE,
                NULL, TEST_TASK_PRIORITY, &read_motor_task_handle);
    xTaskCreate(udp_task, "UdpTask", configMINIMAL_STACK_SIZE, NULL,
                TEST_TASK_PRIORITY, &mqtt_task_handle);

    vTaskStartScheduler();

    return 0;
}