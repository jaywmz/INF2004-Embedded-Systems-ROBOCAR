#include "FreeRTOS.h"
#include "hardware/i2c.h"
#include "lwip/apps/mdns.h"
#include "lwip/apps/mqtt.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "task.h"
#include <math.h>

#define MQTT_SERVER_IP "172.20.10.2"
#define MQTT_SERVER_PORT 1883

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

#define THRESHHOLD 15
#define MAX_PITCH 90.0f // Max pitch angle for full speed
#define MAX_ROLL 90.0f // Max roll angle for full turn
#define MAX_SPEED 100.0f // Max speed percentage
#define ALPHA 0.5 // Smoothing factor

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

void update_speed(int16_t *pitch, int16_t *roll, char *g_direction, uint8_t *g_speed) {
    static uint8_t prev_speed;

    // Determining direction
    if (*pitch > THRESHHOLD && abs(*roll) < THRESHHOLD) {
        *g_direction = 'f';
    }
    else if (*pitch < -THRESHHOLD && abs(*roll) < THRESHHOLD) {
        *g_direction = 'b';
    }
    else if (*roll > THRESHHOLD && abs(*pitch) < THRESHHOLD) {
        *g_direction = 'r';
    }
    else if (*roll < -THRESHHOLD && abs(*pitch) < THRESHHOLD){
        *g_direction = 'l';
    }
    else {
        *g_direction = 's';
    }
    
    // Determining forward speed
    if (abs(*pitch) > THRESHHOLD && abs(*roll) < THRESHHOLD) {
        // find the speed proportionate to pitch angle
        uint8_t pitch_to_speed = (round)((abs((float)*pitch) / MAX_PITCH) * MAX_SPEED);
        
        // Apply exponential smoothing to speed
        *g_speed = ALPHA * pitch_to_speed + (1 - ALPHA) * prev_speed;

        // Update previous speed value
        prev_speed = *g_speed;

        return;
    }
    // // If no change in pitch, then determine left/right speed
    else if (abs(*roll) > THRESHHOLD && abs(*pitch) < THRESHHOLD) {
        // find the speed proportionate to roll angle
        uint8_t roll_to_speed = (round)((abs((float)*roll) / MAX_ROLL) * MAX_SPEED);
        
        // Apply exponential smoothing to speed
        *g_speed = ALPHA * roll_to_speed + (1 - ALPHA) * prev_speed;

        // Update previous speed value
        prev_speed = *g_speed;
        
        return;
    }
    else {
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

// == MQTT Configuration ==

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define DEBUG_printf printf

typedef struct MQTT_CLIENT_T_
{
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
    u32_t reconnect;
} MQTT_CLIENT_T;

// Perform initialisation
static MQTT_CLIENT_T *mqtt_client_init(void)
{
    MQTT_CLIENT_T *state = calloc(1, sizeof(MQTT_CLIENT_T));
    if (!state)
    {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    state->received = 0;
    return state;
}

static u32_t data_in = 0;
static u8_t buffer[1025];
static u8_t data_len = 0;

static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len)
{
    DEBUG_printf("mqtt_pub_start_cb: topic %s\n", topic);

    if (tot_len > 1024)
    {
        DEBUG_printf("Message length exceeds buffer size, discarding");
    }
    else
    {
        data_in = tot_len;
        data_len = 0;
    }
}

static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len,
                             u8_t flags)
{
    if (data_in > 0)
    {
        data_in -= len;
        memcpy(&buffer[data_len], data, len);
        data_len += len;

        if (data_in == 0)
        {
            buffer[data_len] = 0;
            DEBUG_printf("Message received: %s\n", &buffer);
        }
    }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status)
{
    if (status != 0)
    {
        DEBUG_printf("Error during connection: err %d.\n", status);
    }
    else
    {
        DEBUG_printf("MQTT connected.\n");
    }
}

void mqtt_pub_request_cb(void *arg, err_t err)
{
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)arg;
    DEBUG_printf("mqtt_pub_request_cb: err %d\n", err);
    state->received++;
}

void mqtt_sub_request_cb(void *arg, err_t err)
{
    DEBUG_printf("mqtt_sub_request_cb: err %d\n", err);
}

err_t mqtt_test_publish(MQTT_CLIENT_T *state)
{
    char buffer[32] = {0};

    sprintf(buffer, "{D:%c,S:%d%%}", g_direction, g_speed);

    err_t err;
    u8_t qos = 0;
    u8_t retain = 0;
    cyw43_arch_lwip_begin();
    err = mqtt_publish(state->mqtt_client, "pico_w/car", buffer, strlen(buffer),
                       qos, retain, mqtt_pub_request_cb, state);
    cyw43_arch_lwip_end();
    if (err != ERR_OK)
    {
        DEBUG_printf("Publish err: %d\n", err);
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    return err;
}

err_t mqtt_test_connect(MQTT_CLIENT_T *state)
{
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = "Compass";
    ci.client_user = NULL;
    ci.client_pass = NULL;
    ci.keep_alive = 60;
    ci.will_topic = NULL;
    ci.will_msg = NULL;
    ci.will_retain = 0;
    ci.will_qos = 0;

    const struct mqtt_connect_client_info_t *client_info = &ci;

    err = mqtt_client_connect(state->mqtt_client, &(state->remote_addr),
                              MQTT_SERVER_PORT, mqtt_connection_cb, state,
                              client_info);

    if (err != ERR_OK)
    {
        DEBUG_printf("mqtt_connect return %d\n", err);
    }

    return err;
}

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

void mqtt_task(__unused void *params)
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
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA3_WPA2_AES_PSK, 30000))
    {
        printf("SSID: %s\n" WIFI_SSID);
        printf("\nPW: %s\n", WIFI_PASSWORD);
        printf("failed to connect.\n");
    }
    printf("Connected.\n");

    mdns_resp_init();
    printf("mdns host name %s.local\n", hostname);
    mdns_resp_add_netif(&cyw43_state.netif[CYW43_ITF_STA], hostname);

    printf("\nReady, running on host %s\n",
           ip4addr_ntoa(netif_ip4_addr(netif_list)));

    MQTT_CLIENT_T *state = mqtt_client_init();
    ipaddr_aton(MQTT_SERVER_IP, &(state->remote_addr));

    state->mqtt_client = mqtt_client_new();
    state->counter = 0;

    if (state->mqtt_client == NULL)
    {
        DEBUG_printf("Failed to create new mqtt client\n");
        return;
    }

    while (mqtt_test_connect(state) != ERR_OK)
    {
        printf("attempting to connect...... \n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    absolute_time_t timeout = nil_time;
    bool subscribed = false;
    mqtt_set_inpub_callback(state->mqtt_client, mqtt_pub_start_cb,
                            mqtt_pub_data_cb, 0);

    while (true)
    {
        if (mqtt_client_is_connected(state->mqtt_client))
        {
            cyw43_arch_lwip_begin();

            mqtt_test_publish(state);
            cyw43_arch_lwip_end();
            // vTaskDelay(pdMS_TO_TICKS(50));
        }
        else
        {
            // DEBUG_printf(".");
        }
    }

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
    xTaskCreate(mqtt_task, "MqttTask", configMINIMAL_STACK_SIZE, NULL,
                TEST_TASK_PRIORITY, &mqtt_task_handle);

    vTaskStartScheduler();

    while (1)
    {
        tight_loop_contents();
    }
    return 0;
}
