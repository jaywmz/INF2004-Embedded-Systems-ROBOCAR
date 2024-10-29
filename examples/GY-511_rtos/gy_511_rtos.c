#include "FreeRTOS.h"
#include "hardware/i2c.h"
#include "lwip/api.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/mdns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/init.h"
#include "lwip/ip4_addr.h"
#include "lwip/netdb.h"
#include "lwip/pbuf.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "semphr.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// #define MQTT_SERVER_IP "192.168.0.137"
#define MQTT_SERVER_IP "172.20.10.2"
#define MQTT_SERVER_PORT 1883

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

bool g_forward = false;
uint16_t g_left_dutyCycle = 0;
uint16_t g_right_dutyCycle = 0;

#define DEBUG_printf printf

typedef struct MQTT_CLIENT_T_ {
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
    u32_t reconnect;
} MQTT_CLIENT_T;

// Perform initialisation
static MQTT_CLIENT_T *mqtt_client_init(void) {
    MQTT_CLIENT_T *state = calloc(1, sizeof(MQTT_CLIENT_T));
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    state->received = 0;
    return state;
}

u32_t data_in = 0;

u8_t buffer[1025];
u8_t data_len = 0;

static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len) {
    DEBUG_printf("mqtt_pub_start_cb: topic %s\n", topic);

    if (tot_len > 1024) {
        DEBUG_printf("Message length exceeds buffer size, discarding");
    } else {
        data_in = tot_len;
        data_len = 0;
    }
}

static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len,
                             u8_t flags) {
    if (data_in > 0) {
        data_in -= len;
        memcpy(&buffer[data_len], data, len);
        data_len += len;

        if (data_in == 0) {
            buffer[data_len] = 0;
            DEBUG_printf("Message received: %s\n", &buffer);
        }
    }
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status) {
    if (status != 0) {
        DEBUG_printf("Error during connection: err %d.\n", status);
    } else {
        DEBUG_printf("MQTT connected.\n");
    }
}

void mqtt_pub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)arg;
    DEBUG_printf("mqtt_pub_request_cb: err %d\n", err);
    state->received++;
}

void mqtt_sub_request_cb(void *arg, err_t err) {
    DEBUG_printf("mqtt_sub_request_cb: err %d\n", err);
}

err_t mqtt_test_publish(MQTT_CLIENT_T *state) {
    char buffer[128];

    // sprintf(buffer, "{\"message\":\"hello from picow %d / %d\"}",
    //         state->received, state->counter);
    sprintf(buffer, "Forwards: %d, Left Motor Duty: %u, Right Motor Duty: %u",
            g_forward, g_left_dutyCycle, g_right_dutyCycle);

    err_t err;
    u8_t qos = 0; /* 0 1 or 2, see MQTT specification.  AWS IoT does not support
                     QoS 2 */
    u8_t retain = 0;
    cyw43_arch_lwip_begin();
    err = mqtt_publish(state->mqtt_client, "pico_w/dashboard", buffer,
                       strlen(buffer), qos, retain, mqtt_pub_request_cb, state);
    cyw43_arch_lwip_end();
    if (err != ERR_OK) {
        DEBUG_printf("Publish err: %d\n", err);
    }

    return err;
}

err_t mqtt_test_connect(MQTT_CLIENT_T *state) {
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = "PicoW";
    ci.client_user = NULL;
    ci.client_pass = NULL;
    ci.keep_alive = 0;
    ci.will_topic = NULL;
    ci.will_msg = NULL;
    ci.will_retain = 0;
    ci.will_qos = 0;

    const struct mqtt_connect_client_info_t *client_info = &ci;

    err = mqtt_client_connect(state->mqtt_client, &(state->remote_addr),
                              MQTT_SERVER_PORT, mqtt_connection_cb, state,
                              client_info);

    if (err != ERR_OK) {
        DEBUG_printf("mqtt_connect return %d\n", err);
    }

    return err;
}

void mqtt_run_test(MQTT_CLIENT_T *state) {

    while (true) {
    }
}

// Return some characters from the ascii representation of the mac address
// e.g. 112233445566
// chr_off is index of character in mac to start
// chr_len is length of result
// chr_off=8 and chr_len=4 would return "5566"
// Return number of characters put into destination
static size_t get_mac_ascii(int idx, size_t chr_off, size_t chr_len,
                            char *dest_in) {
    static const char hexchr[16] = "0123456789ABCDEF";
    uint8_t mac[6];
    char *dest = dest_in;
    assert(chr_off + chr_len <= (2 * sizeof(mac)));
    cyw43_hal_get_mac(idx, mac);
    for (; chr_len && (chr_off >> 1) < sizeof(mac); ++chr_off, --chr_len) {
        *dest++ = hexchr[mac[chr_off >> 1] >> (4 * (1 - (chr_off & 1))) & 0xf];
    }
    return dest - dest_in;
}

void main_task(__unused void *params) {
    if (cyw43_arch_init()) {
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
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA3_WPA2_AES_PSK, 30000)) {
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

    if (state->mqtt_client == NULL) {
        DEBUG_printf("Failed to create new mqtt client\n");
        return;
    }

    while (mqtt_test_connect(state) != ERR_OK) {
        printf("attempting to connect...... \n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    absolute_time_t timeout = nil_time;
    bool subscribed = false;
    mqtt_set_inpub_callback(state->mqtt_client, mqtt_pub_start_cb,
                            mqtt_pub_data_cb, 0);

    while (true) {
        if (mqtt_client_is_connected(state->mqtt_client)) {
            cyw43_arch_lwip_begin();

            if (!subscribed) {
                mqtt_sub_unsub(state->mqtt_client, "pico_w/recv", 0,
                               mqtt_sub_request_cb, 0, 1);
                subscribed = true;
            }

            if (mqtt_test_publish(state) == ERR_OK) {
                // if (state->counter != 0) {
                //     DEBUG_printf("published %d\n", state->counter);
                // }
                // timeout = make_timeout_time_ms(5000);
                // state->counter++;
            } // else ringbuffer is full and we need to wait for
              // messages to flush.
            cyw43_arch_lwip_end();
        } else {
            // DEBUG_printf(".");
        }

        // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 ms
    }

    cyw43_arch_deinit();
}

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

        // printf("Forwards: %d, Left Motor Duty: %u, Right Motor Duty: %u\n",
        //        forward, left_dutyCycle, right_dutyCycle);

        g_forward = forward;
        g_left_dutyCycle = left_dutyCycle;
        g_right_dutyCycle = right_dutyCycle;
    } else {
        return;
    }
}

void read_motor_task(__unused void *params) {
    int16_t accel_data[3];
    int16_t mag_data[3];
    int16_t pitch, roll, yaw;

    while (1) {
        read_accel(accel_data);
        read_mag(mag_data);

        update_orientation(accel_data[0], accel_data[1], accel_data[2],
                           mag_data[0], mag_data[1], mag_data[2], &pitch, &roll,
                           &yaw);

        update_motor_duty(&pitch, &roll);
        // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 ms
    }
}

// Main Function
int main() {
    stdio_init_all();
    i2c_init_setup();
    accel_init();
    mag_init();

    TaskHandle_t task;

    xTaskCreate(read_motor_task, "ReadMotorTask", configMINIMAL_STACK_SIZE,
                NULL, TEST_TASK_PRIORITY, &task);
    xTaskCreate(main_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL,
                TEST_TASK_PRIORITY, &task);

    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }
    return 0;
}
