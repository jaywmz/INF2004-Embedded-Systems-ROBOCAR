#include "FreeRTOS.h"
#include "hardware/pwm.h"
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
#include "queue.h"
#include "task.h"
#include <stdio.h>

// Define pins for Motor 1
#define MOTOR1_PWM_PIN 2
#define MOTOR1_IN1_PIN 0
#define MOTOR1_IN2_PIN 1

// Define pins for Motor 2
#define MOTOR2_PWM_PIN 6
#define MOTOR2_IN1_PIN 4
#define MOTOR2_IN2_PIN 5

// Variables for motor speeds
int duty_cycle_motor1 = 6250;
int duty_cycle_motor2 = 6250;

int g_p, g_r, g_y;

char prev[32] = {0};

// Define a queue to receive commands from user input
QueueHandle_t commandQueue;

// Setup PWM for speed control on a given motor
void setup_pwm(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_clkdiv(slice_num, 100);
    pwm_set_wrap(slice_num, 12500);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
    pwm_set_enabled(slice_num, true);
}

// Setup motor direction control pins
void setup_motor_pins(uint in1_pin, uint in2_pin) {
    gpio_init(in1_pin);
    gpio_set_dir(in1_pin, GPIO_OUT);
    gpio_init(in2_pin);
    gpio_set_dir(in2_pin, GPIO_OUT);
}

// Function to rotate motor in a specific direction
void rotate_motor(const char *direction, uint in1_pin, uint in2_pin) {
    if (strcmp(direction, "forward") == 0) {
        gpio_put(in1_pin, 1);
        gpio_put(in2_pin, 0);
    } else if (strcmp(direction, "backward") == 0) {
        gpio_put(in1_pin, 0);
        gpio_put(in2_pin, 1);
    }
}

// Start or stop a motor at a given duty cycle
void start_motor(uint slice_num, int duty_cycle) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty_cycle);
}

void stop_motor(uint slice_num) {
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
}

// Task to handle motor 1 commands
void motor1_task(void *params) {
    uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);

    while (true) {
        char command;
        if (xQueueReceive(commandQueue, &command, portMAX_DELAY) == pdPASS) {
            if (command == 'f') {
                rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
                start_motor(slice_num_motor1, duty_cycle_motor1);
                printf("Motor 1 rotating forward.\n");
            } else if (command == 'b') {
                rotate_motor("backward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
                start_motor(slice_num_motor1, duty_cycle_motor1);
                printf("Motor 1 rotating backward.\n");
            } else if (command == '+') {
                duty_cycle_motor1 = (duty_cycle_motor1 < 12500)
                                        ? duty_cycle_motor1 + 1250
                                        : 12500;
                pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A,
                                   duty_cycle_motor1);
                printf("Increased Motor 1 speed. Duty cycle: %d\n",
                       duty_cycle_motor1);
            } else if (command == '-') {
                duty_cycle_motor1 =
                    (duty_cycle_motor1 > 0) ? duty_cycle_motor1 - 1250 : 0;
                pwm_set_chan_level(slice_num_motor1, PWM_CHAN_A,
                                   duty_cycle_motor1);
                printf("Decreased Motor 1 speed. Duty cycle: %d\n",
                       duty_cycle_motor1);
            } else if (command == 's') {
                stop_motor(slice_num_motor1);
                printf("Motor 1 stopped.\n");
            }
        }
    }
}

// Task to handle motor 2 commands
void motor2_task(void *params) {
    uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);

    while (true) {
        char command;
        if (xQueueReceive(commandQueue, &command, portMAX_DELAY) == pdPASS) {
            if (command == 'f') {
                rotate_motor("forward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
                start_motor(slice_num_motor2, duty_cycle_motor2);
                printf("Motor 2 rotating forward.\n");
            } else if (command == 'b') {
                rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
                start_motor(slice_num_motor2, duty_cycle_motor2);
                printf("Motor 2 rotating backward.\n");
            } else if (command == '+') {
                duty_cycle_motor2 = (duty_cycle_motor2 < 12500)
                                        ? duty_cycle_motor2 + 1250
                                        : 12500;
                pwm_set_chan_level(slice_num_motor2, PWM_CHAN_A,
                                   duty_cycle_motor2);
                printf("Increased Motor 2 speed. Duty cycle: %d\n",
                       duty_cycle_motor2);
            } else if (command == '-') {
                duty_cycle_motor2 =
                    (duty_cycle_motor2 > 0) ? duty_cycle_motor2 - 1250 : 0;
                pwm_set_chan_level(slice_num_motor2, PWM_CHAN_A,
                                   duty_cycle_motor2);
                printf("Decreased Motor 2 speed. Duty cycle: %d\n",
                       duty_cycle_motor2);
            } else if (command == 's') {
                stop_motor(slice_num_motor2);
                printf("Motor 2 stopped.\n");
            }
        }
    }
}

// Task to read input from serial and send it to the queue
void input_task(void *params) {
    while (true) {
        char input = getchar_timeout_us(500000); // 500ms timeout
        if (input != PICO_ERROR_TIMEOUT) {
            xQueueSend(commandQueue, &input, portMAX_DELAY);
        }
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// #define MQTT_SERVER_IP "192.168.0.137"
#define MQTT_SERVER_IP "172.20.10.2"
#define MQTT_SERVER_PORT 1883

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

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

u8_t buffer[2049];
u8_t data_len = 0;

static int inpub_id;
static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("Incoming publish at topic %s with total length %u\n", topic,
           (unsigned int)tot_len);

    /* Decode topic string into a user defined reference */
    if (strcmp(topic, "pico_w/dashboard") == 0) {
        inpub_id = 0;
    } else if (topic[0] == 'A') {
        /* All topics starting with 'A' might be handled at the same way */
        inpub_id = 1;
    } else {
        /* For all other topics */
        inpub_id = 2;
    }
    // // DEBUG_printf("mqtt_pub_start_cb: topic %s\n", topic);

    // cyw43_arch_lwip_begin();
    if (tot_len > 1024) {
        DEBUG_printf("Message length exceeds buffer size, discarding");
    } else {
        data_in = tot_len;
        data_len = 0;
    }
    // cyw43_arch_lwip_end();
}

static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len,
                             u8_t flags) {
    if (data_in > 0) {
        data_in -= len;
        memcpy(&buffer[data_len], data, len);
        data_len += len;
        DEBUG_printf("%s\n", buffer);

        char command = buffer[0];
        uint slice_num_motor1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
        uint slice_num_motor2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
        if (prev[0] != 'f' && command == 'f') {
            prev[0] = 'f';
            rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(slice_num_motor1, duty_cycle_motor1);
            rotate_motor("forward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(slice_num_motor2, duty_cycle_motor2);
            printf("Motor rotating forward.\n");
        } else if (prev[0] != 'b' && command == 'b') {
            prev[0] = 'b';
            rotate_motor("backward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(slice_num_motor1, duty_cycle_motor1);
            rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(slice_num_motor2, duty_cycle_motor2);
            printf("Motor rotating backward.\n");
        } else if (prev[0] != 'r' && command == 'r') {
            prev[0] = 'r';
            rotate_motor("forward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(slice_num_motor1, duty_cycle_motor1);
            rotate_motor("backward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(slice_num_motor2, duty_cycle_motor2);
            printf("Motor rotating right.\n");
        } else if (prev[0] != 'l' && command == 'l') {
            prev[0] = 'l';
            rotate_motor("backward", MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
            start_motor(slice_num_motor1, duty_cycle_motor1);
            rotate_motor("forward", MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
            start_motor(slice_num_motor2, duty_cycle_motor2);
            printf("Motor rotating left.\n");
        } else if (prev[0] != 's' && command == 's') {
            prev[0] = 's';
            stop_motor(slice_num_motor1);
            stop_motor(slice_num_motor2);
            printf("Motor stopped.\n");
        }
        // int p, r, y;
        // sscanf(buffer, "p:%d,r:%d,y:%d", &g_p, &g_r, &g_y);
        // DEBUG_printf("p: %d, r: %d, y: %d\n", g_p, g_r, g_y);

        // if (input != PICO_ERROR_TIMEOUT) {
        //     xQueueSend(commandQueue, &input, portMAX_DELAY);
        // }

        if (data_in == 0) {
            // DEBUG_printf("%s\n", (const char *)data);
        }
    }
    printf("Incoming publish payload with length %d, flags %u\n", len,
           (unsigned int)flags);

    if (flags & MQTT_DATA_FLAG_LAST) {
        /* Last fragment of payload received (or whole part if payload fits
           receive buffer See MQTT_VAR_HEADER_BUFFER_LEN)  */

        /* Call function or do action depending on reference, in this case
         * inpub_id */
        if (inpub_id == 0) {
            /* Don't trust the publisher, check zero termination */
            // printf("%s\n", (const char *)data);
            // printf("%s\n", data[len]);
            if (data[len - 1] == 0) {
                printf("%s\n", (const char *)data);
            }
        } else if (inpub_id == 1) {
            /* Call an 'A' function... */
        } else {
            printf("mqtt_incoming_data_cb: Ignoring payload...\n");
        }
    } else {
        /* Handle fragmented payload, store in buffer, write to file or
        whatever
         */
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

    sprintf(buffer, "{\"message\":\"hello from picow %d / %d\"}",
            state->received, state->counter);

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

    ci.client_id = "Consumer";
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

    if (err != ERR_OK) {
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

void mqtt_task(__unused void *params) {
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
                            mqtt_pub_data_cb, mqtt_pub_data_cb);

    while (true) {
        if (mqtt_client_is_connected(state->mqtt_client)) {

            if (!subscribed) {
                cyw43_arch_lwip_begin();
                mqtt_subscribe(state->mqtt_client, "pico_w/dashboard", 0,
                               mqtt_sub_request_cb, state);
                subscribed = true;
                cyw43_arch_lwip_end();
            }
        }
    }

    cyw43_arch_deinit();
}

// Main setup function
void setup() {
    prev[0] = 's';
    stdio_init_all();
    sleep_ms(1000);

    // Setup motor control pins and PWM for both motors
    setup_motor_pins(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN);
    setup_pwm(MOTOR1_PWM_PIN);
    setup_motor_pins(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN);
    setup_pwm(MOTOR2_PWM_PIN);

    // Create a command queue
    commandQueue = xQueueCreate(10, sizeof(char));

    TaskHandle_t mqttTask, motor1Task, motor2Task, inputTask;

    // Create tasks for each motor and input handling
    xTaskCreate(mqtt_task, "Main Task", 2048, NULL, 1, &mqttTask);
    // xTaskCreate(motor1_task, "Motor 1 Task", 256, NULL, 2, &motor1Task);
    // xTaskCreate(motor2_task, "Motor 2 Task", 256, NULL, 2, &motor2Task);
    // xTaskCreate(heartbeat_task, "Heartbeat Task", 256, NULL, 1,
    // &heartbeatTask);

    printf("Motors are ready for control.\n");

    // Start the scheduler
    vTaskStartScheduler();
}

// Main function
int main() {
    setup();
    while (true) {
        // The main loop will not be reached; FreeRTOS tasks will run instead
    }
}
