#include "mqtt.h"
#include "lwip/apps/mdns.h"
#include "lwip/apps/mqtt.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>

// #define MQTT_SERVER_IP "192.168.0.137"
#define MQTT_SERVER_IP "172.20.10.2"
#define MQTT_SERVER_PORT 1883

#define DEBUG_printf printf

typedef struct MQTT_CLIENT_T_
{
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
    u32_t reconnect;
} MQTT_CLIENT_T;

static u32_t data_in = 0;
static u8_t buffer[64];
static u8_t data_len = 0;
static MQTT_CLIENT_T *g_state = NULL;

static Compass *g_compass = NULL;

static char g_direction = 's';
static int g_left_duty = 0;
static int g_right_duty = 0;

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

static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len)
{
    // DEBUG_printf("mqtt_pub_start_cb: topic %s\n", topic);

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

static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    if (data_in > 0)
    {
        if (data_len + len > sizeof(buffer) - 1)
        {
            len = sizeof(buffer) - 1 - data_len;
        }
        memcpy(&buffer[data_len], data, len);
        data_len += len;
        data_in -= len;

        if (data_in == 0)
        {
            buffer[data_len] = '\0'; // Null-terminate the buffer

            // Parse the message
            int direction;
            if (sscanf((const char *)buffer, "{d:%d,l:%d,r:%d}", &direction, &g_left_duty, &g_right_duty) == 3)
            {
                g_direction = (char)direction;
                DEBUG_printf("Parsed values: g_direction=%d, g_left_duty=%d, g_right_duty=%d\n", direction, g_left_duty, g_right_duty);
            }
            else
            {
                DEBUG_printf("Failed to parse message: %s\n", buffer);
            }
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

static void mqtt_pub_request_cb(void *arg, err_t err)
{
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)arg;
    DEBUG_printf("mqtt_pub_request_cb: err %d\n", err);
    state->received++;
}

static void mqtt_sub_request_cb(void *arg, err_t err)
{
    DEBUG_printf("mqtt_sub_request_cb: err %d\n", err);
}

static err_t mqtt_test_publish(MQTT_CLIENT_T *state)
{
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
    if (err != ERR_OK)
    {
        DEBUG_printf("Publish err: %d\n", err);
    }

    return err;
}

static err_t mqtt_test_connect(MQTT_CLIENT_T *state)
{
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = "Car";
    ci.client_user = NULL;
    ci.client_pass = NULL;
    ci.keep_alive = 1;
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

void get_compass_data(Compass *compass)
{
    static bool subscribed = false;

    if (mqtt_client_is_connected(g_state->mqtt_client))
    {
        cyw43_arch_lwip_begin();
        if (!subscribed)
        {
            printf("Subscribing to topic pico_w/car\n");
            mqtt_sub_unsub(g_state->mqtt_client, "pico_w/car", 0,
                           mqtt_sub_request_cb, 0, 1);
            subscribed = true;
        }
        cyw43_arch_lwip_end();
    }
}

void init_mqtt()
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

    g_state = mqtt_client_init();
    ipaddr_aton(MQTT_SERVER_IP, &(g_state->remote_addr));

    g_state->mqtt_client = mqtt_client_new();
    g_state->counter = 0;

    if (g_state->mqtt_client == NULL)
    {
        DEBUG_printf("Failed to create new mqtt client\n");
        return;
    }

    while (mqtt_test_connect(g_state) != ERR_OK)
    {
        printf("attempting to connect...... \n");
    }
    mqtt_set_inpub_callback(g_state->mqtt_client, mqtt_pub_start_cb,
                            mqtt_pub_data_cb, 0);
}
