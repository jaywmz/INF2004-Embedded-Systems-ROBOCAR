#include <stdio.h>
#include <math.h>

#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define UDP_PORT 4444
#define BEACON_MSG_LEN_MAX 127
#define BEACON_TARGET "255.255.255.255"
#define BEACON_INTERVAL_MS 50

static struct udp_pcb *udp_server_pcb;
static struct udp_pcb *g_sender_pcb;
static ip_addr_t g_addr;
static struct pbuf *g_pbuf;

static int g_left_encoder_speed = 0;
static int g_right_encoder_speed = 0;
static int g_ultrasonic_distance = 0;
static char g_decoded_char = ' ';

static void udp_server_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL)
    {
        // printf("Received: %s\n", (char *)p->payload);
        if (sscanf((char *)p->payload, "{ls:%d,rs:%d,d:%d,c:%c}", &g_left_encoder_speed, &g_right_encoder_speed, &g_ultrasonic_distance, &g_decoded_char))
        {
            printf("===\nLeft Encoder Speed: %d\nRight Encoder Speed: %d\nUltrasonic Distance: %dcm\nCharacter: %c\n", g_left_encoder_speed, g_right_encoder_speed, g_ultrasonic_distance, g_decoded_char);
        }
        pbuf_free(p);
    }
}

static void udp_server_init()
{
    udp_server_pcb = udp_new();
    if (!udp_server_pcb)
    {
        printf("Error creating PCB\n");
        return;
    }

    err_t err = udp_bind(udp_server_pcb, IP_ADDR_ANY, UDP_PORT);
    if (err != ERR_OK)
    {
        printf("Error binding PCB: %d\n", err);
        return;
    }

    udp_recv(udp_server_pcb, udp_server_recv, NULL);
    printf("UDP server listening on port %d\n", UDP_PORT);
}

void init_udp()
{
    if (cyw43_arch_init())
    {
        printf("Wi-Fi init failed\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Failed to connect to Wi-Fi\n");
        printf("%s : %s\n", WIFI_SSID, WIFI_PASSWORD);
        sleep_ms(1000);
    }

    printf("Connected to Wi-Fi\n");
    udp_server_init();
}

// Main function for system initialization and launching tasks
int main(void)
{
    stdio_init_all(); // Initialize standard I/O for debugging
    init_udp();
    while (1)
    {
        // printf("Hello, world!\n");
        // tight_loop_contents();
    }

    return 0;
}
