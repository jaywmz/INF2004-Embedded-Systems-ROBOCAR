#include "udp.h"
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define UDP_PORT 4444

static struct udp_pcb *udp_server_pcb;

static int g_direction = 0;
static int g_speed = 0;

static void udp_server_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL)
    {
        // printf("Received packet from %s:%d\n", ipaddr_ntoa(addr), port);
        sscanf((char *)p->payload, "{d:%d,s:%d}", &g_direction, &g_speed);
        // printf("Data: %.*s\n", p->len, (char *)p->payload);
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

void get_compass_data(Compass *compass)
{
    cyw43_arch_poll();
    printf("{d:%d,s:%d}\n", g_direction, g_speed);
    compass->direction = g_direction;
    compass->speed = g_speed;
}