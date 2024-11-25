#ifndef PICO_UDP_H
#define PICO_UDP_H

#include <stdint.h>

typedef struct Compass
{
    uint16_t direction;
    uint16_t speed;
    uint16_t manual_mode;
} Compass;

typedef struct Telemetry
{
    // TODO: Add telemetry data

} Telemetry;

void init_udp();
void poll_wifi();
void get_compass_data(Compass *compass);
void send_telemetry(Telemetry *telemetry);

#endif