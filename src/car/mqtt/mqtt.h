#ifndef PICO_MQTT_H
#define PICO_MQTT_H

#include <stdint.h>

typedef struct Compass
{
    int p;
    int r;
    int y;
    char direction;
    uint16_t left_duty;
    uint16_t right_duty;
} Compass;

typedef struct Telemetry
{
    // TODO: Add telemetry data
} Telemetry;

void init_mqtt();
void get_compass_data(Compass *compass);
void send_telemetry(Telemetry *telemetry);

#endif