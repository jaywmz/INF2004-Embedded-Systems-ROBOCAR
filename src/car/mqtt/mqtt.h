#ifndef PICO_MQTT_H
#define PICO_MQTT_H

typedef struct Compass {
    int p;
    int r;
    int y;
} Compass;

typedef struct Telemetry {
    // TODO: Add telemetry data
} Telemetry;

void init_mqtt();
void get_compass_data(Compass *compass);
void send_telemetry(Telemetry *telemetry);

#endif