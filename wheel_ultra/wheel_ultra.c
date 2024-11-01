#include "wheel_ultra.h"

// Define global variables here
volatile absolute_time_t start_time;
volatile uint64_t latest_pulse_width = 0;
volatile bool obstacleDetected = false;

MessageBufferHandle_t left_buffer = NULL;
MessageBufferHandle_t right_buffer = NULL;
TaskHandle_t left_encoder_task_handle = NULL;
TaskHandle_t right_encoder_task_handle = NULL;

// Kalman Filter Functions
kalman_state *kalman_init(double q, double r, double p, double initial_value) {
    kalman_state *state = (kalman_state *)calloc(1, sizeof(kalman_state));
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;
    return state;
}

void kalman_update(kalman_state *state, double measurement) {
    state->p += state->q;
    state->k = state->p / (state->p + state->r);
    state->x += state->k * (measurement - state->x);
    state->p *= (1 - state->k);
}

// Ultrasonic Sensor Functions
void setupUltrasonicPins() {
    gpio_init(TRIGPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_put(TRIGPIN, 0);
    
    gpio_init(ECHOPIN);
    gpio_set_dir(ECHOPIN, GPIO_IN);
    gpio_pull_down(ECHOPIN);
    
    printf("Ultrasonic pins configured - TRIG: %d, ECHO: %d\n", TRIGPIN, ECHOPIN);
}

uint64_t getPulse() {
    latest_pulse_width = 0;
    
    gpio_put(TRIGPIN, 1);
    sleep_us(10);
    gpio_put(TRIGPIN, 0);

    absolute_time_t timeout_time = make_timeout_time_ms(30);
    while (latest_pulse_width == 0 && !time_reached(timeout_time)) {
        tight_loop_contents();
    }

    printf("Pulse width: %llu µs\n", latest_pulse_width);
    return latest_pulse_width;
}

double getCm(kalman_state *state) {
    uint64_t pulseLength = getPulse();
    if (pulseLength == 0 || pulseLength > ULTRASONIC_TIMEOUT) {
        printf("Error: Pulse timeout or out of range.\n");
        return 0;
    }

    double measured = (double)pulseLength / 58.0;
    if (measured < 2 || measured > 400) {
        printf("Measured distance out of range: %.2f cm\n", measured);
        return 0;
    }

    kalman_update(state, measured);
    return state->x;
}

// Encoder Data Logging Function
void log_encoder_data(const char *wheel, EncoderData *data) {
    printf("%s Wheel -> Pulses: %d, Speed: %.2f m/s, Total Distance: %.2f meters, Pulse Width: %llu us\n",
           wheel, data->pulse_count, data->speed_m_per_s, data->distance_m, data->pulse_width_us);
}

// Unified ISR for Ultrasonic and Encoder Sensors
void unified_gpio_callback(uint gpio, uint32_t events) {
    uint64_t current_time = time_us_64();

    // Ultrasonic Echo Pulse
    if (gpio == ECHOPIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            start_time = get_absolute_time();
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            latest_pulse_width = absolute_time_diff_us(start_time, get_absolute_time());
        }
    }
    // Left Wheel Encoder
    else if (gpio == LEFT_ENCODER_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        static uint32_t left_pulse_count = 0;
        static float left_total_distance = 0.0;
        static uint64_t last_left_pulse_time = 0;

        uint64_t pulse_width_us = current_time - last_left_pulse_time;
        last_left_pulse_time = current_time;

        left_pulse_count++;
        float left_speed = (pulse_width_us > 0 && pulse_width_us < MICROSECONDS_IN_A_SECOND)
                           ? DISTANCE_PER_NOTCH / (pulse_width_us / MICROSECONDS_IN_A_SECOND) : 0;

        left_total_distance += DISTANCE_PER_NOTCH;
        EncoderData data = {left_pulse_count, left_speed, left_total_distance, pulse_width_us};

        if (xMessageBufferSendFromISR(left_buffer, &data, sizeof(data), NULL) == pdFALSE) {
            printf("Error: Left buffer is full\n");
        }
    }
    // Right Wheel Encoder
    else if (gpio == RIGHT_ENCODER_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        static uint32_t right_pulse_count = 0;
        static float right_total_distance = 0.0;
        static uint64_t last_right_pulse_time = 0;

        uint64_t pulse_width_us = current_time - last_right_pulse_time;
        last_right_pulse_time = current_time;

        right_pulse_count++;
        float right_speed = (pulse_width_us > 0 && pulse_width_us < MICROSECONDS_IN_A_SECOND)
                            ? DISTANCE_PER_NOTCH / (pulse_width_us / MICROSECONDS_IN_A_SECOND) : 0;

        right_total_distance += DISTANCE_PER_NOTCH;
        EncoderData data = {right_pulse_count, right_speed, right_total_distance, pulse_width_us};

        if (xMessageBufferSendFromISR(right_buffer, &data, sizeof(data), NULL) == pdFALSE) {
            printf("Error: Right buffer is full\n");
        }
    }
}

// Tasks for Processing Encoder Data
void process_left_encoder_task(void *pvParameters) {
    EncoderData data;
    while (1) {
        if (xMessageBufferReceive(left_buffer, &data, sizeof(data), portMAX_DELAY) > 0) {
            log_encoder_data("Left", &data);
        }
    }
}

void process_right_encoder_task(void *pvParameters) {
    EncoderData data;
    while (1) {
        if (xMessageBufferReceive(right_buffer, &data, sizeof(data), portMAX_DELAY) > 0) {
            log_encoder_data("Right", &data);
        }
    }
}

void ultrasonic_task(void *pvParameters) {
    kalman_state *ultrasonic_kalman = kalman_init(0.1, 0.1, 1, 0);
    setupUltrasonicPins();

    while (1) {
        double distance = getCm(ultrasonic_kalman);
        if (distance > 0) {
            printf("Ultrasonic Distance: %.2f cm\n", distance);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay to allow time for other tasks
    }
}

// Initialization Function for All Sensors and Tasks
void system_init() {
    // Message buffers
    left_buffer = xMessageBufferCreate(256);
    right_buffer = xMessageBufferCreate(256);
    if (left_buffer == NULL || right_buffer == NULL) {
        printf("Error: Message buffer allocation failed\n");
        return;
    }

    // Initialize GPIOs
    gpio_init(TRIGPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_put(TRIGPIN, 0);
    gpio_init(ECHOPIN);
    gpio_set_dir(ECHOPIN, GPIO_IN);
    gpio_pull_down(ECHOPIN);

    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_PIN);

    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_PIN);

    // Attach unified ISR to both encoder and ultrasonic pins
    gpio_set_irq_enabled_with_callback(ECHOPIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &unified_gpio_callback);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &unified_gpio_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &unified_gpio_callback);

    // Create tasks
    xTaskCreate(process_left_encoder_task, "Process Left Encoder Task", 1024, NULL, 1, &left_encoder_task_handle);
    xTaskCreate(process_right_encoder_task, "Process Right Encoder Task", 1024, NULL, 1, &right_encoder_task_handle);
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 1024, NULL, 1, NULL);
}

