#include "ultrasonic.h"

const int timeout = 26000;  // Timeout for ultrasonic sensor (about 4.5 meters)
volatile absolute_time_t start_time;  // Start time for echo pulse
volatile uint64_t latest_pulse_width = 0;    // Pulse width in microseconds
volatile bool obstacleDetected = false;  // Flag to detect obstacles

kalman_state *kalman_init(double q, double r, double p, double initial_value) {
    kalman_state *state = calloc(1, sizeof(kalman_state));
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = initial_value;
    return state;
}

void kalman_update(kalman_state *state, double measurement) {
    state->p = state->p + state->q;
    state->k = state->p / (state->p + state->r);
    state->x = state->x + state->k * (measurement - state->x);
    state->p = (1 - state->k) * state->p;
}

void echo_isr(uint gpio, uint32_t events) {
    if (gpio == ECHOPIN) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            start_time = get_absolute_time();
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            latest_pulse_width = absolute_time_diff_us(start_time, get_absolute_time());
        }
    }
}

void setupUltrasonicPins() {
    // Initialize TRIG pin as output
    gpio_init(TRIGPIN);
    gpio_set_dir(TRIGPIN, GPIO_OUT);
    gpio_put(TRIGPIN, 0);  // Ensure TRIG starts low
    
    // Initialize ECHO pin as input
    gpio_init(ECHOPIN);
    gpio_set_dir(ECHOPIN, GPIO_IN);
    gpio_pull_down(ECHOPIN);  // Add pull-down resistor
    
    // Enable interrupts for ECHO pin
    gpio_set_irq_enabled_with_callback(ECHOPIN, 
                                     GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                     true, 
                                     &echo_isr);
    
    printf("Ultrasonic pins configured - TRIG: %d, ECHO: %d\n", TRIGPIN, ECHOPIN);
}

uint64_t getPulse() {
    latest_pulse_width = 0;  // Reset pulse width
    
    gpio_put(TRIGPIN, 1);    // Send trigger pulse
    sleep_us(10);            // 10 microseconds pulse
    gpio_put(TRIGPIN, 0);    // End trigger pulse

    // Wait for the pulse width to be updated by ISR
    absolute_time_t timeout_time = make_timeout_time_ms(30);
    while (latest_pulse_width == 0 && !time_reached(timeout_time)) {
        tight_loop_contents();
    }

    printf("Pulse width: %llu µs\n", latest_pulse_width);
    return latest_pulse_width;
}

double getCm(kalman_state *state) {
    uint64_t pulseLength = getPulse();
    if (pulseLength == 0 || pulseLength > timeout) {
        printf("Error: Pulse timeout or out of range.\n");
        return 0;
    }

    // Calculate distance in cm (pulse width / 58 gives cm)
    double measured = (double)pulseLength / 58.0;

    if (measured < 2 || measured > 400) {
        printf("Measured distance out of range: %.2f cm\n", measured);
        return 0;
    }

    kalman_update(state, measured);
    return state->x;
}