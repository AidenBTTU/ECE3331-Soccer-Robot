#ifndef HCSR04_H
#define HCSR04_H

#include <stdbool.h>

typedef struct {
    unsigned int trigger_pin;
    unsigned int echo_pin;
    unsigned int echo_timeout_us;
} HCSR04;

void hcsr04_init(HCSR04 *sensor, unsigned int trigger_pin, unsigned int echo_pin, unsigned int echo_timeout_us);
bool hcsr04_send_pulse_and_wait(HCSR04 *sensor, long *pulse_time_us);
bool hcsr04_distance_mm(HCSR04 *sensor, long *distance_mm);
bool hcsr04_distance_cm(HCSR04 *sensor, float *distance_cm);

#endif