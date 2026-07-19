#include "global.h"

#define HCSR04_DEFAULT_ECHO_TIMEOUT_US (500u * 2u * 30u)

void hcsr04_init(HCSR04 *sensor, unsigned int trigger_pin, unsigned int echo_pin, unsigned int echo_timeout_us) {
    sensor->trigger_pin = trigger_pin;
    sensor->echo_pin = echo_pin;
    sensor->echo_timeout_us = echo_timeout_us ? echo_timeout_us : HCSR04_DEFAULT_ECHO_TIMEOUT_US;

    gpio_init(sensor->trigger_pin);
    gpio_set_dir(sensor->trigger_pin, GPIO_OUT);
    gpio_put(sensor->trigger_pin, 0);

    gpio_init(sensor->echo_pin);
    gpio_set_dir(sensor->echo_pin, GPIO_IN);
}

bool hcsr04_send_pulse_and_wait(HCSR04 *sensor, long *pulse_time_us) {
    gpio_put(sensor->trigger_pin, 0);
    sleep_us(5);
    gpio_put(sensor->trigger_pin, 1);
    sleep_us(10);
    gpio_put(sensor->trigger_pin, 0);

    uint64_t start_wait = time_us_64();
    while (!gpio_get(sensor->echo_pin)) {
        if ((time_us_64() - start_wait) > sensor->echo_timeout_us) {
            return false;
        }
    }

    uint64_t pulse_start = time_us_64();
    while (gpio_get(sensor->echo_pin)) {
        if ((time_us_64() - pulse_start) > sensor->echo_timeout_us) {
            return false;
        }
    }

    *pulse_time_us = (long)(time_us_64() - pulse_start);
    return true;
}

bool hcsr04_distance_mm(HCSR04 *sensor, long *distance_mm) {
    long pulse_time_us = 0;
    if (!hcsr04_send_pulse_and_wait(sensor, &pulse_time_us)) {
        return false;
    }

    *distance_mm = pulse_time_us * 100 / 582;
    return true;
}

bool hcsr04_distance_cm(HCSR04 *sensor, float *distance_cm) {
    long pulse_time_us = 0;
    if (!hcsr04_send_pulse_and_wait(sensor, &pulse_time_us)) {
        return false;
    }

    *distance_cm = ((float)pulse_time_us / 2.0f) / 29.1f;
    return true;
}