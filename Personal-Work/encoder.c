#include "global.h"

#define ENCODER_MAX_PIN 30
#define _USE_MATH_DEFINES
#include <math.h>

static Encoder *encoder_by_pin[ENCODER_MAX_PIN];

static const int8_t encoder_transition_table[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

static void encoder_gpio_callback(unsigned int gpio, uint32_t events) {
    (void)events;
    if (gpio >= ENCODER_MAX_PIN) {
        return;
    }
    Encoder *encoder = encoder_by_pin[gpio];
    if (encoder == NULL) {
        return;
    }

    uint8_t a = (uint8_t)gpio_get(encoder->pin_a);
    uint8_t b = (uint8_t)gpio_get(encoder->pin_b);
    uint8_t state = (uint8_t)((a << 1) | b);
    uint8_t transition = (uint8_t)((encoder->last_state << 2) | state);
    encoder->count += encoder_transition_table[transition];
    encoder->last_state = state;
}

void encoder_init(Encoder *encoder, unsigned int pin_a, unsigned int pin_b) {
    encoder->pin_a = pin_a;
    encoder->pin_b = pin_b;
    encoder->count = 0;
    encoder->last_reported = 0;
    encoder->initialized = true;

    gpio_init(pin_a);
    gpio_set_dir(pin_a, GPIO_IN);
    gpio_pull_up(pin_a);
    gpio_init(pin_b);
    gpio_set_dir(pin_b, GPIO_IN);
    gpio_pull_up(pin_b);

    encoder->last_state = (uint8_t)((gpio_get(pin_a) << 1) | gpio_get(pin_b));

    if (pin_a < ENCODER_MAX_PIN) {
        encoder_by_pin[pin_a] = encoder;
    }
    if (pin_b < ENCODER_MAX_PIN) {
        encoder_by_pin[pin_b] = encoder;
    }

    gpio_set_irq_enabled_with_callback(pin_a, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_gpio_callback);
    gpio_set_irq_enabled(pin_b, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

int32_t encoder_get_count(const Encoder *encoder) {
    return encoder->count;
}

int32_t encoder_delta(Encoder *encoder) {
    int32_t current = encoder->count;
    int32_t delta = current - encoder->last_reported;
    encoder->last_reported = current;
    return delta;
}

int32_t encoder_delta_cm(Encoder *encoder) {
    return encoder_delta(encoder) * 80 * M_PI / 1920; // Convert encoder ticks to centimeters
}

void encoder_reset(Encoder *encoder) {
    encoder->count = 0;
    encoder->last_reported = 0;
}
