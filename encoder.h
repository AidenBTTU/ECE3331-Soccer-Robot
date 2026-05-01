#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    unsigned int pin_a;
    unsigned int pin_b;
    volatile int32_t count;
    int32_t last_reported;
    uint8_t last_state;
    bool initialized;
} Encoder;

void encoder_init(Encoder *encoder, unsigned int pin_a, unsigned int pin_b);
int32_t encoder_get_count(const Encoder *encoder);
int32_t encoder_delta(Encoder *encoder);
int32_t encoder_delta_cm(Encoder *encoder);
void encoder_reset(Encoder *encoder);

#endif