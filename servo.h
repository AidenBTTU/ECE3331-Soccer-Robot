#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    unsigned int pin;
    unsigned int slice_num;
    float servo_pwm_freq;
    uint16_t min_u16_duty;
    uint16_t max_u16_duty;
    float min_angle;
    float max_angle;
    float current_angle;
    float angle_conversion_factor;
    bool initialized;
} Servo;

void servo_init(Servo *servo, unsigned int pin);
void servo_update_settings(
    Servo *servo,
    float servo_pwm_freq,
    uint16_t min_u16_duty,
    uint16_t max_u16_duty,
    float min_angle,
    float max_angle,
    unsigned int pin
);
void servo_move(Servo *servo, float angle);
void servo_stop(Servo *servo);
float servo_get_current_angle(const Servo *servo);

void servo_scoop_with(Servo *servo);
void servo_sort_with(Servo *servo, const char *color);

void servo_module_init(unsigned int pin);
void servo_scoop(void);
void servo_sort(const char *color);
Servo *servo_get_default(void);

#endif