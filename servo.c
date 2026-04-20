#include "servo.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define SERVO_DEFAULT_PWM_FREQ 50.0f
#define SERVO_DEFAULT_MIN_U16_DUTY 1802
#define SERVO_DEFAULT_MAX_U16_DUTY 7864
#define SERVO_DEFAULT_MIN_ANGLE 0.0f
#define SERVO_DEFAULT_MAX_ANGLE 180.0f
#define SERVO_PWM_WRAP 19999
#define SERVO_PWM_CLKDIV 125.0f

static Servo default_servo;

static uint16_t servo_angle_to_u16_duty(const Servo *servo, float angle) {
    return (uint16_t)((angle - servo->min_angle) * servo->angle_conversion_factor) + servo->min_u16_duty;
}

static uint16_t servo_u16_to_level(uint16_t duty_u16) {
    return (uint16_t)(SERVO_PWM_WRAP - ((uint32_t)duty_u16 * SERVO_PWM_WRAP) / 65535u);
}

static void servo_apply_pwm_config(Servo *servo, unsigned int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    servo->pin = pin;
    servo->slice_num = pwm_gpio_to_slice_num(pin);

    pwm_set_clkdiv(servo->slice_num, SERVO_PWM_CLKDIV);
    pwm_set_wrap(servo->slice_num, SERVO_PWM_WRAP);
    pwm_set_enabled(servo->slice_num, true);

    servo->initialized = true;
}

void servo_init(Servo *servo, unsigned int pin) {
    servo_update_settings(
        servo,
        SERVO_DEFAULT_PWM_FREQ,
        SERVO_DEFAULT_MIN_U16_DUTY,
        SERVO_DEFAULT_MAX_U16_DUTY,
        SERVO_DEFAULT_MIN_ANGLE,
        SERVO_DEFAULT_MAX_ANGLE,
        pin
    );
}

void servo_update_settings(
    Servo *servo,
    float servo_pwm_freq,
    uint16_t min_u16_duty,
    uint16_t max_u16_duty,
    float min_angle,
    float max_angle,
    unsigned int pin
) {
    servo->servo_pwm_freq = servo_pwm_freq;
    servo->min_u16_duty = min_u16_duty;
    servo->max_u16_duty = max_u16_duty;
    servo->min_angle = min_angle;
    servo->max_angle = max_angle;
    servo->current_angle = -0.001f;
    servo->angle_conversion_factor =
        (float)(max_u16_duty - min_u16_duty) / (max_angle - min_angle);

    servo_apply_pwm_config(servo, pin);
}

void servo_move(Servo *servo, float angle) {
    if (!servo->initialized) {
        return;
    }

    angle = roundf(angle * 100.0f) / 100.0f;
    if (angle < servo->min_angle) {
        angle = servo->min_angle;
    }
    if (angle > servo->max_angle) {
        angle = servo->max_angle;
    }

    if (angle == servo->current_angle) {
        return;
    }

    servo->current_angle = angle;
    uint16_t duty_u16 = servo_angle_to_u16_duty(servo, angle);
    pwm_set_gpio_level(servo->pin, servo_u16_to_level(duty_u16));
}

void servo_stop(Servo *servo) {
    if (!servo->initialized) {
        return;
    }

    pwm_set_enabled(servo->slice_num, false);
    servo->initialized = false;
}

float servo_get_current_angle(const Servo *servo) {
    return servo->current_angle;
}

void servo_scoop_with(Servo *servo) {
    if (servo == NULL) {
        return;
    }

    servo_move(servo, 180.0f);
    sleep_ms(2000);
    servo_move(servo, 0.0f);
    sleep_ms(1000);
}

void servo_sort_with(Servo *servo, const char *color) {
    if (servo == NULL || color == NULL) {
        return;
    }

    if (strcmp(color, "BLUE") == 0 || strcmp(color, "GREEN") == 0) {
        printf("opening g/b\n");
        servo_move(servo, 90.0f);
        sleep_ms(1000);
    } else if (strcmp(color, "RED") == 0) {
        printf("opening red\n");
        servo_move(servo, 0.0f);
        sleep_ms(1000);
    }
}

void servo_module_init(unsigned int pin) {
    servo_init(&default_servo, pin);
}

void servo_scoop(void) {
    servo_scoop_with(&default_servo);
}

void servo_sort(const char *color) {
    servo_sort_with(&default_servo, color);
}

Servo *servo_get_default(void) {
    return &default_servo;
}