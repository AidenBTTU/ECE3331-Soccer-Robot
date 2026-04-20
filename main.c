#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/i2c.h"

#include "colorsensor.h"
#include "hcsr04.h"
#include "servo.h"

#define COLOR_I2C_PORT i2c1
#define COLOR_I2C_SDA 14
#define COLOR_I2C_SCL 15
#define COLOR_I2C_FREQ 100000
#define COLOR_SENSOR_ADDR 0x29

#define DIST_TRIGGER_PIN 27
#define DIST_ECHO_PIN 28
#define REBOOT_BUTTON 16
#define SCOOP_SERVO_PIN 0
#define SORT_SERVO_PIN 1
#define BEAM_SENSOR_PIN 26
#define BEAM_BROKEN_STATE 0

int main() {
    stdio_init_all();
    sleep_ms(3000);

    gpio_init(REBOOT_BUTTON);
    gpio_set_dir(REBOOT_BUTTON, GPIO_IN);
    gpio_pull_up(REBOOT_BUTTON);

    gpio_init(BEAM_SENSOR_PIN);
    gpio_set_dir(BEAM_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(BEAM_SENSOR_PIN);

    i2c_init(COLOR_I2C_PORT, COLOR_I2C_FREQ);
    gpio_set_function(COLOR_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(COLOR_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(COLOR_I2C_SDA);
    gpio_pull_up(COLOR_I2C_SCL);

    TCS34725 sensor;
    if (!tcs34725_init(&sensor, COLOR_I2C_PORT, COLOR_SENSOR_ADDR)) {
        printf("Color sensor init failed\n");
        while (true) {
            sleep_ms(1000);
        }
    }

    uint8_t sensor_id = 0;
    if (tcs34725_get_sensor_id(&sensor, &sensor_id)) {
        printf("Color sensor ID: 0x%02X\n", sensor_id);
    }

    HCSR04 distance_sensor;
    hcsr04_init(&distance_sensor, DIST_TRIGGER_PIN, DIST_ECHO_PIN, 0);

    Servo scoop_servo;
    Servo sort_servo;
    servo_init(&scoop_servo, SCOOP_SERVO_PIN);
    servo_init(&sort_servo, SORT_SERVO_PIN);
    servo_move(&scoop_servo, 0.0f);
    servo_move(&sort_servo, 0.0f);

    printf("Starting detect_color test\n");
    printf("Starting distance test on trig=%d echo=%d\n", DIST_TRIGGER_PIN, DIST_ECHO_PIN);
    printf("Starting scoop servo on pin=%d\n", SCOOP_SERVO_PIN);
    printf("Starting sort servo on pin=%d\n", SORT_SERVO_PIN);
    printf("Starting beam-break test on pin=%d\n", BEAM_SENSOR_PIN);

    bool beam_was_broken = false;

    while (true) {
        if (!gpio_get(REBOOT_BUTTON)) {
            printf("Rebooting to bootloader...\n");
            sleep_ms(500);
            reset_usb_boot(0, 0);
        } 

        bool beam_broken = gpio_get(BEAM_SENSOR_PIN) == 0;
        if (beam_broken && !beam_was_broken) {
            printf("Beam broken\n");
            servo_scoop_with(&scoop_servo);
        }
        beam_was_broken = beam_broken;

        tcs34725_raw_data_t data;
        if (tcs34725_read_raw(&sensor, &data)) {
            const char *detected = tcs34725_detect_color_from_raw(&data);
            if(detected != NULL){
                printf("color sesnor %s\r\n", detected);
                servo_sort_with(&sort_servo, detected);
            }
        } else {
            printf("Color sensor read failed\n");
        }

        long distance_mm = 0;
        if (hcsr04_distance_mm(&distance_sensor, &distance_mm)) {
            if(distance_mm < 100){
                printf("RUN\r\n");
            }
        } else {
            printf("Distance: out of range or no echo\n");
        }

        sleep_ms(100);
    }
}