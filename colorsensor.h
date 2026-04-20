#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
} tcs34725_raw_data_t;

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
    bool active;
    float integration_time_ms;
} TCS34725;

bool tcs34725_init(TCS34725 *sensor, i2c_inst_t *i2c, uint8_t address);
bool tcs34725_set_active(TCS34725 *sensor, bool value);
bool tcs34725_get_sensor_id(TCS34725 *sensor, uint8_t *sensor_id);
bool tcs34725_set_integration_time(TCS34725 *sensor, float value_ms);
float tcs34725_get_integration_time(const TCS34725 *sensor);
bool tcs34725_set_gain(TCS34725 *sensor, uint8_t gain);
bool tcs34725_get_gain(TCS34725 *sensor, uint8_t *gain);
bool tcs34725_read_raw(TCS34725 *sensor, tcs34725_raw_data_t *data);
bool tcs34725_read_temperature_and_lux(TCS34725 *sensor, float *temperature_k, float *lux);
bool tcs34725_get_threshold(TCS34725 *sensor, int *cycles, uint16_t *min_value, uint16_t *max_value);
bool tcs34725_set_threshold(TCS34725 *sensor, int cycles, bool set_cycles, uint16_t min_value, bool set_min, uint16_t max_value, bool set_max);
bool tcs34725_clear_interrupt(TCS34725 *sensor);
bool tcs34725_interrupt_enabled(TCS34725 *sensor, bool *enabled);

void tcs34725_html_rgb(const tcs34725_raw_data_t *data, float *red, float *green, float *blue);
void tcs34725_html_hex(const tcs34725_raw_data_t *data, char out[7]);
const char *tcs34725_detect_color_from_raw(const tcs34725_raw_data_t *data);
const char *tcs34725_detect_color(TCS34725 *sensor);

#endif