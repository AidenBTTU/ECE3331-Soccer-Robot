#include "colorsensor.h"

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"

#define TCS34725_COMMAND_BIT 0x80

#define TCS34725_REGISTER_ENABLE 0x00
#define TCS34725_REGISTER_ATIME 0x01
#define TCS34725_REGISTER_AILT 0x04
#define TCS34725_REGISTER_AIHT 0x06
#define TCS34725_REGISTER_APERS 0x0C
#define TCS34725_REGISTER_CONTROL 0x0F
#define TCS34725_REGISTER_SENSORID 0x12
#define TCS34725_REGISTER_STATUS 0x13
#define TCS34725_REGISTER_CDATA 0x14
#define TCS34725_REGISTER_RDATA 0x16
#define TCS34725_REGISTER_GDATA 0x18
#define TCS34725_REGISTER_BDATA 0x1A

#define TCS34725_ENABLE_AIEN 0x10
#define TCS34725_ENABLE_AEN 0x02
#define TCS34725_ENABLE_PON 0x01

static const unsigned char tcs34725_gains[] = {1, 4, 16, 60};
static const unsigned char tcs34725_cycles[] = {0, 1, 2, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60};

static bool tcs34725_register8(TCS34725 *sensor, unsigned char reg, unsigned char *value) {
    reg |= TCS34725_COMMAND_BIT;
    int rc = i2c_write_blocking(sensor->i2c, sensor->address, &reg, 1, true);
    if (rc != 1) {
        return false;
    }

    rc = i2c_read_blocking(sensor->i2c, sensor->address, value, 1, false);
    return rc == 1;
}

static bool tcs34725_write8(TCS34725 *sensor, unsigned char reg, unsigned char value) {
    unsigned char buf[2] = {(unsigned char)(reg | TCS34725_COMMAND_BIT), value};
    return i2c_write_blocking(sensor->i2c, sensor->address, buf, 2, false) == 2;
}

static bool tcs34725_register16(TCS34725 *sensor, unsigned char reg, unsigned short *value) {
    reg |= TCS34725_COMMAND_BIT;
    int rc = i2c_write_blocking(sensor->i2c, sensor->address, &reg, 1, true);
    if (rc != 1) {
        return false;
    }

    unsigned char data[2];
    rc = i2c_read_blocking(sensor->i2c, sensor->address, data, 2, false);
    if (rc != 2) {
        return false;
    }

    *value = (unsigned short)(data[0] | (data[1] << 8));
    return true;
}

static bool tcs34725_write16(TCS34725 *sensor, unsigned char reg, unsigned short value) {
    unsigned char buf[3] = {
        (unsigned char)(reg | TCS34725_COMMAND_BIT),
        (unsigned char)(value & 0xFF),
        (unsigned char)(value >> 8)
    };
    return i2c_write_blocking(sensor->i2c, sensor->address, buf, 3, false) == 3;
}

static bool tcs34725_valid(TCS34725 *sensor) {
    unsigned char status = 0;
    return tcs34725_register8(sensor, TCS34725_REGISTER_STATUS, &status) && ((status & 0x01u) != 0);
}

bool tcs34725_init(TCS34725 *sensor, i2c_inst_t *i2c, unsigned char address) {
    sensor->i2c = i2c;
    sensor->address = address;
    sensor->active = false;

    if (!tcs34725_set_integration_time(sensor, 2.4f)) {
        return false;
    }

    unsigned char sensor_id = 0;
    if (!tcs34725_get_sensor_id(sensor, &sensor_id)) {
        return false;
    }

    return sensor_id == 0x44 || sensor_id == 0x10 || sensor_id == 0x4D;
}

bool tcs34725_set_active(TCS34725 *sensor, bool value) {
    if (sensor->active == value) {
        return true;
    }

    unsigned char enable = 0;
    if (!tcs34725_register8(sensor, TCS34725_REGISTER_ENABLE, &enable)) {
        return false;
    }

    if (value) {
        if (!tcs34725_write8(sensor, TCS34725_REGISTER_ENABLE, (unsigned char)(enable | TCS34725_ENABLE_PON))) {
            return false;
        }
        sleep_ms(3);
        if (!tcs34725_write8(sensor, TCS34725_REGISTER_ENABLE, (unsigned char)(enable | TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN))) {
            return false;
        }
    } else {
        if (!tcs34725_write8(sensor, TCS34725_REGISTER_ENABLE, (unsigned char)(enable & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)))) {
            return false;
        }
    }

    sensor->active = value;
    return true;
}

bool tcs34725_get_sensor_id(TCS34725 *sensor, unsigned char *sensor_id) {
    return tcs34725_register8(sensor, TCS34725_REGISTER_SENSORID, sensor_id);
}

bool tcs34725_set_integration_time(TCS34725 *sensor, float value_ms) {
    if (value_ms < 2.4f) {
        value_ms = 2.4f;
    }
    if (value_ms > 614.4f) {
        value_ms = 614.4f;
    }

    int cycles = (int)(value_ms / 2.4f);
    if (cycles < 1) {
        cycles = 1;
    }
    sensor->integration_time_ms = cycles * 2.4f;

    return tcs34725_write8(sensor, TCS34725_REGISTER_ATIME, (unsigned char)(256 - cycles));
}

float tcs34725_get_integration_time(const TCS34725 *sensor) {
    return sensor->integration_time_ms;
}

bool tcs34725_set_gain(TCS34725 *sensor, unsigned char gain) {
    for (unsigned int i = 0; i < sizeof(tcs34725_gains) / sizeof(tcs34725_gains[0]); i++) {
        if (tcs34725_gains[i] == gain) {
            return tcs34725_write8(sensor, TCS34725_REGISTER_CONTROL, (unsigned char)i);
        }
    }

    return false;
}

bool tcs34725_get_gain(TCS34725 *sensor, unsigned char *gain) {
    unsigned char value = 0;
    if (!tcs34725_register8(sensor, TCS34725_REGISTER_CONTROL, &value)) {
        return false;
    }

    if (value >= sizeof(tcs34725_gains) / sizeof(tcs34725_gains[0])) {
        return false;
    }

    *gain = tcs34725_gains[value];
    return true;
}

bool tcs34725_read_raw(TCS34725 *sensor, tcs34725_raw_data_t *data) {
    bool was_active = sensor->active;
    if (!tcs34725_set_active(sensor, true)) {
        return false;
    }

    while (!tcs34725_valid(sensor)) {
        sleep_ms((unsigned int)(sensor->integration_time_ms + 0.9f));
    }

    bool ok = tcs34725_register16(sensor, TCS34725_REGISTER_RDATA, &data->r) &&
              tcs34725_register16(sensor, TCS34725_REGISTER_GDATA, &data->g) &&
              tcs34725_register16(sensor, TCS34725_REGISTER_BDATA, &data->b) &&
              tcs34725_register16(sensor, TCS34725_REGISTER_CDATA, &data->c);

    if (!was_active) {
        tcs34725_set_active(sensor, false);
    }

    return ok;
}

bool tcs34725_read_temperature_and_lux(TCS34725 *sensor, float *temperature_k, float *lux) {
    tcs34725_raw_data_t data;
    if (!tcs34725_read_raw(sensor, &data)) {
        return false;
    }

    float x = -0.14282f * data.r + 1.54924f * data.g - 0.95641f * data.b;
    float y = -0.32466f * data.r + 1.57837f * data.g - 0.73191f * data.b;
    float z = -0.68202f * data.r + 0.77073f * data.g + 0.56332f * data.b;
    float d = x + y + z;
    if (d == 0.0f || (0.1858f - y / d) == 0.0f) {
        return false;
    }

    float n = (x / d - 0.3320f) / (0.1858f - y / d);
    *temperature_k = 449.0f * n * n * n + 3525.0f * n * n + 6823.3f * n + 5520.33f;
    *lux = y;
    return true;
}

bool tcs34725_get_threshold(TCS34725 *sensor, int *cycles, unsigned short *min_value, unsigned short *max_value) {
    if (!tcs34725_register16(sensor, TCS34725_REGISTER_AILT, min_value) ||
        !tcs34725_register16(sensor, TCS34725_REGISTER_AIHT, max_value)) {
        return false;
    }

    unsigned char enable = 0;
    if (!tcs34725_register8(sensor, TCS34725_REGISTER_ENABLE, &enable)) {
        return false;
    }

    if ((enable & TCS34725_ENABLE_AIEN) == 0) {
        *cycles = -1;
        return true;
    }

    unsigned char persistence = 0;
    if (!tcs34725_register8(sensor, TCS34725_REGISTER_APERS, &persistence)) {
        return false;
    }

    persistence &= 0x0F;
    *cycles = tcs34725_cycles[persistence];
    return true;
}

bool tcs34725_set_threshold(TCS34725 *sensor, int cycles, bool set_cycles, unsigned short min_value, bool set_min, unsigned short max_value, bool set_max) {
    if (set_min && !tcs34725_write16(sensor, TCS34725_REGISTER_AILT, min_value)) {
        return false;
    }
    if (set_max && !tcs34725_write16(sensor, TCS34725_REGISTER_AIHT, max_value)) {
        return false;
    }

    if (set_cycles) {
        unsigned char enable = 0;
        if (!tcs34725_register8(sensor, TCS34725_REGISTER_ENABLE, &enable)) {
            return false;
        }

        if (cycles == -1) {
            return tcs34725_write8(sensor, TCS34725_REGISTER_ENABLE, (unsigned char)(enable & ~TCS34725_ENABLE_AIEN));
        }

        unsigned char persistence_index = 0xFF;
        for (unsigned int i = 0; i < sizeof(tcs34725_cycles) / sizeof(tcs34725_cycles[0]); i++) {
            if (tcs34725_cycles[i] == cycles) {
                persistence_index = (unsigned char)i;
                break;
            }
        }
        if (persistence_index == 0xFF) {
            return false;
        }

        return tcs34725_write8(sensor, TCS34725_REGISTER_ENABLE, (unsigned char)(enable | TCS34725_ENABLE_AIEN)) &&
               tcs34725_write8(sensor, TCS34725_REGISTER_APERS, persistence_index);
    }

    return true;
}

bool tcs34725_clear_interrupt(TCS34725 *sensor) {
    unsigned char cmd = 0xE6;
    return i2c_write_blocking(sensor->i2c, sensor->address, &cmd, 1, false) == 1;
}

bool tcs34725_interrupt_enabled(TCS34725 *sensor, bool *enabled) {
    unsigned char status = 0;
    if (!tcs34725_register8(sensor, TCS34725_REGISTER_STATUS, &status)) {
        return false;
    }

    *enabled = (status & TCS34725_ENABLE_AIEN) != 0;
    return true;
}

void tcs34725_html_rgb(const tcs34725_raw_data_t *data, float *red, float *green, float *blue) {
    if (data->c == 0) {
        *red = 0.0f;
        *green = 0.0f;
        *blue = 0.0f;
        return;
    }

    *red = powf(((float)((int)(((float)data->r / data->c) * 256.0f)) / 255.0f), 2.5f) * 255.0f;
    *green = powf(((float)((int)(((float)data->g / data->c) * 256.0f)) / 255.0f), 2.5f) * 255.0f;
    *blue = powf(((float)((int)(((float)data->b / data->c) * 256.0f)) / 255.0f), 2.5f) * 255.0f;
}

void tcs34725_html_hex(const tcs34725_raw_data_t *data, char out[7]) {
    float red = 0.0f;
    float green = 0.0f;
    float blue = 0.0f;
    tcs34725_html_rgb(data, &red, &green, &blue);
    snprintf(out, 7, "%02x%02x%02x", (int)red, (int)green, (int)blue);
}

const char *tcs34725_detect_color_from_raw(const tcs34725_raw_data_t *data) {
    if (data->c == 0) {
        return NULL;
    }

    float red_pct = (float)data->r / data->c;
    float green_pct = (float)data->g / data->c;
    float blue_pct = (float)data->b / data->c;

    if (red_pct * 0.8f > green_pct && red_pct * 0.7f > blue_pct) {
        return "RED";
    }
    if (green_pct * 0.9f > red_pct && green_pct * 0.7f > blue_pct) {
        return "GREEN";
    }
    if (blue_pct > red_pct && blue_pct * 1.2f > green_pct) {
        return "BLUE";
    }

    return NULL;
}

const char *tcs34725_detect_color(TCS34725 *sensor) {
    tcs34725_raw_data_t data;
    if (!tcs34725_read_raw(sensor, &data)) {
        return NULL;
    }
    return tcs34725_detect_color_from_raw(&data);
}