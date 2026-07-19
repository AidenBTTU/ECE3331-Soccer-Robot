#include "global.h"

#define AK8963_WIA 0x00
#define AK8963_HXL 0x03
#define AK8963_ST2 0x09
#define AK8963_CNTL1 0x0A
#define AK8963_ASAX 0x10

#define AK8963_SO_14BIT 0.6f
#define AK8963_SO_16BIT 0.15f

static bool ak8963_read_u8(AK8963 *sensor, uint8_t reg, uint8_t *value) {
    int rc = i2c_write_blocking(sensor->i2c, sensor->address, &reg, 1, true);
    if (rc != 1) {
        return false;
    }

    rc = i2c_read_blocking(sensor->i2c, sensor->address, value, 1, false);
    return rc == 1;
}

static bool ak8963_write_u8(AK8963 *sensor, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_blocking(sensor->i2c, sensor->address, buf, 2, false) == 2;
}

static bool ak8963_read_three_shorts(AK8963 *sensor, uint8_t reg, int16_t *x, int16_t *y, int16_t *z) {
    int rc = i2c_write_blocking(sensor->i2c, sensor->address, &reg, 1, true);
    if (rc != 1) {
        return false;
    }

    uint8_t buf[6];
    rc = i2c_read_blocking(sensor->i2c, sensor->address, buf, 6, false);
    if (rc != 6) {
        return false;
    }

    *x = (int16_t)(buf[0] | (buf[1] << 8));
    *y = (int16_t)(buf[2] | (buf[3] << 8));
    *z = (int16_t)(buf[4] | (buf[5] << 8));
    return true;
}

bool ak8963_init(
    AK8963 *sensor,
    i2c_inst_t *i2c,
    uint8_t address,
    uint8_t mode,
    uint8_t output,
    ak8963_vector_t offset,
    ak8963_vector_t scale
) {
    sensor->i2c = i2c;
    sensor->address = address;
    sensor->mode = mode;
    sensor->output = output;
    sensor->offset = offset;
    sensor->scale = scale;

    uint8_t whoami = 0;
    if (!ak8963_get_whoami(sensor, &whoami) || whoami != 0x48) {
        return false;
    }

    if (!ak8963_write_u8(sensor, AK8963_CNTL1, AK8963_MODE_FUSE_ROM_ACCESS)) {
        return false;
    }

    uint8_t asax = 0;
    uint8_t asay = 0;
    uint8_t asaz = 0;
    if (!ak8963_read_u8(sensor, AK8963_ASAX, &asax) ||
        !ak8963_read_u8(sensor, AK8963_ASAX + 1, &asay) ||
        !ak8963_read_u8(sensor, AK8963_ASAX + 2, &asaz)) {
        return false;
    }

    if (!ak8963_write_u8(sensor, AK8963_CNTL1, AK8963_MODE_POWER_DOWN)) {
        return false;
    }

    sleep_us(100);

    sensor->adjustment.x = (0.5f * (asax - 128)) / 128 + 1;
    sensor->adjustment.y = (0.5f * (asay - 128)) / 128 + 1;
    sensor->adjustment.z = (0.5f * (asaz - 128)) / 128 + 1;

    if (!ak8963_write_u8(sensor, AK8963_CNTL1, (uint8_t)(mode | output))) {
        return false;
    }

    sensor->so = (output == AK8963_OUTPUT_16_BIT) ? AK8963_SO_16BIT : AK8963_SO_14BIT;
    return true;
}

bool ak8963_get_whoami(AK8963 *sensor, uint8_t *whoami) {
    return ak8963_read_u8(sensor, AK8963_WIA, whoami);
}

bool ak8963_get_adjustment(const AK8963 *sensor, ak8963_vector_t *adjustment) {
    *adjustment = sensor->adjustment;
    return true;
}

bool ak8963_get_magnetic(AK8963 *sensor, ak8963_vector_t *magnetic) {
    int16_t raw_x = 0;
    int16_t raw_y = 0;
    int16_t raw_z = 0;
    if (!ak8963_read_three_shorts(sensor, AK8963_HXL, &raw_x, &raw_y, &raw_z)) {
        return false;
    }

    uint8_t st2 = 0;
    if (!ak8963_read_u8(sensor, AK8963_ST2, &st2)) {
        return false;
    }

    magnetic->x = raw_x;
    magnetic->y = raw_y;
    magnetic->z = raw_z;

    magnetic->x *= sensor->adjustment.x;
    magnetic->y *= sensor->adjustment.y;
    magnetic->z *= sensor->adjustment.z;

    magnetic->x *= sensor->so;
    magnetic->y *= sensor->so;
    magnetic->z *= sensor->so;

    magnetic->x -= sensor->offset.x;
    magnetic->y -= sensor->offset.y;
    magnetic->z -= sensor->offset.z;

    magnetic->x *= sensor->scale.x;
    magnetic->y *= sensor->scale.y;
    magnetic->z *= sensor->scale.z;

    return true;
}

bool ak8963_set_offset(AK8963 *sensor, ak8963_vector_t offset) {
    sensor->offset = offset;
    return true;
}

bool ak8963_set_scale(AK8963 *sensor, ak8963_vector_t scale) {
    sensor->scale = scale;
    return true;
}

bool ak8963_calibrate(AK8963 *sensor, int count, int delay_ms, ak8963_vector_t *offset, ak8963_vector_t *scale) {
    sensor->offset.x = 0.0f;
    sensor->offset.y = 0.0f;
    sensor->offset.z = 0.0f;
    sensor->scale.x = 1.0f;
    sensor->scale.y = 1.0f;
    sensor->scale.z = 1.0f;

    ak8963_vector_t reading;
    if (!ak8963_get_magnetic(sensor, &reading)) {
        return false;
    }

    float minx = reading.x;
    float maxx = reading.x;
    float miny = reading.y;
    float maxy = reading.y;
    float minz = reading.z;
    float maxz = reading.z;

    while (count > 0) {
        sleep_ms(delay_ms);
        if (!ak8963_get_magnetic(sensor, &reading)) {
            return false;
        }

        if (reading.x < minx) minx = reading.x;
        if (reading.x > maxx) maxx = reading.x;
        if (reading.y < miny) miny = reading.y;
        if (reading.y > maxy) maxy = reading.y;
        if (reading.z < minz) minz = reading.z;
        if (reading.z > maxz) maxz = reading.z;

        count--;
        printf("%d\n", count);
    }

    sensor->offset.x = (maxx + minx) / 2.0f;
    sensor->offset.y = (maxy + miny) / 2.0f;
    sensor->offset.z = (maxz + minz) / 2.0f;

    float avg_delta_x = (maxx - minx) / 2.0f;
    float avg_delta_y = (maxy - miny) / 2.0f;
    float avg_delta_z = (maxz - minz) / 2.0f;
    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0f;

    sensor->scale.x = avg_delta / avg_delta_x;
    sensor->scale.y = avg_delta / avg_delta_y;
    sensor->scale.z = avg_delta / avg_delta_z;

    if (offset != NULL) {
        *offset = sensor->offset;
    }
    if (scale != NULL) {
        *scale = sensor->scale;
    }

    return true;
}