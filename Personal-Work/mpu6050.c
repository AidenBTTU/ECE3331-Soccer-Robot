#include "global.h"

#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_WHO_AM_I 0x75

#define MPU6050_ACCEL_SO_2G 16384.0f
#define MPU6050_ACCEL_SO_4G 8192.0f
#define MPU6050_ACCEL_SO_8G 4096.0f
#define MPU6050_ACCEL_SO_16G 2048.0f

#define MPU6050_GYRO_SO_250DPS 131.0f
#define MPU6050_GYRO_SO_500DPS 62.5f
#define MPU6050_GYRO_SO_1000DPS 32.8f
#define MPU6050_GYRO_SO_2000DPS 16.4f

#define MPU6050_TEMP_SO 333.87f
#define MPU6050_TEMP_OFFSET 21.0f

static bool mpu6050_read_big_endian_short(MPU6050 *sensor, uint8_t reg, int16_t *value) {
    int rc = i2c_write_blocking(sensor->i2c, sensor->address, &reg, 1, true);
    if (rc != 1) {
        return false;
    }

    uint8_t buf[2];
    rc = i2c_read_blocking(sensor->i2c, sensor->address, buf, 2, false);
    if (rc != 2) {
        return false;
    }

    *value = (int16_t)((buf[0] << 8) | buf[1]);
    return true;
}

static bool mpu6050_read_three_big_endian_shorts(MPU6050 *sensor, uint8_t reg, int16_t *x, int16_t *y, int16_t *z) {
    int rc = i2c_write_blocking(sensor->i2c, sensor->address, &reg, 1, true);
    if (rc != 1) {
        return false;
    }

    uint8_t buf[6];
    rc = i2c_read_blocking(sensor->i2c, sensor->address, buf, 6, false);
    if (rc != 6) {
        return false;
    }

    *x = (int16_t)((buf[0] << 8) | buf[1]);
    *y = (int16_t)((buf[2] << 8) | buf[3]);
    *z = (int16_t)((buf[4] << 8) | buf[5]);
    return true;
}

static float mpu6050_accel_so_from_fs(uint8_t value) {
    switch (value) {
        case MPU6050_ACCEL_FS_SEL_2G: return MPU6050_ACCEL_SO_2G;
        case MPU6050_ACCEL_FS_SEL_4G: return MPU6050_ACCEL_SO_4G;
        case MPU6050_ACCEL_FS_SEL_8G: return MPU6050_ACCEL_SO_8G;
        case MPU6050_ACCEL_FS_SEL_16G: return MPU6050_ACCEL_SO_16G;
        default: return MPU6050_ACCEL_SO_2G;
    }
}

static float mpu6050_gyro_so_from_fs(uint8_t value) {
    switch (value) {
        case MPU6050_GYRO_FS_SEL_250DPS: return MPU6050_GYRO_SO_250DPS;
        case MPU6050_GYRO_FS_SEL_500DPS: return MPU6050_GYRO_SO_500DPS;
        case MPU6050_GYRO_FS_SEL_1000DPS: return MPU6050_GYRO_SO_1000DPS;
        case MPU6050_GYRO_FS_SEL_2000DPS: return MPU6050_GYRO_SO_2000DPS;
        default: return MPU6050_GYRO_SO_250DPS;
    }
}

bool mpu6050_read_u8(MPU6050 *sensor, uint8_t reg, uint8_t *value) {
    int rc = i2c_write_blocking(sensor->i2c, sensor->address, &reg, 1, true);
    if (rc != 1) {
        return false;
    }
    rc = i2c_read_blocking(sensor->i2c, sensor->address, value, 1, false);
    return rc == 1;
}

bool mpu6050_write_u8(MPU6050 *sensor, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_blocking(sensor->i2c, sensor->address, buf, 2, false) == 2;
}

bool mpu6050_init(
    MPU6050 *sensor,
    i2c_inst_t *i2c,
    uint8_t address,
    uint8_t accel_fs,
    uint8_t gyro_fs,
    float accel_sf,
    float gyro_sf,
    mpu6050_vector_t gyro_offset
) {
    sensor->i2c = i2c;
    sensor->address = address;
    sensor->accel_sf = accel_sf;
    sensor->gyro_sf = gyro_sf;
    sensor->gyro_offset = gyro_offset;

    uint8_t whoami = 0;
    if (!mpu6050_get_whoami(sensor, &whoami) || (whoami != 0x70 && whoami != 0x71 && whoami != 0x68)) {
        return false;
    }

    if (!mpu6050_write_u8(sensor, MPU6050_REG_ACCEL_CONFIG, accel_fs)) {
        return false;
    }
    if (!mpu6050_write_u8(sensor, MPU6050_REG_GYRO_CONFIG, gyro_fs)) {
        return false;
    }

    sensor->accel_so = mpu6050_accel_so_from_fs(accel_fs);
    sensor->gyro_so = mpu6050_gyro_so_from_fs(gyro_fs);
    return true;
}

bool mpu6050_get_acceleration(MPU6050 *sensor, mpu6050_vector_t *acceleration) {
    int16_t x = 0, y = 0, z = 0;
    if (!mpu6050_read_three_big_endian_shorts(sensor, MPU6050_REG_ACCEL_XOUT_H, &x, &y, &z)) {
        return false;
    }

    acceleration->x = ((float)x / sensor->accel_so) * sensor->accel_sf;
    acceleration->y = ((float)y / sensor->accel_so) * sensor->accel_sf;
    acceleration->z = ((float)z / sensor->accel_so) * sensor->accel_sf;
    return true;
}

bool mpu6050_get_gyro(MPU6050 *sensor, mpu6050_vector_t *gyro) {
    int16_t x = 0, y = 0, z = 0;
    if (!mpu6050_read_three_big_endian_shorts(sensor, MPU6050_REG_GYRO_XOUT_H, &x, &y, &z)) {
        return false;
    }

    gyro->x = ((float)x / sensor->gyro_so) * sensor->gyro_sf - sensor->gyro_offset.x;
    gyro->y = ((float)y / sensor->gyro_so) * sensor->gyro_sf - sensor->gyro_offset.y;
    gyro->z = ((float)z / sensor->gyro_so) * sensor->gyro_sf - sensor->gyro_offset.z;
    return true;
}

bool mpu6050_get_temperature(MPU6050 *sensor, float *temperature_c) {
    int16_t temp = 0;
    if (!mpu6050_read_big_endian_short(sensor, MPU6050_REG_TEMP_OUT_H, &temp)) {
        return false;
    }
    *temperature_c = ((temp - MPU6050_TEMP_OFFSET) / MPU6050_TEMP_SO) + MPU6050_TEMP_OFFSET;
    return true;
}

bool mpu6050_get_whoami(MPU6050 *sensor, uint8_t *whoami) {
    return mpu6050_read_u8(sensor, MPU6050_REG_WHO_AM_I, whoami);
}

bool mpu6050_calibrate_gyro(MPU6050 *sensor, int count, int delay_ms, mpu6050_vector_t *gyro_offset) {
    sensor->gyro_offset.x = 0.0f;
    sensor->gyro_offset.y = 0.0f;
    sensor->gyro_offset.z = 0.0f;
    float n = (float)count;
    float ox = 0.0f, oy = 0.0f, oz = 0.0f;

    while (count > 0) {
        sleep_ms(delay_ms);
        mpu6050_vector_t gyro;
        if (!mpu6050_get_gyro(sensor, &gyro)) {
            return false;
        }
        ox += gyro.x;
        oy += gyro.y;
        oz += gyro.z;
        count--;
    }

    sensor->gyro_offset.x = ox / n;
    sensor->gyro_offset.y = oy / n;
    sensor->gyro_offset.z = oz / n;
    if (gyro_offset != NULL) {
        *gyro_offset = sensor->gyro_offset;
    }
    return true;
}