#include "mpu9250.h"

#include "pico/stdlib.h"

#define MPU9250_REG_INT_PIN_CFG 0x37
#define MPU9250_REG_USER_CTRL 0x6A
#define MPU9250_I2C_BYPASS_MASK 0x02
#define MPU9250_I2C_BYPASS_EN 0x02
#define MPU9250_I2C_MST_EN 0x20

bool mpu9250_init(MPU9250 *sensor, i2c_inst_t *i2c) {
    mpu6050_vector_t gyro_offset = {0.0f, 0.0f, 0.0f};
    bool mpu_ok = mpu6050_init(
        &sensor->mpu6050,
        i2c,
        0x68,
        MPU6050_ACCEL_FS_SEL_2G,
        MPU6050_GYRO_FS_SEL_250DPS,
        MPU6050_SF_M_S2,
        MPU6050_SF_RAD_S,
        gyro_offset);
    if (!mpu_ok) {
        mpu_ok = mpu6050_init(
            &sensor->mpu6050,
            i2c,
            0x69,
            MPU6050_ACCEL_FS_SEL_2G,
            MPU6050_GYRO_FS_SEL_250DPS,
            MPU6050_SF_M_S2,
            MPU6050_SF_RAD_S,
            gyro_offset);
    }
    if (!mpu_ok) {
        return false;
    }

    // Ensure the MPU I2C master is off so BYPASS mode can expose the AK8963 on the bus.
    uint8_t user_ctrl = 0;
    if (!mpu6050_read_u8(&sensor->mpu6050, MPU9250_REG_USER_CTRL, &user_ctrl)) {
        return false;
    }
    user_ctrl &= (uint8_t)~MPU9250_I2C_MST_EN;
    if (!mpu6050_write_u8(&sensor->mpu6050, MPU9250_REG_USER_CTRL, user_ctrl)) {
        return false;
    }

    uint8_t reg = 0;
    if (!mpu6050_read_u8(&sensor->mpu6050, MPU9250_REG_INT_PIN_CFG, &reg)) {
        return false;
    }
    reg &= (uint8_t)~MPU9250_I2C_BYPASS_MASK;
    reg |= MPU9250_I2C_BYPASS_EN;
    if (!mpu6050_write_u8(&sensor->mpu6050, MPU9250_REG_INT_PIN_CFG, reg)) {
        return false;
    }
    sleep_ms(10);

    ak8963_vector_t offset = {0.0f, 0.0f, 0.0f};
    ak8963_vector_t scale = {1.0f, 1.0f, 1.0f};
    return ak8963_init(
        &sensor->ak8963,
        i2c,
        AK8963_DEFAULT_ADDRESS,
        AK8963_MODE_CONTINUOUS_MEASURE_1,
        AK8963_OUTPUT_16_BIT,
        offset,
        scale);
}

bool mpu9250_get_acceleration(MPU9250 *sensor, mpu6050_vector_t *acceleration) {
    return mpu6050_get_acceleration(&sensor->mpu6050, acceleration);
}

bool mpu9250_get_gyro(MPU9250 *sensor, mpu6050_vector_t *gyro) {
    return mpu6050_get_gyro(&sensor->mpu6050, gyro);
}

bool mpu9250_get_temperature(MPU9250 *sensor, float *temperature_c) {
    return mpu6050_get_temperature(&sensor->mpu6050, temperature_c);
}

bool mpu9250_get_magnetic(MPU9250 *sensor, ak8963_vector_t *magnetic) {
    return ak8963_get_magnetic(&sensor->ak8963, magnetic);
}

bool mpu9250_get_whoami(MPU9250 *sensor, uint8_t *whoami) {
    return mpu6050_get_whoami(&sensor->mpu6050, whoami);
}