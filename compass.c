#include "global.h"

bool compass_init(Compass *compass, i2c_inst_t *i2c, float declination_degrees) {
    compass->filtered_magx = 0.0f;
    compass->filtered_magy = 0.0f;
    compass->declination_degrees = declination_degrees;
    return mpu9250_init(&compass->sensor, i2c);
}

bool compass_calibrate(Compass *compass, ak8963_vector_t *offset, ak8963_vector_t *scale) {
    return ak8963_calibrate(&compass->sensor.ak8963, 256, 200, offset, scale);
}
bool compass_set_offset(Compass *compass, ak8963_vector_t offset) {
    return ak8963_set_offset(&compass->sensor.ak8963, offset);
}

bool compass_set_scale(Compass *compass, ak8963_vector_t scale) {
    return ak8963_set_scale(&compass->sensor.ak8963, scale);
}

bool compass_apply_calibration(Compass *compass, ak8963_vector_t offset, ak8963_vector_t scale) {
    if (!compass_set_offset(compass, offset)) return false;
    if (!compass_set_scale(compass, scale)) return false;
    return true;
}

bool compass_get_magnetic(Compass *compass, ak8963_vector_t *magnetic) {
    return mpu9250_get_magnetic(&compass->sensor, magnetic);
}

float compass_low_pass_filter(float previous, float value) {
    return 0.85f * previous + 0.15f * value;
}

bool compass_read_heading(Compass *compass, float *heading_degrees, float *heading_plus_declination_degrees) {
    ak8963_vector_t magnetic;
    if (!mpu9250_get_magnetic(&compass->sensor, &magnetic)) {
        return false;
    }

    compass->filtered_magx = compass_low_pass_filter(compass->filtered_magx, magnetic.x);
    compass->filtered_magy = compass_low_pass_filter(compass->filtered_magy, magnetic.y);

    float heading = atan2f(compass->filtered_magx, compass->filtered_magy) * (180.0f / (float)M_PI);
    float with_declination = heading + compass->declination_degrees;

    if (with_declination < 0.0f) {
        heading += 360.0f;
        with_declination += 360.0f;
    }

    if (heading_degrees != NULL) {
        *heading_degrees = heading;
    }
    if (heading_plus_declination_degrees != NULL) {
        *heading_plus_declination_degrees = with_declination;
    }
    return true;
}