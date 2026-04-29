#ifndef COMPASS_H
#define COMPASS_H

#include <stdbool.h>

#include "mpu9250.h"

typedef struct {
    MPU9250 sensor;
    float filtered_magx;
    float filtered_magy;
    float declination_degrees;
} Compass;

bool compass_init(Compass *compass, i2c_inst_t *i2c, float declination_degrees);
bool compass_calibrate(Compass *compass, ak8963_vector_t *offset, ak8963_vector_t *scale);
bool compass_get_magnetic(Compass *compass, ak8963_vector_t *magnetic);
float compass_low_pass_filter(float previous, float value);
bool compass_read_heading(Compass *compass, float *heading_degrees, float *heading_plus_declination_degrees);

#endif