#ifndef AK8963_H
#define AK8963_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

#define AK8963_DEFAULT_ADDRESS 0x0C

#define AK8963_MODE_POWER_DOWN 0x00
#define AK8963_MODE_SINGLE_MEASURE 0x01
#define AK8963_MODE_CONTINUOUS_MEASURE_1 0x02
#define AK8963_MODE_EXTERNAL_TRIGGER_MEASURE 0x04
#define AK8963_MODE_CONTINUOUS_MEASURE_2 0x06
#define AK8963_MODE_SELF_TEST 0x08
#define AK8963_MODE_FUSE_ROM_ACCESS 0x0F

#define AK8963_OUTPUT_14_BIT 0x00
#define AK8963_OUTPUT_16_BIT 0x10

typedef struct {
    float x;
    float y;
    float z;
} ak8963_vector_t;

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
    uint8_t mode;
    uint8_t output;
    float so;
    ak8963_vector_t adjustment;
    ak8963_vector_t offset;
    ak8963_vector_t scale;
} AK8963;

bool ak8963_init(
    AK8963 *sensor,
    i2c_inst_t *i2c,
    uint8_t address,
    uint8_t mode,
    uint8_t output,
    ak8963_vector_t offset,
    ak8963_vector_t scale
);
bool ak8963_get_whoami(AK8963 *sensor, uint8_t *whoami);
bool ak8963_get_adjustment(const AK8963 *sensor, ak8963_vector_t *adjustment);
bool ak8963_get_magnetic(AK8963 *sensor, ak8963_vector_t *magnetic);
bool ak8963_set_offset(AK8963 *sensor, ak8963_vector_t offset);
bool ak8963_set_scale(AK8963 *sensor, ak8963_vector_t scale);
bool ak8963_calibrate(AK8963 *sensor, int count, int delay_ms, ak8963_vector_t *offset, ak8963_vector_t *scale);

#endif