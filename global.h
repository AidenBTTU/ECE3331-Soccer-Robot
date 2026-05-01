#ifndef GLOBAL_H
#define GLOBAL_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"

#include "ak8963.h"
#include "camera.h"
#include "colorsensor.h"
#include "compass.h"
#include "encoder.h"
#include "hcsr04.h"
#include "motornoencoder.h"
#include "mpu6050.h"
#include "mpu9250.h"
#include "servo.h"

void sleepcheck(int x);
float normalize_heading(float relative);

#endif
