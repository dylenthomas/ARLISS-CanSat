#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

// Provide the Pico I2C instance and 7-bit device address (0x68 if AD0=0, 0x69 if AD0=1)
void mpu6050_setdevparams(i2c_inst_t *dev, int addr);

// Read WHO_AM_I (0x75). Prints result; blocks forever on I2C error.
void mpu6050_ping(void);

// rst_to_default = true does a device reset (0x80), false just clears sleep (0x00)
void mpu6050_wake(bool rst_to_default);

// gyro_range, accel_range in {0,1,2,3} -> ±250/500/1000/2000 dps and ±2/4/8/16 g
void mpu6050_configure(int gyro_range, int accel_range);

// Optional power helpers
void mpu6050_setlowpwr(void);
void mpu6050_setfullpwr(void);

// Reads all 14 data bytes starting at 0x3B. accel in g’s, gyro in deg/s, temp in °C.
void mpu6050_read(volatile float accel[3], volatile float gyro[3], volatile float *temp);

#endif
