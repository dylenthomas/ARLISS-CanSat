#include "mpu6050.h"

#include <math.h>
#include <stdio.h>

static i2c_inst_t *s_i2c = NULL;
static int s_addr = 0x68;

// Registers
static const uint8_t REG_WHO_AM_I   = 0x75;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_PWR_MGMT_2 = 0x6C;
static const uint8_t REG_GYRO_CFG   = 0x1B;
static const uint8_t REG_ACCEL_CFG  = 0x1C;
static const uint8_t REG_DATA_START = 0x3B;

// Sensitivities at range index 0 (±2 g, ±250 dps). Divide by 2^range_idx.
static float s_accel_lsb_sens = 16384.0f; // LSB per g
static float s_gyro_lsb_sens  = 131.0f;   // LSB per deg/s

static void handle_err(const char *what, int ret, bool catastrophic){
    if (ret < 0){
        printf("I2C error in %s: %d\n", what, ret);
        if (catastrophic){ while (true){ /* halt */ } }
    }
}

void mpu6050_setdevparams(i2c_inst_t *dev, int addr){
    s_i2c = dev;
    s_addr = addr;
}

void mpu6050_ping(void){
    uint8_t id = 0;
    int w = i2c_write_blocking(s_i2c, s_addr, (uint8_t*)&(uint8_t){REG_WHO_AM_I}, 1, true);
    handle_err("WHO_AM_I write", w, true);
    int r = i2c_read_blocking(s_i2c, s_addr, &id, 1, false);
    handle_err("WHO_AM_I read", r, true);
    // Typical ID is 0x68 on MPU-6050
    printf("MPU6050 WHO_AM_I: 0x%02X (addr=0x%02X)\n", id, s_addr);
}

void mpu6050_wake(bool rst_to_default){
    uint8_t wake[2] = { REG_PWR_MGMT_1, rst_to_default ? 0x80 : 0x00 };
    int w = i2c_write_blocking(s_i2c, s_addr, wake, 2, false);
    handle_err("wake", w, true);
    if (rst_to_default){
        // After reset, clear sleep
        sleep_ms(50);
        uint8_t clr[2] = { REG_PWR_MGMT_1, 0x00 };
        w = i2c_write_blocking(s_i2c, s_addr, clr, 2, false);
        handle_err("wake-clear", w, true);
    }
}

void mpu6050_configure(int gyro_range, int accel_range){
    if (gyro_range < 0 || gyro_range > 3) handle_err("gyro_range", -1, true);
    if (accel_range < 0 || accel_range > 3) handle_err("accel_range", -1, true);

    // Update LSB sensitivities based on selected range
    s_gyro_lsb_sens  = 131.0f    / (float)(1 << gyro_range);
    s_accel_lsb_sens = 16384.0f  / (float)(1 << accel_range);

    uint8_t g[2] = { REG_GYRO_CFG,  (uint8_t)((gyro_range  & 0x03) << 3) };
    uint8_t a[2] = { REG_ACCEL_CFG, (uint8_t)((accel_range & 0x03) << 3) };

    int w = i2c_write_blocking(s_i2c, s_addr, g, 2, false);
    handle_err("gyro cfg", w, true);
    w = i2c_write_blocking(s_i2c, s_addr, a, 2, false);
    handle_err("accel cfg", w, true);
}

void mpu6050_setlowpwr(void){
    uint8_t p1[2] = { REG_PWR_MGMT_1, 0x28 }; // cycle + internal 8 MHz (example)
    uint8_t p2[2] = { REG_PWR_MGMT_2, 0x7F }; // disable axes in LP mode (example)
    int w = i2c_write_blocking(s_i2c, s_addr, p1, 2, false);
    handle_err("low power pwr1", w, true);
    w = i2c_write_blocking(s_i2c, s_addr, p2, 2, false);
    handle_err("low power pwr2", w, true);
}

void mpu6050_setfullpwr(void){
    uint8_t p1[2] = { REG_PWR_MGMT_1, 0x00 };
    int w = i2c_write_blocking(s_i2c, s_addr, p1, 2, false);
    handle_err("full power", w, false);
}

void mpu6050_read(float accel[3], float gyro[3], float *temp){
    uint8_t buf[14];

    // Set register pointer to ACCEL_XOUT_H
    int w = i2c_write_blocking(s_i2c, s_addr, (uint8_t*)&(uint8_t){REG_DATA_START}, 1, true);
    handle_err("data addr write", w, false);

    int r = i2c_read_blocking(s_i2c, s_addr, buf, 14, false);
    handle_err("data read", r, false);
    if (r <= 0) return;

    // ACCEL XYZ
    for (int i = 0; i < 3; i++){
        int16_t raw = (int16_t)((buf[2*i] << 8) | buf[2*i + 1]);
        accel[i] = (float)raw / s_accel_lsb_sens;
    }

    // TEMP
    int16_t traw = (int16_t)((buf[6] << 8) | buf[7]);
    *temp = ((float)traw) / 340.0f + 36.53f;

    // GYRO XYZ
    for (int i = 0; i < 3; i++){
        int off = 8 + 2*i;
        int16_t raw = (int16_t)((buf[off] << 8) | buf[off + 1]);
        gyro[i] = (float)raw / s_gyro_lsb_sens;
    }
}
