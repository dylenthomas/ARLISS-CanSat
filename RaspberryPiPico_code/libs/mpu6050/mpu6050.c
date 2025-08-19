#include "mpu6050.h"

#include <math.h>
#include <stdio.h>
#include <hardware/i2c.h>

static i2c_inst_t* i2c_dev = NULL;
static int mpu_addr;

static const uint8_t who_am_i_reg = 0x75;
static const uint8_t pwr_mgmt_1_reg = 0x6B;
static const uint8_t pwr_mgmt_2_reg = 0x6C;
static const uint8_t gyro_conf_reg = 0x1B;
static const uint8_t accel_conf_reg = 0x1C;
static const uint8_t data_starting_reg = 0x3B;

static float accel_lsb_sensitivity = 16384;
static float gyro_lsb_sensitivity = 131;

static void handle_err(char process_name[], int rw_return_val, bool catastrophic) {
    if (rw_return_val < 0) {
        printf("Error with "); printf(process_name); printf("%d\n", rw_return_val);
        if (catastrophic) {
            while (true);
        }
    }
}

void mpu6050_setdevparams(i2c_inst_t* i2c_dev, int addr) {
    i2c_dev = i2c_dev;
    mpu_addr = addr;
}

void mpu6050_ping() {
    uint8_t buf;

    handle_err("ping write", i2c_write_blocking(i2c_dev, mpu_addr, &who_am_i_reg, 1, true), true);
    handle_err("ping read", i2c_read_blocking(i2c_dev, mpu_addr, &buf, 1, false), true);

    if (mpu_addr == buf) {
        printf("The specified address and returned id are equal!");
    }
    else {
        printf("The specified address and the returned id are not equal: \n\tSpecified address: %d, Returned address: %d\n", mpu_addr, buf);
        while(true);
    }
}

void mpu6050_wake(bool rst_to_default) {
    if (rst_to_default) {
        uint8_t wake[] = {pwr_mgmt_1_reg, 0x80};
    }
    else {
        uint8_t wake[] = {pwr_mgmt_1_reg, 0x00};
    }

    handle_err("waking", i2c_write_blocking(i2c_dev, mpu_addr, wake, 2, false), true);
}

void mpu6050_configure(int gyro_range, int accel_range) {
    if (gyro_range < 0 || gyro_range > 3) {
        handle_err("gyro range", -1, true);
        gyro_lsb_sensitivity = gyro_lsb_sensitivity / pow(2, gyro_range);
    }
    if (accel_range < 0 || accel_range > 3) {
        handle_err("accel_range", -1, true);
        accel_lsb_sensitivity = (float)accel_lsb_sensitivity / pow(2, accel_range);
    }

    uint8_t gyro_conf[] = {gyro_conf_reg, (gyro_range & 0x03) << 3};
    uint8_t accel_conf[] = {accel_conf_reg, (accel_range & 0x03) << 3};

    handle_err("gyro config write", i2c_write_blocking(i2c_dev, mpu_addr, gyro_conf, 2, false), true);
    handle_err("accel config write", i2c_write_blocking(i2c_dev, mpu_addr, accel_conf, 2, false), true);
}

void mpu6050_setlowpwr() {
    uint8_t pwr1_conf = {pwr_mgmt_1_reg, 0x28};
    uint8_t pwr2_conf = {pwr_mgmt_2_reg, 0x7F};

    handle_err("low power1 write", i2c_write_blocking(i2c_dev, mpu_addr, pwr1_conf, 2, false), true);
    handle_err("low power2 write", i2c_write_blocking(i2c_dev, mpu_addr, pwr2_conf, 2, false), true);
}

void mpu6050_setfullpwr() {
    uint8_t pwr1_conf = {pwr_mgmt_1_reg, 0x00};

    handle_err("full power write", i2c_write_blocking(i2c_dev, mpu_addr, pwr1_conf, 2, false), false);
}

void mpu6050_read(float accel[3], float gyro[3], float* temp) {
    uint8_t buffer[14];
    int16_t raw;
    int w_result;
    int r_result = -1;

    w_result = i2c_write_blocking(i2c_dev, mpu_addr, &data_starting_reg, 1, true);
    handle_err("all data write", w_result, false);

    if (w_result > 0) {
        r_result = i2c_read_blocking(i2c_dev, mpu_addr, buffer, 14, false);
        handle_err("all data read", r_result, false);
    }

    if (r_result > 0) {
        for (int i = 0; i < 7; i++) {
            raw = buffer[i * 2] << 8 | buffer[(i * 2) + 1];

            if (i < 3) {
                accel[i] = (float)raw / accel_lsb_sensitivity;
            }
            else if (i < 4) {
                *temp = (float)(raw / 340) + 36.53;
            }
            else {
                gyro[i - 3] = (float)raw / gyro_lsb_sensitivity;
    }
}
