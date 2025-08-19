#ifndef MPU6050_H

#define MPU6050_H

static void handle_err(char process_name[], int rw_return_val);

void mpu6050_setdevparams(i2c_inst_t* i2c_dev, int addr);
void mpu6050_ping();
void mpu6050_wake(bool rst_to_defualt);
/*
 * gyro_range is 0, 1, 2, 3
 * accel_range is 0, 1, 2, 3
 *
 * refer to mpu6050 register datasheet
 */
void mpu6050_configure(int gyro_range, int accel_range);
/*
 * set to acceleration and gyro only low power mode
 * both will be sampled by the device at 5 Hz, so incase they do not pwoer back on the imu is still usable
 */
void mpu6050_setlowpwr();
void mpu6050_setfullpwr();

void mpu6050_read(float accel[3], float gyro[3], float* temp);

#endif
