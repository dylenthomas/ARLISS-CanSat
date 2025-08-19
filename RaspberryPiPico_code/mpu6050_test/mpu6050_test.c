#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "tusb.h"
#include "hardware/i2c.h"

#define i2c_dev_SDA 6
#define i2c_dev_SCL 7
#define i2c_dev i2c1
#define mpu_addr 0x68
#define accel_lsb_scaler 2048.0f
#define gyro_lsb_scaler 16.4f

void handle_err(char reg_type[], int read_write_status) {
    if (read_write_status < 0) {
        printf("Error with "); printf(reg_type); printf("%d", read_write_status); 
        while (true);
    }
}

static void mpu6050_ping() {
    printf("Checking MPU id...\n");
    uint8_t buf = 0;
    const uint8_t who_am_i = 0x75;

    handle_err("ping write", i2c_write_blocking(i2c_dev, mpu_addr, &who_am_i, 1, true));
    handle_err("ping read", i2c_read_blocking(i2c_dev, mpu_addr, &buf, 1, false));

    if (mpu_addr == buf) {
        printf("The specified address and returned id are equal!");
    }
    else {
        printf("The specified address and the returned id are not equal: \n\tSpecified address: %d, Returned address: %d\n", mpu_addr, buf);
        while(true);
    }
} 

static void mpu6050_wake() {
    uint8_t wake[] = {0x6B, 0x00};
    handle_err("waking", i2c_write_blocking(i2c_dev, mpu_addr, wake, 2, false));
    printf("Successfully woke MPU.");
}

static void mpu6050_configure() {
    printf("Configuring gyro...\n");

    uint8_t gyro_conf[] = { 0x1B, 0x18 }; // set to +/- 2000 deg/s
    handle_err("gyro config write", i2c_write_blocking(i2c_dev, mpu_addr, gyro_conf, 2, false));
    

    uint8_t accel_conf[] = { 0x1C, 0x18 }; // set to +/- 16 gs
    handle_err("accel config write", i2c_write_blocking(i2c_dev, mpu_addr, accel_conf, 2, false));
}

static void mpu6050_read_data(float accel[3], float gyro[3], float* temp) {
   uint8_t buffer[14];
   int16_t raw;

   uint8_t data_starting_reg = 0x3B;
   handle_err("asking for all data", i2c_write_blocking(i2c_dev, mpu_addr, &data_starting_reg, 1, true));
   handle_err("reading all data", i2c_read_blocking(i2c_dev, mpu_addr, buffer, 14, false));

   for (int i = 0; i < 7; i++) {
       raw = buffer[i * 2] << 8 | buffer[(i * 2) + 1];

       if (i < 3) {
           accel[i] = (float)raw / accel_lsb_scaler;
       }
       else if (i < 4) {
           *temp = (float)(raw / 340) + 36.53;        
       }
       else {
            gyro[i - 3] = (float)raw / gyro_lsb_scaler;
       }
   }
}

void toggle_led(bool led_on) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
}

void blink_led(int num_blinks) {
    int blink_time = 250;
    for (int i = 0; i < num_blinks; i++) {
        toggle_led(true);
        sleep_ms(blink_time);
        toggle_led(false);
        sleep_ms(blink_time);
    } 
}

int main()
{
    stdio_init_all();
    // wait for USB connection
    while (!tud_cdc_connected());

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    blink_led(3);

    i2c_init(i2c_dev, 100 * 1000); // 100kHz
    gpio_set_function(i2c_dev_SDA, GPIO_FUNC_I2C);
    gpio_set_function(i2c_dev_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_dev_SDA);
    gpio_pull_up(i2c_dev_SCL);

    mpu6050_wake();
    mpu6050_ping();
    mpu6050_configure();

    float accel_data[3];
    float gyro_data[3];
    float temp_data;

    while (true) {
        mpu6050_read_data(accel_data, gyro_data, &temp_data);

        printf("Acc.  X = %f gs,    Y = %f gs,    Z = %f gs\n", accel_data[0], accel_data[1], accel_data[2]);
        printf("Gyro. X = %f deg/s, Y = %f deg/s, Z = %f deg/s\n", gyro_data[0], gyro_data[1], gyro_data[2]);
        printf("Chip temp. = %fC\n", temp_data);
    }
    return 0;
}
