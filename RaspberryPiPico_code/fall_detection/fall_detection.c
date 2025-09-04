#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "ff.h"
#include "sd_card.h"
#include "Neo6M_UBXParser.h"
#include "tusb.h"
#include <math.h>
#include "mpu6050.h"

#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define i2c_dev i2c0
#define i2c_dev_SDA 20
#define i2c_dev_SCL 21
#define mpu_addr 0x68
#define imu_polling 100 // Hz

#define accel_fall_thres 0.4f // gs
#define velD_fall_thres 1.0f // m/s
#define fall_time_thres_us 1 * 1e6
#define accel_landed_thres 0.9f // gs
#define velD_landed_thres 0.05 // m/s
#define landed_time_thres_us 1 * 1e6

#define control_log_rate 25 // Hz

// GPS Data Structs -------------------------------------------------------------------------------
struct posllhData posllh;
struct velnedData velned;
struct statusData status;

volatile float accel_data[3];
volatile float gyro_data[3];
volatile float temp_data;

FATFS fs;
FIL file_log;
const char* hdr = "TOW,velD,accel,falling,landed\r\n";
//const char* hdr = "accX,accY,accZ,gyX,gyY,gyZ\r\n";
uint64_t last_log, last_sync;

uint64_t started_falling, falling_ts_candidate, landed_ts_candidate;
bool is_falling, has_landed;

bool repeating_imu_cb(__unused struct repeating_timer *t) {
    mpu6050_read(accel_data, gyro_data, &temp_data);
    return true;
}

static void log_line(FIL *f, unsigned long timestamp, float v1, float v2, bool falling, bool landed) {
    char line[96];
    int len = snprintf(line, sizeof line, "%lu,%f,%f,%d,%d\r\n",
                       (unsigned long)timestamp, v1, v2, falling,landed);
    UINT bw;
    f_write(f, line, (UINT)len, &bw);

    printf("Saved: %s\n", line);
}


int main(void) {
    stdio_init_all();
    while(!tud_cdc_connected()) tight_loop_contents();
    sleep_ms(200);

    printf("Initializing uart...\n");
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    printf("Initializing mpu6050...\n");
    mpu6050_setdevparams(i2c_dev, mpu_addr);
    i2c_init(i2c_dev, 100 * 1000);
    gpio_set_function(i2c_dev_SDA, GPIO_FUNC_I2C);
    gpio_set_function(i2c_dev_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_dev_SDA);
    gpio_pull_up(i2c_dev_SCL);

    printf("Waking mpu6050...\n");
    mpu6050_wake(true);
    mpu6050_ping();
    printf("Configuring mpu6050...\n");
    mpu6050_configure(3, 3);

    printf("Starting IMU Callback timer...\n");
    struct repeating_timer imu_timer;
    add_repeating_timer_us((int)(1e6 / imu_polling), repeating_imu_cb, NULL, &imu_timer);

    sleep_ms(1000);

    while (!posllhChanged() || !status.gpsFixOk) {
        while(uart_is_readable(UART_ID)) { 
            addByte(uart_getc(UART_ID)); 
        }
        printf("[posllh: %d, fix: %d, fix type: %d] Waiting for GPS fix..\n", posllhChanged(), status.gpsFixOk, status.gpsFix);
        if (statusChanged()) {status = getSTATUS();}
    }
    printf("GPS Fix good!\n");

    if (!sd_init_driver()) { while (1) tight_loop_contents(); }
    if (f_mount(&fs, "0:", 1) != FR_OK) { while (1) tight_loop_contents(); }

    FRESULT fr = f_open(&file_log, "log.csv", FA_WRITE | FA_OPEN_ALWAYS);
    if (fr != FR_OK) { while (1) tight_loop_contents(); }
    UINT bw; 
    f_write(&file_log, hdr, (UINT)strlen(hdr), &bw);
    f_sync(&file_log);

    last_log = get_absolute_time();
    last_sync = get_absolute_time();

    float amag;
    static float velD;
    is_falling = false;
    has_landed = false;

    while (true) {
        while(uart_is_readable(UART_ID)) {
            addByte(uart_getc(UART_ID));
        }

        if (velnedChanged()) {
            velned = getVELNED();
            velD = velned.velD;
        }

        amag = sqrt(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);

        bool accel_freefall = amag <= accel_fall_thres;
        bool velD_freefall = velD >= velD_fall_thres;

        bool accel_landed = amag >= accel_landed_thres;
        bool velD_landed = velD <= velD_landed_thres;

        if (!is_falling){
            if (accel_freefall || velD_freefall) {
                if (falling_ts_candidate == 0) falling_ts_candidate = get_absolute_time();
                else if (absolute_time_diff_us(falling_ts_candidate, get_absolute_time()) > fall_time_thres_us) {
                    is_falling = true;
                }
            }
            else {
                falling_ts_candidate = 0;
            }
            
        }
        else {
            if (accel_landed || velD_landed) {
                if (landed_ts_candidate == 0) landed_ts_candidate = get_absolute_time();
                else if (absolute_time_diff_us(landed_ts_candidate, get_absolute_time()) > landed_time_thres_us) {
                    has_landed = true;
                    //f_close(&file_log);
                    //f_mount(NULL, "0:", 0);
                }
            }
            else {
                landed_ts_candidate = 0;
            }
        }

        if (absolute_time_diff_us(last_sync, get_absolute_time()) > (int)(1e6/0.1)) {
            f_sync(&file_log);
            last_sync = get_absolute_time();
        }
        if (absolute_time_diff_us(last_log, get_absolute_time()) > (int)(1e6/control_log_rate)) {
            log_line(&file_log, get_absolute_time(), velned.velD, amag, is_falling, has_landed);
            //log_line(&file_log, get_absolute_time(), accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2]);
            last_log = get_absolute_time();
        }
    }
}