#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "tusb.h"

#include "Neo6M_UBXParser.h"
#include "mpu6050.h"
#include "navigation.h"
#include "kalman_stuff.h"
#include "PWMmotorDriver.h"
#include "ff.h"
#include "sd_card.h"

FATFS fs;
FIL file;

// TARGET INFORMATION -------------------------------------------------------------------------------------------

struct TARGET {
    float deg;
    float arcmin;
    float arcsec;

    float deg_dec;
};
struct TARGET target_lat; // NORTH/SOUTH
struct TARGET target_lon;  // EAST/WEST

void setTarget() {
    target_lat.deg = 33.782032;
    target_lat.arcmin = 0.0;
    target_lat.arcsec = 0.0;
    target_lat.deg_dec = target_lat.deg + (float)(target_lat.arcmin * 1/60) + (float)(target_lat.arcsec * 1/3600);

    target_lon.deg = -84.407196;
    target_lon.arcmin = 0.0;
    target_lon.arcsec = 0.0;
    target_lon.deg_dec = target_lon.deg + (float)(target_lon.arcmin * 1/60) + (float)(target_lon.arcsec * 1/3600);
}
// --------------------------------------------------------------------------------------------------------------

#ifndef PI
#define PI 3.14159265359
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD PI / 180.0f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 180.0f / PI
#endif

#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define i2c_dev i2c0
#define i2c_dev_SDA 20
#define i2c_dev_SCL 21
#define mpu_addr 0x68
#define imu_polling 100 // Hz

#define Vforward 1.0 //0.95
#define KaP 0.05
#define KaD 0.005
#define stopping_dist 5 // m

#define m1_IN1 15
#define m1_IN2 13
#define m2_IN1 6
#define m2_IN2 7
#define min_DC 0.4
#define max_DC 1.0

#define servo_pin 0

#define fall_time_threshold 3

#define control_log_rate 2 // Hz

#define accel_fall_thres 0.5
#define fall_time_thres_us 5 * 1e5 // us
#define acc_landed_a 0.9
#define acc_landed_b 1.1
#define landed_time_thres_us 5 * 1e6 // us

static uint servo_slice;
static uint servo_chan;
const int servo_ticks = 21;

// Motor Pin Structs ------------------------------------------------------------------------------
struct motor Motor1;
struct motor Motor2;

// GPS Data Structs -------------------------------------------------------------------------------
struct posllhData posllh;
struct velnedData velned;
struct statusData status;

// IMU Vars ---------------------------------------------------------------------------------------
unsigned long last_imu_time;
float accel_data[3];
float gyro_data[3];
float temp_data;

// Fall tracking Vars ---------------------------------------------------------------------------------------
uint64_t started_falling, falling_ts_candidate, landed_ts_candidate;
bool is_falling;

// Controller Vars --------------------------------------------------------------------------------
float u_r;
float u_l;
float last_alpha;
float alpha;
float Tp;
float Td;
float T;
float u_r_internal;
float u_l_internal;
float target_heading;
uint64_t last_contr_call;
float est_heading;
uint64_t last_log;
bool was_paused;

const char *hdr = "TOW,control_dt,heading,lat,lon,alt\r\n";

// Controller Funcs -------------------------------------------------------------------------------
static float shortestRotation(float x) {
	return((x) > (PI) ? (x - 2 * PI): (x) < (-PI) ? (x + 2 * PI): (x));
}

static float constrainFloat(float x, float min, float max) {
	return ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)));
}

static float max(float a, float b) {
    return((a) < (b) ? (b): (a));
}

static float absFloat(float x) {
    return((x) < (0.0) ? (x * -1.0) : (x));
}

static float boundTo2Pi(float x) {
    return((x) > (2 * PI) ? (x - 2 * PI): (x) < (0.0f) ? (x + 2 * PI): (x));
}

float calcT(float current_heading) {
    float dt, de;

    dt = absolute_time_diff_us(last_contr_call, get_absolute_time());
    dt = (float)(dt * 1e-6);
   
    alpha = shortestRotation(target_heading - current_heading);
    de = alpha - last_alpha;

    Tp = KaP * alpha;
    Td = constrainFloat(KaD * de / dt, -2.0, 2.0);

    last_alpha = alpha;
    last_contr_call = get_absolute_time();

    return constrainFloat(Tp + Td, -0.5, 0.5);
}

void control(float current_heading) {
    T = calcT(current_heading);
    u_l_internal = Vforward + T;
    u_r_internal = Vforward - T;

    u_l = constrainFloat(absFloat(u_l_internal), min_DC, max_DC);
    u_r = constrainFloat(absFloat(u_r_internal), min_DC, max_DC);

    setMotorPWM(Motor1, u_l, (u_l_internal) > (0.0) ? (FORWARD): (REVERSE));
    setMotorPWM(Motor2, u_r, (u_r_internal) > (0.0) ? (FORWARD): (REVERSE));
}
// ------------------------------------------------------------------------------------------------

bool repeating_imu_cb(__unused struct repeating_timer *t) {
    mpu6050_read(accel_data, gyro_data, &temp_data);
    last_imu_time = get_absolute_time();

    return true;
}

static void log_line(FIL *f, unsigned long TOW, float dt_log, float heading, float lat, float lon, float alt) {
    char line[96];
    int len = snprintf(line, sizeof line, "%lu,%.2f,%.2f,%3.7f,%3.7f,%.2f\r\n",
                       (unsigned long)TOW, dt_log, heading, lat, lon, alt);
    UINT bw;
    f_write(f, line, (UINT)len, &bw);
    f_sync(f);
}

void config_servo_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(servo_slice, 20000 - 1);
    pwm_set_clkdiv(servo_slice, 125);
    pwm_set_enabled(servo_slice, true);

    if (pin % 2 == 0) {
        servo_chan = PWM_CHAN_A;
    }
    else {
        servo_chan = PWM_CHAN_B;
    }
}

void set_servo(int deg) {
    int flip_val = (int)(800 + 7.78 * deg);
    pwm_set_chan_level(servo_slice, servo_chan, flip_val);
}

void release_servo() {
    int tick_speed = 500;

    for (int i = 0; i < servo_ticks; i++) {
        int deg = 10 * i;
        set_servo(180 - deg);
        sleep_ms(tick_speed);
    }
}

bool landed() {
    float amag = sqrt(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
    
    bool accel_freefall = amag < accel_fall_thres;
    bool accel_landed = acc_landed_a > amag && amag < acc_landed_b;

    if (!is_falling){
        if (accel_freefall) {
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
        if (accel_landed) {
            if (landed_ts_candidate == 0) landed_ts_candidate = get_absolute_time();
            else if (absolute_time_diff_us(landed_ts_candidate, get_absolute_time()) > landed_time_thres_us) {
                return true;
            }
        }
        else {
            landed_ts_candidate = 0;
        }
    }

    return false;
}

int main()
{
// PRE LAUNCH CODE ----------------------------------------------------------------------------------------------
    stdio_init_all();
    while(!tud_cdc_connected()) tight_loop_contents();
    sleep_ms(2000);

    printf("Hello! Starting configuration and pre flight checks!\n");

    setTarget();
    printf("Target location set!\n");

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

    printf("Configuring motor PWM...\n");
    addPins(&Motor1, m1_IN1, m1_IN2);
    addPins(&Motor2, m2_IN1, m2_IN2);

    printf("Mounting SD Card...\n");
    if (!sd_init_driver()) { while (1) tight_loop_contents(); }
    if (f_mount(&fs, "0:", 1) != FR_OK) { while (1) tight_loop_contents(); }

    printf("Opening log file...\n");
    FRESULT fr = f_open(&file, "CtrlLog.csv", FA_WRITE | FA_OPEN_ALWAYS);
    if (fr != FR_OK) { while (1) tight_loop_contents(); }

    if (f_size(&file) == 0) {
        UINT bw; 
        f_write(&file, hdr, (UINT)strlen(hdr), &bw);
        f_sync(&file);
    }
    else {
        printf("Could not create header of log file!\n");
        printf("File seems to have data in it already!\n");
        while (1) tight_loop_contents();
    }
    printf("Control log created.\n");

    printf("Everything is configured!\n");
    printf("Checking GPS status now...\n");

    sleep_ms(5000);

    while (!posllhChanged() || !status.gpsFixOk) {
        while(uart_is_readable(UART_ID)) { 
            addByte(uart_getc(UART_ID)); 
        }
        printf("[posllh: %d, fix: %d, fix type: %d] Waiting for GPS fix..\n", posllhChanged(), status.gpsFixOk, status.gpsFix);
        if (statusChanged()) {status = getSTATUS();}
    }
    printf("GPS Fix good!\n");

// END PRE LAUNCH -----------------------------------------------------------------------------------------------
    printf("I am ready for launch!\n");
// LOAD INTO ROCKET ---------------------------------------------------------------------------------------------
    falling_ts_candidate = 0;
    landed_ts_candidate = 0;
    //while (!landed()) tight_loop_contents();
    //release_servo();

    posllh = getPOSLLH();
    set_ref_pos(posllh.lon, posllh.lat, posllh.height);

    log_line(&file, posllh.iTOW, 0.0, -1.0, posllh.lat, posllh.lon, posllh.hMSL);

    float destination[3];
    computeNEDpos(target_lon.deg_dec, target_lat.deg_dec, posllh.height, destination);

    float current_pos[3];
    float dN, dE;
    float dest_heading;
    float velN, velE, velD;
    uint64_t last_velocity;
    float dt_vel;

// NAVIGATION TO TARGET -----------------------------------------------------------------------------------------
    while(true) {
        if (was_paused) {
            unpausePWM();
            was_paused = false;
        }

        while(uart_is_readable(UART_ID)) {
            addByte(uart_getc(UART_ID));
        }

        if (posllhChanged()) {
            posllh = getPOSLLH();
            computeNEDpos(posllh.lon, posllh.lat, posllh.height, current_pos);

            z_X[0] = current_pos[0];
            z_Y[0] = current_pos[1];
            kalmanUpdate_X();
            kalmanUpdate_Y();

            float lat1 = posllh.lat * DEG_TO_RAD;
            float lon1 = posllh.lon * DEG_TO_RAD;
            float lat2 = target_lat.deg_dec * DEG_TO_RAD;
            float lon2 = target_lon.deg_dec * DEG_TO_RAD;
            float y = sin(lon2 - lon1) * cos(lat2);
            float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
            dest_heading = boundTo2Pi(atan2(y, x));

            dN = destination[0] - xhat_X[0];
            dE = destination[1] - xhat_Y[0];
            //dest_heading = boundTo2Pi(atan2(dE, dN)) * RAD_TO_DEG;

            //printf("Destination: [%f, %f, %f], Position: [%f, %f, %f], heading to destination: %f, current heading: %f\n", destination[0], destination[1], destination[2], xhat_X[0], xhat_Y[0], current_pos[2], dest_heading, est_heading);
        }

        if (absolute_time_diff_us(last_imu_time, get_absolute_time()) > (int)(1e6/imu_polling)) {           
            float dt = (float)(absolute_time_diff_us(last_imu_time, get_absolute_time()) * 1e-6);
            kalmanPredict_heading(gyro_data[2], dt);
        }

        if (velnedChanged()) {
            velned = getVELNED();
            dt_vel = (float)(absolute_time_diff_us(last_velocity, get_absolute_time()) * 1e-6);
            //printf("Delta Lat = %f\n", lat_dest - posllh.lat);
            //printf("Meas X = %f, Est X = %f, Est velX bias = %f, velX = %f\n", current_pos[0], xhat_X[0], xhat_X[1], velned.velN);

            kalmanPredict_X(velned.velN, dt_vel);
            kalmanPredict_Y(velned.velE, dt_vel);
            last_velocity = get_absolute_time();

            z_heading[0] = velned.heading;
            kalmanUpdate_heading();
        }

        est_heading = boundTo2Pi(xhat_heading[0]);
        control(est_heading);

        //if (sqrt(dN * dN + dE * dE) < stopping_dist) {
        //    pausePWM();
        //    printf("Reached destination!\n");
        //    log_line(&file, posllh.iTOW, 0.0, -1.0, posllh.lat, posllh.lon, posllh.hMSL);
        //    f_close(&file);
        //    f_mount(NULL, "0:", 0);
        //    while (true) tight_loop_contents();
        //}

        if (absolute_time_diff_us(last_log, get_absolute_time()) > (int)(1e6/control_log_rate)) {           
            unsigned long timestamp = posllh.iTOW;
            float log_dt = (float)(absolute_time_diff_us(last_log, get_absolute_time()) * 1e-6);
            float heading = est_heading * RAD_TO_DEG;

            // wait until the first line of data is written to start logging
            if (f_size(&file) > strlen(hdr)) {
                log_line(&file, timestamp, log_dt, heading, posllh.lat, posllh.lon, posllh.hMSL);
            }

            last_log = get_absolute_time();
        }

        while (!status.gpsFixOk) {
            while(uart_is_readable(UART_ID)) {
                addByte(uart_getc(UART_ID));
            }

            if (!was_paused) {
                pausePWM();
                was_paused = true;
            }
        }
    }
}
