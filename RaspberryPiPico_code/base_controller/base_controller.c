#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "tusb.h"

#include "Neo6M_UBXParser.h"
#include "mpu6050.h"
#include "navigation.h"
#include "kalman_stuff.h"
#include "PWMmotorDriver.h"

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
float velD;
float altitude, last_altitude, calc_velD;
uint64_t last_alt_time;
uint64_t started_falling;

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
static uint64_t last_contr_call;
float dt, de;

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
    float dt;
    float u;

    mpu6050_read(accel_data, gyro_data, &temp_data);

    dt = (float)(absolute_time_diff_us(last_imu_time, get_absolute_time()) * 1e-6);
    last_imu_time = get_absolute_time();
    u = gyro_data[2];
    kalmanPredict_heading(u, dt);

    return true;
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

bool onGround() {
    if (velnedChanged() && posllhChanged()) {
        velned = getVELNED();
        velD = velned.velD;

        posllh = getPOSLLH();
        altitude = posllh.hMSL;

        float dt = (float)(absolute_time_diff_us(last_alt_time, get_absolute_time()) * 1e-6);
        dt = max(dt, 1e-6); // prevent divide by zero
        calc_velD = (float)((altitude - last_altitude) / (dt));
            
        last_altitude = altitude;
        last_alt_time = get_absolute_time();
    }

    if (round(velD) > 0.0 && round(calc_velD) < 0.0 && started_falling == 0) {
        started_falling = get_absolute_time();
    }
    else {
        started_falling = 0;
    }

    uint64_t time_falling = absolute_time_diff_us(started_falling, get_absolute_time());

    if (started_falling != 0 && time_falling >= fall_time_threshold * 1e6) {
        return true;
    }
    else {
        return false;
    }
}

int main()
{
    stdio_init_all();
    while(!tud_cdc_connected());
    sleep_ms(2000);

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

    struct repeating_timer timer;
    add_repeating_timer_us((int)(1e6 / imu_polling), repeating_imu_cb, NULL, &timer);

    printf("Configuring motor PWM...\n");
    addPins(&Motor1, m1_IN1, m1_IN2);
    addPins(&Motor2, m2_IN1, m2_IN2);

    printf("Everything is configured!\n");

    sleep_ms(5000);

    while (!posllhChanged() || !status.gpsFixOk) {
        while(uart_is_readable(UART_ID)) { 
            addByte(uart_getc(UART_ID)); 
        }
        printf("[posllh: %d, fix: %d, fix type: %d] Waiting for GPS fix..\n", posllhChanged(), status.gpsFixOk, status.gpsFix);
        if (statusChanged()) {status = getSTATUS();}
    }
    printf("GPS Fix good!\n");

    last_alt_time = get_absolute_time();
    while (!onGround()) {
        while (uart_is_readable(UART_ID)) {
            addByte(uart_getc(UART_ID));
        }
    }

    for (int i = 0; i < servo_ticks; i++) {
        int deg = 10 * i;
        set_servo(180 - deg);
        sleep_ms(500);
    }

    posllh = getPOSLLH();
    set_ref_pos(posllh.lon, posllh.lat, posllh.height);

    float target_lat[] = {33.782032};
    float lat_dest = 0.0f;
    float target_lon[] = {-84.407196};
    float lon_dest = 0.0f;

    float destination[3];

    for (int i = 0; i < sizeof(target_lat)/sizeof(target_lat[0]); i ++) {
		lat_dest += (float)(target_lat[i] * pow(60, -i));
	}
	for (int i = 0; i < sizeof(target_lon)/sizeof(target_lon[0]); i ++) {
		lon_dest += (float)(target_lon[i] * pow(60, -i));
	}

    computeNEDpos(lon_dest, lat_dest, posllh.height, destination);

    float current_pos[3];
    float dN, dE;
    float dest_heading;
    float velN, velE, velD;
    uint64_t last_velocity;
    float dt_vel;
    float est_heading;

    // Main loop
    while(true) {
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

            dN = destination[0] - xhat_X[0];
            dE = destination[1] - xhat_Y[0];
            dest_heading = boundTo2Pi(atan2(dE, dN)) * RAD_TO_DEG;

            //printf("Destination: [%f, %f, %f], Position: [%f, %f, %f], heading to destination: %f, current heading: %f\n", destination[0], destination[1], destination[2], xhat_X[0], xhat_Y[0], current_pos[2], dest_heading, est_heading);
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
        //    while (true);
        //}

        while (!status.gpsFixOk) {
            printf("Lost GPS fix!\n");
        }
    }
}
