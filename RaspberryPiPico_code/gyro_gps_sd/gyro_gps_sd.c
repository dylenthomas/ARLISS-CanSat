#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "tusb.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "hardware/uart.h"
#include "Neo6M_UBXParser.h"
#include "btstack.h"
#include "att_db_util.h"

#include "mpu6050.h"

#include "pico/time.h"
#include "ff.h"
#include "sd_card.h"

// --- gps params ---
static bool gps_enabled = true;

// --- I2C bus/pin selection (your wiring: GP6/GP7 -> I2C1) ---
#define I2C_PORT    i2c0
#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21

// --- GPS UART (NEO-6M) ---
#define GPS_UART      uart1
#define GPS_BAUD      9600
#define GPS_RX_PIN    5      // GPS TX -> GP5

// === GPS data storage ===
static double last_lat = 0.0;
static double last_lon = 0.0;
static float last_alt = 0.0f;
static uint32_t last_gps_time = 0;
static bool gps_fix_valid = false;
static int gps_satellites = 0;

// ===== BLE state =====
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static btstack_packet_callback_registration_t hci_event_cb;
static uint16_t notify_val_handle, notify_cccd_handle, write_val_handle;
static int notify_enabled = 0;

#define MSG_MAX 96
static char pending_msg[MSG_MAX];
static uint16_t pending_len = 0;

FATFS fs;
FIL file;

static inline void queue_note(const char *s){
    if (!s || !notify_enabled || connection_handle == HCI_CON_HANDLE_INVALID) return;
    if (pending_len) return;
    size_t n = strlen(s); if (n > MSG_MAX) n = MSG_MAX;
    memcpy(pending_msg, s, n); pending_len = (uint16_t)n;
    att_server_request_can_send_now_event(connection_handle);
}

static const uint8_t UUID_SVC_19B1_0000[16] = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x00,0x00,0xb1,0x19};
static const uint8_t UUID_CHR_19B1_0001[16] = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x01,0x00,0xb1,0x19};
static const uint8_t UUID_CHR_19B1_0002[16] = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x02,0x00,0xb1,0x19};

static uint8_t adv_data[]  = {
    0x02, 0x01, 0x06,
    0x11, 0x07,
    0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x00,0x00,0xb1,0x19
};
static uint8_t scan_resp[] = { 0x08, 0x09, 'P','i','c','o','B','L','E' };

static inline void set_led(bool on){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0);
}

// Command flags set via BLE writes
static volatile int zero_heading = 0;
static volatile int recal_bias   = 0;

// Forward decl for heading timer
static void restart_head_timer(void);
static void restart_gps_timer(void);

static void log_line(FIL *f, uint32_t t_ms, float a1, float a2, float a3, float g1, float g2, float g3);

// ===== ATT callbacks =====
static uint16_t att_read_cb(hci_con_handle_t ch, uint16_t h, uint16_t off, uint8_t *buf, uint16_t sz){
    (void)ch; (void)h; (void)off; if (buf && sz){ buf[0]=0; return 1; } return 0;
}
static int att_write_cb(hci_con_handle_t ch, uint16_t h, uint16_t mode, uint16_t off, uint8_t *data, uint16_t len){
    (void)mode; (void)off;
    if (h == notify_cccd_handle){
        if (len >= 2){
            uint16_t v = (uint16_t)(data[0] | (data[1]<<8));
            notify_enabled = (v & 1) ? 1 : 0;
            if (notify_enabled){
                queue_note("<READY>\n");
                restart_head_timer();
                restart_gps_timer();
            } else {
                pending_len = 0;
            }
        }
        return 0;
    }
    if (h == write_val_handle){
        char cmd[64]; if (len>63) len=63; memcpy(cmd,data,len); cmd[len]=0;
        for (int i=0;i<len;i++) cmd[i]=(char)tolower((unsigned char)cmd[i]);
        if (!strcmp(cmd,"led on")) { set_led(true);  queue_note("OK\n"); }
        else if (!strcmp(cmd,"led off")){ set_led(false); queue_note("OK\n"); }
        else if (!strcmp(cmd,"led?"))   { queue_note("LED=?\n"); }
        else if (!strcmp(cmd,"zero"))   { zero_heading = 1; queue_note("OK\n"); }
        else if (!strcmp(cmd,"recal"))  { recal_bias   = 1; queue_note("OK\n"); }
        else if (!strcmp(cmd,"gps"))    { 
            // Send current GPS status
            char gps_status[MSG_MAX];
            if (gps_fix_valid) {
                snprintf(gps_status, sizeof(gps_status), "GPS=%.6f,%.6f,%.1f,%d\n", 
                        last_lat, last_lon, last_alt, gps_satellites);
            } else {
                snprintf(gps_status, sizeof(gps_status), "GPS=NO_FIX\n");
            }
            queue_note(gps_status);
        }
        else                            { queue_note("OK\n"); }
        return 0;
    }
    return 0;
}

// ===== HCI events =====
static void packet_handler(uint8_t type, uint16_t ch, uint8_t *packet, uint16_t size){
    (void)ch; (void)size;
    if (type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                gap_advertisements_set_params(0x30,0x30,0,0,(bd_addr_t){0},0x07,0x00);
                gap_advertisements_set_data(sizeof(adv_data), adv_data);
                gap_scan_response_set_data(sizeof(scan_resp), scan_resp);
                gap_advertisements_enable(1);
                printf("Advertising\n");
            }
            break;
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE){
                connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                gap_request_connection_parameter_update(connection_handle,24,40,0,400);
                printf("Connected (handle %04x)\n", connection_handle);
            }
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("Disconnected\n");
            connection_handle = HCI_CON_HANDLE_INVALID;
            notify_enabled = 0; pending_len = 0;
            gap_advertisements_enable(1);
            break;
        case ATT_EVENT_CAN_SEND_NOW:
            if (notify_enabled && pending_len && connection_handle != HCI_CON_HANDLE_INVALID){
                att_server_notify(connection_handle, notify_val_handle, (const uint8_t*)pending_msg, pending_len);
                pending_len = 0;
            }
            break;
        default: break;
    }
}

// ===== GATT build =====
static void build_gatt(void){
    att_db_util_init();

    att_db_util_add_service_uuid16(0x1800);
    att_db_util_add_characteristic_uuid16(
        0x2A00, ATT_PROPERTY_READ, ATT_SECURITY_NONE, ATT_SECURITY_NONE,
        (uint8_t*)"PicoBLE", 7
    );

    uint8_t svc_changed[4] = {0,0,0,0};
    att_db_util_add_service_uuid16(0x1801);
    att_db_util_add_characteristic_uuid16(
        0x2A05, ATT_PROPERTY_INDICATE, ATT_SECURITY_NONE, ATT_SECURITY_NONE,
        svc_changed, sizeof(svc_changed)
    );

    att_db_util_add_service_uuid128(UUID_SVC_19B1_0000);

    uint8_t init_val[1] = {0x00};
    notify_val_handle = att_db_util_add_characteristic_uuid128(
        UUID_CHR_19B1_0001,
        ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY,
        ATT_SECURITY_NONE, ATT_SECURITY_NONE,
        init_val, sizeof(init_val)
    );
    notify_cccd_handle = (uint16_t)(notify_val_handle + 1);

    
    write_val_handle = att_db_util_add_characteristic_uuid128(
        UUID_CHR_19B1_0002,
        ATT_PROPERTY_WRITE | ATT_PROPERTY_DYNAMIC,
        ATT_SECURITY_NONE, ATT_SECURITY_NONE,
        NULL, 0
    );
}

// ======== IMU detect, sampling, yaw integration, logging ========
static bool   imu_present = false;
static uint8_t imu_whoami = 0x00;
static int    imu_addr    = 0x68;

static btstack_timer_source_t imu_timer;   
static btstack_timer_source_t imu_log_timer; 
static btstack_timer_source_t head_timer;
static btstack_timer_source_t gps_send_timer;

static float g_accel[3] = {0};
static float g_gyro[3]  = {0};
static float g_temp_c   = 0.0f;

static float heading_deg = 0.0f;
static float gyro_bias_z_dps = 0.0f;
static int   calib_samples = 0;
static uint32_t last_ms = 0;

static inline float wrap_deg(float x){
    while (x >= 360.0f) x -= 360.0f;
    while (x <    0.0f) x += 360.0f;
    return x;
}

static bool try_whoami_addr(int addr, uint8_t *out_id){
    uint8_t reg = 0x75;
    int w = i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    if (w < 0) return false;
    int r = i2c_read_blocking(I2C_PORT, addr, out_id, 1, false);
    return r > 0;
}

static void imu_timer_cb(btstack_timer_source_t *ts){
    (void)ts;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    float dt = (last_ms == 0) ? 0.0f : (float)(now_ms - last_ms) * 0.001f;
    last_ms = now_ms;
    if (dt <= 0.0f) dt = 0.01f;

    if (imu_present){
        mpu6050_read(g_accel, g_gyro, &g_temp_c);

        if (zero_heading){ heading_deg = 0.0f; zero_heading = 0; }
        if (recal_bias){ gyro_bias_z_dps = 0.0f; calib_samples = 0; recal_bias = 0; }

        if (calib_samples >= 0 && calib_samples < 400){
            gyro_bias_z_dps += g_gyro[2];
            calib_samples++;
            if (calib_samples == 400) gyro_bias_z_dps /= 400.0f;
        } else {
            float rate = g_gyro[2] - gyro_bias_z_dps;
            heading_deg = wrap_deg(heading_deg + rate * dt);
        }
    }

    btstack_run_loop_set_timer(&imu_timer, 10);    // ~100 Hz
    btstack_run_loop_add_timer(&imu_timer);
}

static void imu_log_timer_cb(btstack_timer_source_t *ts){
    (void)ts;
    if (imu_present){
        //printf("IMU ok (WHO_AM_I=0x%02X @0x%02X) | GyroZ=%6.2f dps | Temp=%.2f C | Heading=%.1f deg (bias=%.3f)\n",
               //imu_whoami, imu_addr, g_gyro[2], g_temp_c, heading_deg, gyro_bias_z_dps);
        uint32_t t_ms = to_ms_since_boot(get_absolute_time());

        float a1 = g_accel[0];   // heading (deg)
        float a2 = g_accel[1];
        float a3 = g_accel[2];
        float g1 = g_gyro[0];    // gyro (deg/s)
        float g2 = g_gyro[1];
        float g3 = g_gyro[2];

        log_line(&file, t_ms, a1, a2, a3, g1, g2, g3);
        f_sync(&file);           
    } else {
        printf("IMU not found; advertising BLE; streaming HEAD=NA\n");
    }

    if (gps_fix_valid) {
        printf("GPS: Lat=%.6f, Lon=%.6f, Alt=%.1fm, Sats=%d\n",
               last_lat, last_lon, last_alt, gps_satellites);
    } else {
        printf("GPS: No fix\n");
    }

    btstack_run_loop_set_timer(&imu_log_timer, 100);
    btstack_run_loop_add_timer(&imu_log_timer);
}


static void head_timer_cb(btstack_timer_source_t *ts){
    (void)ts;
    if (notify_enabled && connection_handle != HCI_CON_HANDLE_INVALID){
        if (imu_present && calib_samples >= 400){
            float rate = g_gyro[2] - gyro_bias_z_dps;
            char line[MSG_MAX];
            snprintf(line, sizeof(line), "HEAD=%.1f,rate=%.2f\n", heading_deg, rate);
            queue_note(line);
        } else if (imu_present){
            queue_note("HEAD=0.0,rate=0.00\n"); // during bias settle
        } else {
            queue_note("HEAD=NA\n");
        }
    }
    
    restart_head_timer();
}

static void restart_head_timer(void){
    if (!notify_enabled || connection_handle == HCI_CON_HANDLE_INVALID) return;
    btstack_run_loop_set_timer(&head_timer, 200);
    btstack_run_loop_add_timer(&head_timer);
}

// GPS data transmission timer (every 2 seconds when GPS has fix)
static void gps_send_timer_cb(btstack_timer_source_t *ts){
    (void)ts;
    if (notify_enabled && connection_handle != HCI_CON_HANDLE_INVALID && gps_fix_valid){
        char line[MSG_MAX];
        snprintf(line, sizeof(line), "GPS=%.6f,%.6f,%.1f,%d\n", 
                last_lat, last_lon, last_alt, gps_satellites);
        queue_note(line);
    }
    restart_gps_timer();
}

static void restart_gps_timer(void){
    if (!notify_enabled || connection_handle == HCI_CON_HANDLE_INVALID) return;
    btstack_run_loop_set_timer(&gps_send_timer, 2000);
    btstack_run_loop_add_timer(&gps_send_timer);
}

static btstack_timer_source_t gps_poll_timer;

// GPS data processing - called when new GPS data is parsed
void gps_data_callback(double lat, double lon, float alt, int sats, bool valid) {
    last_lat = lat;
    last_lon = lon;
    last_alt = alt;
    gps_satellites = sats;
    gps_fix_valid = valid;
    last_gps_time = to_ms_since_boot(get_absolute_time());
}

static void gps_poll_cb(btstack_timer_source_t *ts){
    (void)ts;
    
    int bytes_read = 0;
    while (uart_is_readable(GPS_UART) && bytes_read < 32) {
        uint8_t byte = uart_getc(GPS_UART);
        addByte(byte);
        //printf("%c", byte); // Echo raw GPS data to console
        
        bytes_read++;
    }
    
    btstack_run_loop_set_timer(&gps_poll_timer, 10);   // ~100 Hz poll
    btstack_run_loop_add_timer(&gps_poll_timer);
}

//for sd card logging
static void log_line(FIL *f, uint32_t t_ms, float a1, float a2, float a3, float g1, float g2, float g3) {
    char line[96];
    int len = snprintf(line, sizeof line, "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                       (unsigned long)t_ms, a1, a2, a3, g1, g2, g3);
    UINT bw;
    f_write(f, line, (UINT)len, &bw);
}

static void initsdcard() {
    if (!sd_init_driver()) { while (1) tight_loop_contents(); }
    if (f_mount(&fs, "0:", 1) != FR_OK) { while (1) tight_loop_contents(); }

    FRESULT fr = f_open(&file, "imulog.csv", FA_WRITE | FA_OPEN_ALWAYS);
    if (fr != FR_OK) { while (1) tight_loop_contents(); }

    if (f_size(&file) == 0) {
        const char *hdr = "timestamp_ms,accel[g] ax,ay,az,gyro[deg/s], gx, gy, gz\r\n";
        UINT bw; f_write(&file, hdr, (UINT)strlen(hdr), &bw);
    }
}


// ===== main =====
int main(void){
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(5000); 

    printf("init sd card\n");
    initsdcard();
    printf("Starting PicoBLE + MPU6050 + GPS initialization\n");

    // Initialize CYW43
    if (cyw43_arch_init() != 0){
        printf("CYW43 init failed\n");
        return 1;
    }
    set_led(false);
    printf("CYW43 initialized\n");

    // --- GPS UART init: UART1 @ 9600, RX=GP5, TX=GP4 ---
    gpio_set_function(GPS_RX_PIN, UART_FUNCSEL_NUM(GPS_UART, 5));
    gpio_set_function(GPS_RX_PIN - 1, UART_FUNCSEL_NUM(GPS_UART, 4));
    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);   // GP5 as UART1 RX
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    printf("GPS UART initialized\n");

    
    // --- I2C init (I2C0 @ 400k on GP20/GP21) ---
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C initialized\n");

    // --- Probe IMU safely (no fatal errors) ---
    if (try_whoami_addr(0x68, &imu_whoami) && (imu_whoami == 0x68 || imu_whoami == 0x70)){
        imu_present = true; imu_addr = 0x68;
    } else if (try_whoami_addr(0x69, &imu_whoami) && (imu_whoami == 0x68 || imu_whoami == 0x70)){
        imu_present = true; imu_addr = 0x69;
    } else {
        imu_present = false;
    }

    if (imu_present){
        printf("MPU6050 detected: WHO_AM_I=0x%02X @0x%02X\n", imu_whoami, imu_addr);
        mpu6050_setdevparams(I2C_PORT, imu_addr);
        mpu6050_wake(false);
        mpu6050_configure(3, 3);  // ±250 dps, ±2g
    } else {
        printf("MPU6050 not detected; skipping IMU init (BLE will still run)\n");
    }

    // Initialize BTStack 
    printf("Initializing BTStack...\n");
    l2cap_init();
    sm_init();
    build_gatt();
    printf("GATT built - notify=0x%04x write=0x%04x cccd=0x%04x\n", notify_val_handle, write_val_handle, notify_cccd_handle);
    
    att_server_init(att_db_util_get_address(), &att_read_cb, &att_write_cb);
    hci_event_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_event_cb);
    att_server_register_packet_handler(&packet_handler);

    printf("Setting up timers...\n");
    
    // Initialize timer sources
    last_ms = to_ms_since_boot(get_absolute_time());
    
    // Set up timer handlers
    btstack_run_loop_set_timer_handler(&imu_timer, imu_timer_cb);
    btstack_run_loop_set_timer_handler(&imu_log_timer, imu_log_timer_cb);
    btstack_run_loop_set_timer_handler(&head_timer, head_timer_cb);
    btstack_run_loop_set_timer_handler(&gps_poll_timer, gps_poll_cb);
    btstack_run_loop_set_timer_handler(&gps_send_timer, gps_send_timer_cb);
    
    // Start timers
    btstack_run_loop_set_timer(&imu_timer, 10);
    btstack_run_loop_add_timer(&imu_timer);
    
    btstack_run_loop_set_timer(&imu_log_timer, 100);
    btstack_run_loop_add_timer(&imu_log_timer);
    
    if (gps_enabled){
        btstack_run_loop_set_timer(&gps_poll_timer, 100);  
        btstack_run_loop_add_timer(&gps_poll_timer);
        printf("GPS polling timer started\n");
    }
    
    
    printf("Starting BLE and main loop...\n");
    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
    return 0;
    
}