// BLE peripheral for Raspberry Pi Pico W - WASD LED Controller
// Real-time LED control via BLE: W=GP16, A=GP14, S=GP15, D=GP13
// Sends status updates at 10Hz when notifications are enabled

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "btstack_run_loop.h"
#include "att_db_util.h"

// GPIO Pin assignments
#define PIN_W 16
#define PIN_A 14
#define PIN_S 15
#define PIN_D 13

static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static btstack_packet_callback_registration_t hci_event_cb;

static uint16_t notify_val_handle = 0;
static uint16_t write_val_handle  = 0;
static uint16_t notify_cccd_handle = 0;
static int      notify_enabled = 0;

// LED states
static bool led_w = false;
static bool led_a = false;
static bool led_s = false;
static bool led_d = false;

// Simple single-message queue for notifications
#define MSG_MAX 128
static char pending_msg[MSG_MAX];
static uint16_t pending_len = 0;

static void queue_note(const char *s){
    if (!s) return;
    if (connection_handle == HCI_CON_HANDLE_INVALID || !notify_enabled) return;
    if (pending_len) return; // drop if previous hasn't been sent yet
    size_t n = strlen(s);
    if (n > MSG_MAX) n = MSG_MAX;
    memcpy(pending_msg, s, n);
    pending_len = (uint16_t)n;
    att_server_request_can_send_now_event(connection_handle);
}

static inline uint32_t now_ms(void){ return to_ms_since_boot(get_absolute_time()); }

// 10 Hz status streaming
static btstack_timer_source_t stream_timer;

static void restart_stream_timer(void){
    if (!notify_enabled || connection_handle == HCI_CON_HANDLE_INVALID) return;
    btstack_run_loop_set_timer(&stream_timer, 100);   // 10 Hz
    btstack_run_loop_add_timer(&stream_timer);
}

static void stream_cb(btstack_timer_source_t *ts){
    (void)ts;
    if (notify_enabled && connection_handle != HCI_CON_HANDLE_INVALID){
        char line[MSG_MAX];
        snprintf(line, sizeof(line), "W:%d A:%d S:%d D:%d\n", 
                 led_w ? 1 : 0, led_a ? 1 : 0, led_s ? 1 : 0, led_d ? 1 : 0);
        queue_note(line);
    }
    restart_stream_timer();
}

// UUIDs (little-endian bytes)
static const uint8_t UUID_SVC_19B1_0000[16] = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x00,0x00,0xb1,0x19};
static const uint8_t UUID_CHR_19B1_0001[16] = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x01,0x00,0xb1,0x19};
static const uint8_t UUID_CHR_19B1_0002[16] = {0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x02,0x00,0xb1,0x19};

// Advertise data
static uint8_t adv_data[] = {
    0x02, 0x01, 0x06,
    0x11, 0x07,
    0x14,0x12,0x8a,0x76,0x04,0xd1,0x6c,0x4f,0x7e,0x53,0xf2,0xe8,0x00,0x00,0xb1,0x19
};

static uint8_t scan_resp[] = {
    0x08, 0x09, 'P','i','c','o','B','L','E'
};

static void set_leds(bool w, bool a, bool s, bool d){
    led_w = w;
    led_a = a;
    led_s = s;
    led_d = d;
    
    gpio_put(PIN_W, w ? 1 : 0);
    gpio_put(PIN_A, a ? 1 : 0);
    gpio_put(PIN_S, s ? 1 : 0);
    gpio_put(PIN_D, d ? 1 : 0);
}

static void init_gpio(void){
    gpio_init(PIN_W);
    gpio_set_dir(PIN_W, GPIO_OUT);
    gpio_put(PIN_W, 0);
    
    gpio_init(PIN_A);
    gpio_set_dir(PIN_A, GPIO_OUT);
    gpio_put(PIN_A, 0);
    
    gpio_init(PIN_S);
    gpio_set_dir(PIN_S, GPIO_OUT);
    gpio_put(PIN_S, 0);
    
    gpio_init(PIN_D);
    gpio_set_dir(PIN_D, GPIO_OUT);
    gpio_put(PIN_D, 0);
}

// ATT callbacks
static uint16_t att_read_cb(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t offset,
                            uint8_t *buffer, uint16_t buffer_size){
    (void)con_handle; (void)att_handle; (void)offset;
    if (buffer && buffer_size){
        buffer[0] = 0x00;
        return 1;
    }
    return 0;
}

static int att_write_cb(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode,
                        uint16_t offset, uint8_t *data, uint16_t len){
    (void)con_handle; (void)transaction_mode; (void)offset;

    // Client toggled notifications?
    if (att_handle == notify_cccd_handle){
        int en = 0;
        if (len >= 2){
            uint16_t v = data[0] | (data[1] << 8);
            en = (v & 0x0001) ? 1 : 0;
        }
        notify_enabled = en;
        if (notify_enabled){
            queue_note("<READY>\n");
            restart_stream_timer();
        } else {
            pending_len = 0;
        }
        return 0;
    }

    // Command written to write characteristic?
    if (att_handle == write_val_handle){
        if (len == 0 || len > 120) return 0;

        char cmd[128];
        memcpy(cmd, data, len); 
        cmd[len] = '\0';

        // Parse WASD command format: "WASD:wxaxsxdx" where x is 0 or 1
        // Example: "WASD:1010" means W=on, A=off, S=on, D=off
        if (len >= 9 && strncmp(cmd, "WASD:", 5) == 0){
            bool w = (cmd[5] == '1');
            bool a = (cmd[6] == '1');
            bool s = (cmd[7] == '1');
            bool d = (cmd[8] == '1');
            set_leds(w, a, s, d);
            queue_note("OK\n");
        } else {
            queue_note("ERR\n");
        }
        return 0;
    }

    return 0;
}

// HCI / ATT event handler
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                gap_advertisements_set_params(0x30, 0x30, 0, 0, (bd_addr_t){0}, 0x07, 0x00);
                gap_advertisements_set_data(sizeof(adv_data), adv_data);
                gap_scan_response_set_data(sizeof(scan_resp), scan_resp);
                gap_advertisements_enable(1);
                printf("Advertising WASD Controller\n");
            }
            break;

        case HCI_EVENT_LE_META: {
            uint8_t sub = hci_event_le_meta_get_subevent_code(packet);
            if (sub == HCI_SUBEVENT_LE_CONNECTION_COMPLETE){
                connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                printf("Connected (handle %04x)\n", connection_handle);
                gap_request_connection_parameter_update(connection_handle, 24, 40, 0, 400);
                if (notify_enabled) restart_stream_timer();
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("Disconnected\n");
            connection_handle = HCI_CON_HANDLE_INVALID;
            notify_enabled = 0;
            pending_len = 0;
            set_leds(false, false, false, false);
            gap_advertisements_enable(1);
            break;

        case ATT_EVENT_CAN_SEND_NOW:
            if (notify_enabled && pending_len > 0 && connection_handle != HCI_CON_HANDLE_INVALID){
                att_server_notify(connection_handle, notify_val_handle,
                                  (const uint8_t*)pending_msg, pending_len);
                pending_len = 0;
            }
            break;

        default:
            break;
    }
}

static void build_gatt(void){
    att_db_util_init();

    // GAP: Device Name
    att_db_util_add_service_uuid16(0x1800);
    att_db_util_add_characteristic_uuid16(
        0x2A00, ATT_PROPERTY_READ,
        ATT_SECURITY_NONE, ATT_SECURITY_NONE,
        (uint8_t*)"PicoBLE", 7
    );

    // GATT: Service Changed
    uint8_t svc_changed_val[4] = {0x00,0x00,0x00,0x00};
    att_db_util_add_service_uuid16(0x1801);
    att_db_util_add_characteristic_uuid16(
        0x2A05, ATT_PROPERTY_INDICATE,
        ATT_SECURITY_NONE, ATT_SECURITY_NONE,
        svc_changed_val, sizeof(svc_changed_val)
    );

    // Custom service
    att_db_util_add_service_uuid128(UUID_SVC_19B1_0000);

    // Notify characteristic
    uint8_t init_notify_val[1] = {0x00};
    notify_val_handle = att_db_util_add_characteristic_uuid128(
        UUID_CHR_19B1_0001,
        ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY,
        ATT_SECURITY_NONE, ATT_SECURITY_NONE,
        init_notify_val, sizeof(init_notify_val)
    );
    notify_cccd_handle = (uint16_t)(notify_val_handle + 1);

    // Write characteristic
    write_val_handle = att_db_util_add_characteristic_uuid128(
        UUID_CHR_19B1_0002,
        ATT_PROPERTY_WRITE | ATT_PROPERTY_DYNAMIC,
        ATT_SECURITY_NONE, ATT_SECURITY_NONE,
        NULL, 0
    );
}

int main(void){
    stdio_init_all();
    for (int i = 0; i < 100; ++i) sleep_ms(10);
    printf("BLE: WASD LED Controller\n");
    printf("Pin Mapping: W=GP%d A=GP%d S=GP%d D=GP%d\n", PIN_W, PIN_A, PIN_S, PIN_D);

    if (cyw43_arch_init() != 0){
        printf("CYW43 init failed\n");
        return 1;
    }
    
    init_gpio();
    set_leds(false, false, false, false);

    l2cap_init();
    sm_init();

    build_gatt();
    printf("notify=0x%04x write=0x%04x cccd=0x%04x\n",
           notify_val_handle, write_val_handle, notify_cccd_handle);

    btstack_run_loop_set_timer_handler(&stream_timer, stream_cb);

    att_server_init(att_db_util_get_address(), &att_read_cb, &att_write_cb);

    hci_event_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_event_cb);
    att_server_register_packet_handler(&packet_handler);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
    return 0;
}