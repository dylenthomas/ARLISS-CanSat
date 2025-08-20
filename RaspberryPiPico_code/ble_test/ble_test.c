#include <stdio.h>
#include "pico/stdlib.h"
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "tusb.h"
#include "pico/btstack_cyw43.h"

//https://github.com/bluekitchen/btstack/blob/develop/example/gatt_counter.c

static uint8_t adv_data[]  ={
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    // Name
    0x0b, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'p', 'i', 'c', 'o', 'w',
    // Service class UUID
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x14, 0x12, 0x8A, 0x76,
    0x04, 0xD1, 0x6C, 0x4F,
    0x7E, 0x53, 0xF2, 0xE8,
    0x00, 0x00, 0xB1, 0x19
};

const uint8_t adv_data_len = sizeof(adv_data);

int main()
{
    stdio_init_all();
}
