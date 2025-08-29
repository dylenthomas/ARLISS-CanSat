#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "tusb.h"
#include "Neo6M_UBXParser.h"

#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4
#define UART_RX_PIN 5

struct posllhData posllh;
struct velnedData velned;
struct statusData status;

int main()
{
    stdio_init_all();

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART

    while(!tud_cdc_connected());

    float lat = posllh.lat;
    float lon = posllh.lon;
    bool gpsFix = status.gpsFixOk;

    while(true) {
        while(uart_is_readable(UART_ID)) {
            unsigned char c = uart_getc(UART_ID);
            addByte(c);
        }

        if (posllhChanged()) {
            lat = posllh.lat;
            lon = posllh.lon;
        }

        if (statusChanged()) {
            gpsFix = status.gpsFixOk;
        }

        printf("GPS_fix: %d: Longitude: %2.6f, Latitude: %2.6f", gpsFix, lon, lat);
    }
}
