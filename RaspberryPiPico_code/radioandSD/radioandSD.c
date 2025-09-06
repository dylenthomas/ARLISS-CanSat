//sck - gp14
//miso - gp12
//mosi - gp11
//cs - gp9
//reset - gp8

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "LoRa.h"

// Pin definitions for Raspberry Pi Pico W
#define CS_PIN     9
#define RESET_PIN  8
#define IRQ_PIN    -1    // Not using DIO1

// Define a structure to hold SPI and CS pin information
typedef struct {
    spi_inst_t *spi;
    uint cs_pin;
} pico_spi_context_t;

// Global variables
absolute_time_t lastTransmit;
const uint32_t TX_INTERVAL_MS = 250;

// HAL implementation for Raspberry Pi Pico
void pico_spi_transfer(void *user, const uint8_t *tx, uint8_t *rx, size_t n) {
    pico_spi_context_t *ctx = (pico_spi_context_t *)user;
    spi_inst_t *spi = ctx->spi;
    
    if (tx && rx) {
        spi_write_read_blocking(spi, tx, rx, n);
    } else if (tx) {
        spi_write_blocking(spi, tx, n);
    } else if (rx) {
        spi_read_blocking(spi, 0, rx, n);
    }
}

void pico_cs_set(void *user, bool active) {
    pico_spi_context_t *ctx = (pico_spi_context_t *)user;
    gpio_put(ctx->cs_pin, active ? 0 : 1); // Active low
}

void pico_delay_ms(void *user, uint32_t ms) {
    sleep_ms(ms);
}

void pico_gpio_write(void *user, int pin, bool level) {
    gpio_put(pin, level);
}

bool pico_gpio_read(void *user, int pin) {
    return gpio_get(pin);
}

void pico_spi_set_frequency(void *user, uint32_t hz) {
    pico_spi_context_t *ctx = (pico_spi_context_t *)user;
    spi_set_baudrate(ctx->spi, hz);
}

// Create a global context for SPI and CS
pico_spi_context_t spi_ctx = {
    .spi = spi1,
    .cs_pin = CS_PIN
};

// Create HAL instance
lora_hal_t pico_hal = {
    .spi_transfer = pico_spi_transfer,
    .cs_set = pico_cs_set,
    .delay_ms = pico_delay_ms,
    .gpio_write = pico_gpio_write,
    .gpio_read = pico_gpio_read,
    .spi_set_frequency = pico_spi_set_frequency,
    .user = (void *)&spi_ctx  // Use the context structure
};

// Global LoRa instance
LoRa *lora;

int main() {
    // Initialize stdio
    stdio_init_all();

    printf("LoRa Transmitter Starting...\n");
    // Initialize SPI
    spi_init(spi1, 8000000); // 8MHz
    gpio_set_function(14, GPIO_FUNC_SPI); // SCK
    gpio_set_function(11, GPIO_FUNC_SPI); // MOSI
    gpio_set_function(12, GPIO_FUNC_SPI); // MISO
    
    // Initialize CS pin
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1); // Active low, so set high initially
    
    // Initialize reset pin if used
    if (RESET_PIN >= 0) {
        gpio_init(RESET_PIN);
        gpio_set_dir(RESET_PIN, GPIO_OUT);
        gpio_put(RESET_PIN, 1); // Set high initially
    }
    
    // Create LoRa instance
    lora = lora_create(&pico_hal, CS_PIN, RESET_PIN, IRQ_PIN);
    if (!lora) {
        printf("Failed to create LoRa instance!\n");
        return 1;
    }
    
    // Configure LoRa (must match receiver settings)
    lora_set_spreading_factor(lora, 12);
    lora_set_signal_bandwidth(lora, 7800); // 7.8kHz
    lora_set_coding_rate4(lora, 8);
    lora_set_tx_power(lora, 20, 1); // 20dBm, PA_BOOST
    lora_set_preamble_length(lora, 12);
    lora_set_sync_word(lora, 0x12);
    
    // Initialize LoRa
    while (!lora_begin(lora, 915000000)) {
        printf("LoRa init failed!\n");
        sleep_ms(5000);
    }
    
    printf("LoRa transmitter ready.\n");
    
    // Initialize lastTransmit
    lastTransmit = get_absolute_time();
    
    // Counter for varying the message
    int counter = 0;
    
    while (1) {
        // Transmit at intervals
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(lastTransmit, now) >= TX_INTERVAL_MS * 1000) {
            lastTransmit = now;
            
            // Create payload string (simulating GPS coordinates)
            char payload[30];
            // Vary the coordinates slightly to simulate movement
            float lat = 69.67f;
            float lon = 420.0f;
            snprintf(payload, sizeof(payload), "%.5f,%.5f", lat, lon);
            
            // Send packet
            lora_begin_packet(lora, 0);
            lora_write(lora, (const uint8_t *)payload, strlen(payload));
            lora_end_packet(lora, false);
            
            printf("Sent: %s\n", payload);
            
        }
        
        // Small delay to prevent busy waiting
        sleep_ms(10);
    }
    
    lora_destroy(lora);
    return 0;
}