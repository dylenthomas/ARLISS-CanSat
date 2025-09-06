#include "LoRa.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// Register definitions (same as C++ version)
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// Modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_CAD_DONE_MASK          0x04
#define IRQ_CAD_DETECTED_MASK      0x01

// Constants
#define RF_MID_BAND_THRESHOLD    525000000
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164
#define MAX_PKT_LENGTH           255

// LoRa context structure
struct LoRa {
    const lora_hal_t *hal;
    int ss_pin;
    int reset_pin;
    int dio0_pin;
    long frequency;
    int packet_index;
    int implicit_header_mode;
    lora_on_receive_cb on_receive;
    lora_on_cad_done_cb on_cad_done;
    lora_on_tx_done_cb on_tx_done;
    uint32_t spi_frequency;
};

// Internal helper functions
static uint8_t read_register(LoRa *ctx, uint8_t address);
static void write_register(LoRa *ctx, uint8_t address, uint8_t value);
static uint8_t single_transfer(LoRa *ctx, uint8_t address, uint8_t value);
static void explicit_header_mode(LoRa *ctx);
static void implicit_header_mode(LoRa *ctx);
static void lora_set_ldo_flag(LoRa *ctx);
static bool is_transmitting(LoRa *ctx);
static long get_signal_bandwidth(LoRa *ctx);
static int get_spreading_factor(LoRa *ctx);

static uint8_t read_register(LoRa *ctx, uint8_t address) {
    return single_transfer(ctx, address & 0x7f, 0x00);
}

static void write_register(LoRa *ctx, uint8_t address, uint8_t value) {
    single_transfer(ctx, address | 0x80, value);
}

static uint8_t single_transfer(LoRa *ctx, uint8_t address, uint8_t value) {
    if (!ctx || !ctx->hal) return 0;
    
    uint8_t response;
    uint8_t tx_buf[2] = {address, value};
    uint8_t rx_buf[2] = {0};
    
    ctx->hal->cs_set(ctx->hal->user, true); // Active low, so set to true to enable
    ctx->hal->spi_transfer(ctx->hal->user, tx_buf, rx_buf, 2);
    ctx->hal->cs_set(ctx->hal->user, false); // Disable
    
    response = rx_buf[1]; // Response is received on the second byte
    return response;
}

static void explicit_header_mode(LoRa *ctx) {
    if (!ctx) return;
    ctx->implicit_header_mode = 0;
    write_register(ctx, REG_MODEM_CONFIG_1, read_register(ctx, REG_MODEM_CONFIG_1) & 0xfe);
}

static void implicit_header_mode(LoRa *ctx) {
    if (!ctx) return;
    ctx->implicit_header_mode = 1;
    write_register(ctx, REG_MODEM_CONFIG_1, read_register(ctx, REG_MODEM_CONFIG_1) | 0x01);
}

static bool is_transmitting(LoRa *ctx) {
    if (!ctx) return false;
    
    if ((read_register(ctx, REG_OP_MODE) & MODE_TX) == MODE_TX) {
        return true;
    }
    
    if (read_register(ctx, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
        // Clear IRQ's
        write_register(ctx, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }
    
    return false;
}

static long get_signal_bandwidth(LoRa *ctx) {
    if (!ctx) return 0;
    
    uint8_t bw = (read_register(ctx, REG_MODEM_CONFIG_1) >> 4);
    
    switch (bw) {
        case 0: return 7800;
        case 1: return 10400;
        case 2: return 15600;
        case 3: return 20800;
        case 4: return 31250;
        case 5: return 41700;
        case 6: return 62500;
        case 7: return 125000;
        case 8: return 250000;
        case 9: return 500000;
        default: return -1;
    }
}

static int get_spreading_factor(LoRa *ctx) {
    if (!ctx) return 0;
    return read_register(ctx, REG_MODEM_CONFIG_2) >> 4;
}

static void lora_set_ldo_flag(LoRa *ctx) {
    if (!ctx) return;
    
    // Section 4.1.1.5
    long symbol_duration = 1000 / (get_signal_bandwidth(ctx) / (1L << get_spreading_factor(ctx)));
    
    // Section 4.1.1.6
    bool ldo_on = symbol_duration > 16;
    
    uint8_t config3 = read_register(ctx, REG_MODEM_CONFIG_3);
    if (ldo_on) {
        config3 |= 0x08;
    } else {
        config3 &= 0xf7;
    }
    write_register(ctx, REG_MODEM_CONFIG_3, config3);
}


LoRa *lora_create(const lora_hal_t *hal, int ss_pin, int reset_pin, int dio0_pin) {
    LoRa *ctx = (LoRa *)malloc(sizeof(LoRa));
    if (!ctx) return NULL;
    
    memset(ctx, 0, sizeof(LoRa));
    ctx->hal = hal;
    ctx->ss_pin = ss_pin;
    ctx->reset_pin = reset_pin;
    ctx->dio0_pin = dio0_pin;
    ctx->spi_frequency = 8000000; // Default 8MHz
    
    return ctx;
}

void lora_destroy(LoRa *ctx) {
    if (ctx) free(ctx);
}

void lora_set_pins(LoRa *ctx, int ss_pin, int reset_pin, int dio0_pin) {
    if (!ctx) return;
    ctx->ss_pin = ss_pin;
    ctx->reset_pin = reset_pin;
    ctx->dio0_pin = dio0_pin;
}

void lora_set_spi_frequency(LoRa *ctx, uint32_t hz) {
    if (!ctx) return;
    ctx->spi_frequency = hz;
    if (ctx->hal->spi_set_frequency) {
        ctx->hal->spi_set_frequency(ctx->hal->user, hz);
    }
}

int lora_begin(LoRa *ctx, long frequency_hz) {
    if (!ctx || !ctx->hal) return 0;

    // Setup SS pin
    ctx->hal->cs_set(ctx->hal->user, false); // Active low, so set to false to disable initially
    
    // Handle reset if reset pin is configured
    if (ctx->reset_pin >= 0 && ctx->hal->gpio_write) {
        ctx->hal->gpio_write(ctx->hal->user, ctx->reset_pin, false);
        ctx->hal->delay_ms(ctx->hal->user, 10);
        ctx->hal->gpio_write(ctx->hal->user, ctx->reset_pin, true);
        ctx->hal->delay_ms(ctx->hal->user, 10);
    }
    
    // Check version
    uint8_t version = read_register(ctx, REG_VERSION);
    if (version != 0x12) {
        return 0;
    }
    
    // Put in sleep mode
    lora_sleep(ctx);
    
    // Set frequency
    lora_set_frequency(ctx, frequency_hz);
    
    // Set base addresses
    write_register(ctx, REG_FIFO_TX_BASE_ADDR, 0);
    write_register(ctx, REG_FIFO_RX_BASE_ADDR, 0);
    
    // Set LNA boost
    write_register(ctx, REG_LNA, read_register(ctx, REG_LNA) | 0x03);
    
    // Set auto AGC
    write_register(ctx, REG_MODEM_CONFIG_3, 0x04);
    
    // Set output power to 17 dBm
    lora_set_tx_power(ctx, 17, 1); // PA_BOOST
    
    // Put in standby mode
    lora_idle(ctx);
    
    return 1;
}

void lora_end(LoRa *ctx) {
    if (!ctx) return;
    lora_sleep(ctx);
}

int lora_begin_packet(LoRa *ctx, int implicit_header) {
    if (!ctx || is_transmitting(ctx)) return 0;
    
    // Put in standby mode
    lora_idle(ctx);
    
    if (implicit_header) {
        implicit_header_mode(ctx);
    } else {
        explicit_header_mode(ctx);
    }
    
    // Reset FIFO address and payload length
    write_register(ctx, REG_FIFO_ADDR_PTR, 0);
    write_register(ctx, REG_PAYLOAD_LENGTH, 0);
    
    return 1;
}

int lora_end_packet(LoRa *ctx, bool async) {
    if (!ctx) return 0;
    
    if (async && ctx->on_tx_done) {
        write_register(ctx, REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
    }
    
    // Put in TX mode
    write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    
    if (!async) {
        // Wait for TX done
        while ((read_register(ctx, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
            // Yield equivalent - just a small delay
            ctx->hal->delay_ms(ctx->hal->user, 1);
        }
        // Clear IRQ's
        write_register(ctx, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }
    
    return 1;
}

int lora_parse_packet(LoRa *ctx, int size) {
    if (!ctx) return 0;
    
    int packet_length = 0;
    int irq_flags = read_register(ctx, REG_IRQ_FLAGS);
    
    if (size > 0) {
        implicit_header_mode(ctx);
        write_register(ctx, REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        explicit_header_mode(ctx);
    }
    
    // Clear IRQ's
    write_register(ctx, REG_IRQ_FLAGS, irq_flags);
    
    if ((irq_flags & IRQ_RX_DONE_MASK) && (irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        // Received a packet
        ctx->packet_index = 0;
        
        // Read packet length
        if (ctx->implicit_header_mode) {
            packet_length = read_register(ctx, REG_PAYLOAD_LENGTH);
        } else {
            packet_length = read_register(ctx, REG_RX_NB_BYTES);
        }
        
        // Set FIFO address to current RX address
        write_register(ctx, REG_FIFO_ADDR_PTR, read_register(ctx, REG_FIFO_RX_CURRENT_ADDR));
        
        // Put in standby mode
        lora_idle(ctx);
    } else if (read_register(ctx, REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
        // Not currently in RX mode
        
        // Reset FIFO address
        write_register(ctx, REG_FIFO_ADDR_PTR, 0);
        
        // Put in single RX mode
        write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }
    
    return packet_length;
}

int lora_packet_rssi(LoRa *ctx) {
    if (!ctx) return 0;
    return (read_register(ctx, REG_PKT_RSSI_VALUE) - 
           (ctx->frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float lora_packet_snr(LoRa *ctx) {
    if (!ctx) return 0.0f;
    return ((int8_t)read_register(ctx, REG_PKT_SNR_VALUE)) * 0.25f;
}

long lora_packet_frequency_error(LoRa *ctx) {
    if (!ctx) return 0;
    
    int32_t freq_error = 0;
    freq_error = (int32_t)(read_register(ctx, REG_FREQ_ERROR_MSB) & 0b111);
    freq_error <<= 8L;
    freq_error += (int32_t)read_register(ctx, REG_FREQ_ERROR_MID);
    freq_error <<= 8L;
    freq_error += (int32_t)read_register(ctx, REG_FREQ_ERROR_LSB);
    
    if (read_register(ctx, REG_FREQ_ERROR_MSB) & 0b1000) {
        freq_error -= 524288; // Sign bit is on
    }
    
    const float f_xtal = 32E6; // Crystal oscillator frequency
    long bandwidth = get_signal_bandwidth(ctx);
    float f_error = ((float)freq_error * (1L << 24) / f_xtal) * (bandwidth / 500000.0f);
    
    return (long)f_error;
}

int lora_rssi(LoRa *ctx) {
    if (!ctx) return 0;
    return (read_register(ctx, REG_RSSI_VALUE) - 
           (ctx->frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

int lora_available(LoRa *ctx) {
    if (!ctx) return 0;
    return (read_register(ctx, REG_RX_NB_BYTES) - ctx->packet_index);
}

int lora_read(LoRa *ctx) {
    if (!ctx || !lora_available(ctx)) return -1;
    ctx->packet_index++;
    return read_register(ctx, REG_FIFO);
}

int lora_peek(LoRa *ctx) {
    if (!ctx || !lora_available(ctx)) return -1;
    
    // Store current FIFO address
    int current_address = read_register(ctx, REG_FIFO_ADDR_PTR);
    
    // Read
    uint8_t b = read_register(ctx, REG_FIFO);
    
    // Restore FIFO address
    write_register(ctx, REG_FIFO_ADDR_PTR, current_address);
    
    return b;
}

void lora_flush(LoRa *ctx) {
    // Nothing to do in C implementation
}

int lora_write_byte(LoRa *ctx, uint8_t b) {
    return lora_write(ctx, &b, 1);
}

int lora_write(LoRa *ctx, const uint8_t *buf, size_t len) {
    if (!ctx || !buf) return 0;
    
    int current_length = read_register(ctx, REG_PAYLOAD_LENGTH);
    
    // Check size
    if ((current_length + len) > MAX_PKT_LENGTH) {
        len = MAX_PKT_LENGTH - current_length;
    }
    
    // Write data
    for (size_t i = 0; i < len; i++) {
        write_register(ctx, REG_FIFO, buf[i]);
    }
    
    // Update length
    write_register(ctx, REG_PAYLOAD_LENGTH, current_length + len);
    
    return len;
}

void lora_idle(LoRa *ctx) {
    if (!ctx) return;
    write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void lora_sleep(LoRa *ctx) {
    if (!ctx) return;
    write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void lora_set_tx_power(LoRa *ctx, int level_dbm, int use_pa_boost) {
    if (!ctx) return;
    
    if (use_pa_boost == 0) { // RFO
        if (level_dbm < 0) level_dbm = 0;
        else if (level_dbm > 14) level_dbm = 14;
        
        write_register(ctx, REG_PA_CONFIG, 0x70 | level_dbm);
    } else { // PA BOOST
        if (level_dbm > 17) {
            if (level_dbm > 20) level_dbm = 20;
            
            // Subtract 3 from level, so 18-20 maps to 15-17
            level_dbm -= 3;
            
            // High Power +20 dBm Operation
            write_register(ctx, REG_PA_DAC, 0x87);
            lora_set_ocp(ctx, 140);
        } else {
            if (level_dbm < 2) level_dbm = 2;
            
            // Default value PA_HF/LF or +17dBm
            write_register(ctx, REG_PA_DAC, 0x84);
            lora_set_ocp(ctx, 100);
        }
        
        write_register(ctx, REG_PA_CONFIG, PA_BOOST | (level_dbm - 2));
    }
}

void lora_set_frequency(LoRa *ctx, long frequency_hz) {
    if (!ctx) return;
    ctx->frequency = frequency_hz;
    
    uint64_t frf = ((uint64_t)frequency_hz << 19) / 32000000;
    
    write_register(ctx, REG_FRF_MSB, (uint8_t)(frf >> 16));
    write_register(ctx, REG_FRF_MID, (uint8_t)(frf >> 8));
    write_register(ctx, REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_set_spreading_factor(LoRa *ctx, int sf) {
    if (!ctx) return;
    
    if (sf < 6) sf = 6;
    else if (sf > 12) sf = 12;
    
    if (sf == 6) {
        write_register(ctx, REG_DETECTION_OPTIMIZE, 0xc5);
        write_register(ctx, REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        write_register(ctx, REG_DETECTION_OPTIMIZE, 0xc3);
        write_register(ctx, REG_DETECTION_THRESHOLD, 0x0a);
    }
    
    write_register(ctx, REG_MODEM_CONFIG_2, 
                  (read_register(ctx, REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    lora_set_ldo_flag(ctx);
}

void lora_set_signal_bandwidth(LoRa *ctx, long bandwidth_hz) {
    if (!ctx) return;
    
    int bw;
    
    if (bandwidth_hz <= 7800) bw = 0;
    else if (bandwidth_hz <= 10400) bw = 1;
    else if (bandwidth_hz <= 15600) bw = 2;
    else if (bandwidth_hz <= 20800) bw = 3;
    else if (bandwidth_hz <= 31250) bw = 4;
    else if (bandwidth_hz <= 41700) bw = 5;
    else if (bandwidth_hz <= 62500) bw = 6;
    else if (bandwidth_hz <= 125000) bw = 7;
    else if (bandwidth_hz <= 250000) bw = 8;
    else bw = 9;
    
    write_register(ctx, REG_MODEM_CONFIG_1, 
                  (read_register(ctx, REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    lora_set_ldo_flag(ctx);
}

void lora_set_coding_rate4(LoRa *ctx, int denominator) {
    if (!ctx) return;
    
    if (denominator < 5) denominator = 5;
    else if (denominator > 8) denominator = 8;
    
    int cr = denominator - 4;
    
    write_register(ctx, REG_MODEM_CONFIG_1, 
                  (read_register(ctx, REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void lora_set_preamble_length(LoRa *ctx, long len_symbols) {
    if (!ctx) return;
    write_register(ctx, REG_PREAMBLE_MSB, (uint8_t)(len_symbols >> 8));
    write_register(ctx, REG_PREAMBLE_LSB, (uint8_t)(len_symbols >> 0));
}

void lora_set_sync_word(LoRa *ctx, int sw) {
    if (!ctx) return;
    write_register(ctx, REG_SYNC_WORD, sw);
}

void lora_enable_crc(LoRa *ctx) {
    if (!ctx) return;
    write_register(ctx, REG_MODEM_CONFIG_2, read_register(ctx, REG_MODEM_CONFIG_2) | 0x04);
}

void lora_disable_crc(LoRa *ctx) {
    if (!ctx) return;
    write_register(ctx, REG_MODEM_CONFIG_2, read_register(ctx, REG_MODEM_CONFIG_2) & 0xfb);
}

void lora_enable_invert_iq(LoRa *ctx) {
    if (!ctx) return;
    write_register(ctx, REG_INVERTIQ, 0x66);
    write_register(ctx, REG_INVERTIQ2, 0x19);
}

void lora_disable_invert_iq(LoRa *ctx) {
    if (!ctx) return;
    write_register(ctx, REG_INVERTIQ, 0x27);
    write_register(ctx, REG_INVERTIQ2, 0x1d);
}

void lora_enable_low_data_rate_optimize(LoRa *ctx) {
    if (!ctx) return;
    uint8_t config3 = read_register(ctx, REG_MODEM_CONFIG_3);
    write_register(ctx, REG_MODEM_CONFIG_3, config3 | 0x08);
}

void lora_disable_low_data_rate_optimize(LoRa *ctx) {
    if (!ctx) return;
    uint8_t config3 = read_register(ctx, REG_MODEM_CONFIG_3);
    write_register(ctx, REG_MODEM_CONFIG_3, config3 & 0xf7);
}

void lora_set_ocp(LoRa *ctx, uint8_t ma) {
    if (!ctx) return;
    
    uint8_t ocp_trim = 27;
    
    if (ma <= 120) {
        ocp_trim = (ma - 45) / 5;
    } else if (ma <= 240) {
        ocp_trim = (ma + 30) / 10;
    }
    
    write_register(ctx, REG_OCP, 0x20 | (0x1F & ocp_trim));
}

void lora_set_gain(LoRa *ctx, uint8_t gain) {
    if (!ctx) return;
    
    // Check allowed range
    if (gain > 6) gain = 6;
    
    // Set to standby
    lora_idle(ctx);
    
    // Set gain
    if (gain == 0) {
        // If gain = 0, enable AGC
        write_register(ctx, REG_MODEM_CONFIG_3, 0x04);
    } else {
        // Disable AGC
        write_register(ctx, REG_MODEM_CONFIG_3, 0x00);
        
        // Clear Gain and set LNA boost
        write_register(ctx, REG_LNA, 0x03);
        
        // Set gain
        write_register(ctx, REG_LNA, read_register(ctx, REG_LNA) | (gain << 5));
    }
}

int lora_random(LoRa *ctx) {
    if (!ctx) return 0;
    return read_register(ctx, REG_RSSI_WIDEBAND);
}



void lora_receive(LoRa *ctx, int size) {
    if (!ctx) return;
    
    write_register(ctx, REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE
    
    if (size > 0) {
        implicit_header_mode(ctx);
        write_register(ctx, REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        explicit_header_mode(ctx);
    }
    
    write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void lora_channel_activity_detection(LoRa *ctx) {
    if (!ctx) return;
    write_register(ctx, REG_DIO_MAPPING_1, 0x80); // DIO0 => CADDONE
    write_register(ctx, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}

void lora_on_receive(LoRa *ctx, lora_on_receive_cb cb) {
    if (ctx) ctx->on_receive = cb;
}

void lora_on_cad_done(LoRa *ctx, lora_on_cad_done_cb cb) {
    if (ctx) ctx->on_cad_done = cb;
}

void lora_on_tx_done(LoRa *ctx, lora_on_tx_done_cb cb) {
    if (ctx) ctx->on_tx_done = cb;
}

int lora_get_implicit_header_mode(LoRa *ctx) {
    return ctx ? ctx->implicit_header_mode : 0;
}

void lora_set_implicit_header_mode(LoRa *ctx, int enable) {
    if (!ctx) return;
    
    if (enable) {
        implicit_header_mode(ctx);
    } else {
        explicit_header_mode(ctx);
    }
}

int lora_read_dio0(LoRa *ctx) {
    if (!ctx || !ctx->hal->gpio_read || ctx->dio0_pin < 0) return -1;
    return ctx->hal->gpio_read(ctx->hal->user, ctx->dio0_pin) ? 1 : 0;
}

int lora_chip_version(LoRa *ctx) {
    if (!ctx) return -1;
    return read_register(ctx, REG_VERSION);
}

