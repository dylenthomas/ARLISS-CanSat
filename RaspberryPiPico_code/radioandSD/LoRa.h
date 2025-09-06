/* lora.h — C rewrite of Arduino LoRa.h (Sandeep Mistry) without C++/Arduino types.
 * Public domain-style API wrapper around SX1276/77/78/79 LoRa radios.
 *
 * You must provide a tiny HAL (SPI, GPIO, delay) via lora_hal_t.
 * This header mirrors the original class methods as C functions with a context.
 */

#ifndef LORA_C_H
#define LORA_C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------- Platform HAL -------------------- */

typedef struct {
  /* Called to transfer exactly n bytes. If rx==NULL, discard RX. If tx==NULL, send zeros. */
  void (*spi_transfer)(void *user, const uint8_t *tx, uint8_t *rx, size_t n);

  /* Chip-select control (active-low). */
  void (*cs_set)(void *user, bool active);

  /* Millisecond delay. */
  void (*delay_ms)(void *user, uint32_t ms);

  /* Optional: pin I/O for reset and DIO0 (if you wire them). If unavailable, set to NULL. */
  void (*gpio_write)(void *user, int pin, bool level);     /* e.g., reset pin */
  bool (*gpio_read)(void *user, int pin);                  /* e.g., DIO0 pin */

  /* Optional: set SPI clock (Hz). Set to NULL if fixed by your platform. */
  void (*spi_set_frequency)(void *user, uint32_t hz);

  /* Opaque handle passed back to each HAL call. */
  void *user;
} lora_hal_t;

/* -------------------- Forward types & callbacks -------------------- */

typedef struct LoRa LoRa;

typedef void (*lora_on_receive_cb)(int packet_len);
typedef void (*lora_on_cad_done_cb)(bool channel_active);
typedef void (*lora_on_tx_done_cb)(void);

/* -------------------- Construction / setup -------------------- */

/* Create/destroy a LoRa context. Pins may be -1 if unused/not wired. */
LoRa *lora_create(const lora_hal_t *hal, int ss_pin, int reset_pin, int dio0_pin);
void  lora_destroy(LoRa *ctx);

/* Change pins after creation (explicit). */
void  lora_set_pins(LoRa *ctx, int ss_pin, int reset_pin, int dio0_pin);

/* Optional: set SPI clock (Hz). No-op if HAL didn't provide spi_set_frequency. */
void  lora_set_spi_frequency(LoRa *ctx, uint32_t hz);

/* Initialize the radio and enter standby at a given RF frequency (Hz).
 * Returns 1 on success, 0 on failure. */
int   lora_begin(LoRa *ctx, long frequency_hz);

/* Power down the radio and free any active buffers (context remains valid). */
void  lora_end(LoRa *ctx);

/* -------------------- Packet TX/RX (stream-like) -------------------- */

/* Start a packet. implicit_header = 0 => explicit header (default), 1 => implicit header mode. */
int   lora_begin_packet(LoRa *ctx, int implicit_header);

/* Finish TX. async = false blocks until TX done; true returns immediately. */
int   lora_end_packet(LoRa *ctx, bool async);

/* Put radio in continuous RX. If size>0, uses implicit header with that size. */
void  lora_receive(LoRa *ctx, int size);

/* Parse an incoming packet (moves RX data into FIFO). If size>0, implicit mode with that size.
 * Returns packet length, or 0 if none available. */
int   lora_parse_packet(LoRa *ctx, int size);

/* Number of bytes still available to read from current packet. */
int   lora_available(LoRa *ctx);

/* Read a single byte from RX FIFO; returns -1 if none. */
int   lora_read(LoRa *ctx);

/* Peek next byte without removing; returns -1 if none. */
int   lora_peek(LoRa *ctx);

/* Flush TX FIFO (no-op for RX). */
void  lora_flush(LoRa *ctx);

/* Write a single byte to TX FIFO; returns 1 on success, 0 on failure. */
int   lora_write_byte(LoRa *ctx, uint8_t b);

/* Write a buffer to TX FIFO; returns number of bytes written. */
int   lora_write(LoRa *ctx, const uint8_t *buf, size_t len);

/* -------------------- Mode control -------------------- */

void  lora_idle(LoRa *ctx);   /* Standby */
void  lora_sleep(LoRa *ctx);  /* Sleep   */

/* -------------------- Radio configuration -------------------- */

void  lora_set_tx_power(LoRa *ctx, int level_dbm, int use_pa_boost /*0=RFO, nonzero=PA_BOOST*/);
void  lora_set_frequency(LoRa *ctx, long frequency_hz);         /* e.g., 915000000 */
void  lora_set_spreading_factor(LoRa *ctx, int sf);             /* 6..12 */
void  lora_set_signal_bandwidth(LoRa *ctx, long bandwidth_hz);  /* e.g., 7800..500000 */
void  lora_set_coding_rate4(LoRa *ctx, int denominator);        /* 5..8 => 4/5..4/8 */
void  lora_set_preamble_length(LoRa *ctx, long len_symbols);    /* default ~8 */
void  lora_set_sync_word(LoRa *ctx, int sw);                    /* 0x00..0xFF */
void  lora_enable_crc(LoRa *ctx);
void  lora_disable_crc(LoRa *ctx);
/* Aliases for parity with original header */
static inline void lora_crc(LoRa *ctx)    { lora_enable_crc(ctx); }
static inline void lora_no_crc(LoRa *ctx) { lora_disable_crc(ctx); }

void  lora_enable_invert_iq(LoRa *ctx);
void  lora_disable_invert_iq(LoRa *ctx);

void  lora_set_ocp(LoRa *ctx, uint8_t ma);   /* Overcurrent protection in mA (0=off => defaults) */
void  lora_set_gain(LoRa *ctx, uint8_t gain);/* 0=auto, 1..6 manual */

/* Some extras from the Arduino API */
int   lora_random(LoRa *ctx);          /* returns 16-bit-ish random from wideband RSSI */
static void  lora_set_ldo_flag(LoRa *ctx);    /* configure LDO based on symbol duration */

/* -------------------- Packet/Link stats -------------------- */

int   lora_packet_rssi(LoRa *ctx);     /* dBm (approx) */
float lora_packet_snr(LoRa *ctx);      /* dB */
long  lora_packet_frequency_error(LoRa *ctx); /* Hz */
int   lora_rssi(LoRa *ctx);            /* current RSSI, dBm (approx) */

/* -------------------- CAD (Channel Activity Detection) -------------------- */

void  lora_channel_activity_detection(LoRa *ctx);

/* -------------------- Callbacks (optional) -------------------- */

void  lora_on_receive(LoRa *ctx, lora_on_receive_cb cb);
void  lora_on_cad_done(LoRa *ctx, lora_on_cad_done_cb cb);
void  lora_on_tx_done(LoRa *ctx, lora_on_tx_done_cb cb);

/* -------------------- Introspection -------------------- */

/* Get/set implicit header mode flag explicitly (0=explicit, 1=implicit). */
int   lora_get_implicit_header_mode(LoRa *ctx);
void  lora_set_implicit_header_mode(LoRa *ctx, int enable);


/* Expose DIO0 level (useful if you wired it and want to poll). Returns -1 if HAL lacks gpio_read. */
int   lora_read_dio0(LoRa *ctx);

/* Version register (0x42) or -1 on error. Useful for sanity-checking chip presence. */
int   lora_chip_version(LoRa *ctx);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LORA_C_H */
