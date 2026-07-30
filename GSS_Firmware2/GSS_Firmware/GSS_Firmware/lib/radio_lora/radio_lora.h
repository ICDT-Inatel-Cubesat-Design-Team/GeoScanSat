#pragma once

#include <Arduino.h>
#include <stddef.h>

typedef struct
{
  uint8_t sck_pin;
  uint8_t rst_pin;
  uint8_t miso_pin;
  uint8_t mosi_pin;
  uint8_t cs_pin;
  uint8_t dio0_pin;

  long spi_hz;

  long frequency_hz;
  long bandwidth_hz;
  uint8_t spreading_factor;
  uint8_t coding_rate_denom;
  uint8_t preamble_len;
  uint8_t sync_word;
  int tx_power_dbm;

  bool crc_enable;
} radio_lora_config_t;

typedef struct
{
  uint32_t ts_ms;
  float lat;
  float lon;
  uint8_t s;
} lora_module_rx_t;

typedef struct
{
  uint32_t ts_ms;

  float conductivity_us_cm;
  float turbidity_ntu;
  float ph;
  float temperature_c;

  uint8_t status;
} water_module_rx_t;

bool radio_lora_init(const radio_lora_config_t *cfg);
bool radio_lora_try_read_module_rx(lora_module_rx_t *out);
bool radio_lora_send_bytes(const uint8_t *data, size_t len);
bool radio_lora_try_read_water_rx(water_module_rx_t *out);

int  radio_lora_last_packet_len();
int  radio_lora_last_rssi();
float radio_lora_last_snr();
long radio_lora_last_freq_error();
uint8_t radio_lora_read_version_reg();
