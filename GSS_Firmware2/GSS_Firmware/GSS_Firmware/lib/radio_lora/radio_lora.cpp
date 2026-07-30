#include "radio_lora.h"

#include <SPI.h>
#include <LoRa.h>

#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static radio_lora_config_t g_cfg{};
static bool g_inited = false;

static int g_last_len = 0;
static int g_last_rssi = 0;
static float g_last_snr = NAN;
static long g_last_fe = 0;

static SemaphoreHandle_t g_lora_mutex = nullptr;

static void lock_lora()
{
  if (g_lora_mutex) (void)xSemaphoreTake(g_lora_mutex, portMAX_DELAY);
}

static void unlock_lora()
{
  if (g_lora_mutex) (void)xSemaphoreGive(g_lora_mutex);
}

static void reset_pulse()
{
  pinMode(g_cfg.rst_pin, OUTPUT);
  digitalWrite(g_cfg.rst_pin, HIGH);
  delay(10);
  digitalWrite(g_cfg.rst_pin, LOW);
  delay(10);
  digitalWrite(g_cfg.rst_pin, HIGH);
  delay(10);
}

static uint8_t spi_read_reg(uint8_t reg)
{
  SPI.beginTransaction(SPISettings(g_cfg.spi_hz, MSBFIRST, SPI_MODE0));
  digitalWrite(g_cfg.cs_pin, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(g_cfg.cs_pin, HIGH);
  SPI.endTransaction();
  return v;
}

static bool parse_float_field(const char *buf, const char *key, float *out)
{
  const char *p = strstr(buf, key);
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  p++;

  while (*p && (isspace((unsigned char)*p) || *p == '\"')) p++;

  char *end = nullptr;
  float v = strtof(p, &end);
  if (end == p) return false;

  *out = v;
  return true;
}

static bool parse_int_field(const char *buf, const char *key, long *out)
{
  const char *p = strstr(buf, key);
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  p++;

  while (*p && (isspace((unsigned char)*p) || *p == '\"')) p++;

  char *end = nullptr;
  long v = strtol(p, &end, 10);
  if (end == p) return false;

  *out = v;
  return true;
}

uint8_t radio_lora_read_version_reg()
{
  lock_lora();
  uint8_t v = spi_read_reg(0x42);
  unlock_lora();
  return v;
}

bool radio_lora_init(const radio_lora_config_t *cfg)
{
  if (!cfg) return false;
  g_cfg = *cfg;

  if (!g_lora_mutex) g_lora_mutex = xSemaphoreCreateMutex();

  pinMode(g_cfg.cs_pin, OUTPUT);
  digitalWrite(g_cfg.cs_pin, HIGH);

  pinMode(g_cfg.dio0_pin, INPUT);

  reset_pulse();

  SPI.begin(g_cfg.sck_pin, g_cfg.miso_pin, g_cfg.mosi_pin, g_cfg.cs_pin);

  lock_lora();

  LoRa.setPins(g_cfg.cs_pin, g_cfg.rst_pin, (int)g_cfg.dio0_pin);
  LoRa.setSPIFrequency(g_cfg.spi_hz);

  if (!LoRa.begin(g_cfg.frequency_hz))
  {
    g_inited = false;
    unlock_lora();
    return false;
  }

  LoRa.setSignalBandwidth(g_cfg.bandwidth_hz);
  LoRa.setSpreadingFactor(g_cfg.spreading_factor);
  LoRa.setCodingRate4(g_cfg.coding_rate_denom);
  LoRa.setPreambleLength(g_cfg.preamble_len);

  if (g_cfg.crc_enable) LoRa.enableCrc();
  else LoRa.disableCrc();

  LoRa.setTxPower(g_cfg.tx_power_dbm);
  LoRa.setSyncWord(g_cfg.sync_word);

  LoRa.receive();

  g_inited = true;

  unlock_lora();
  return true;
}

bool radio_lora_try_read_module_rx(lora_module_rx_t *out)
{
  if (!g_inited || !out) return false;

  lock_lora();

  const int packet_len = LoRa.parsePacket();
  if (packet_len <= 0)
  {
    unlock_lora();
    return false;
  }

  g_last_len = packet_len;
  g_last_rssi = LoRa.packetRssi();
  g_last_snr = LoRa.packetSnr();
  g_last_fe = LoRa.packetFrequencyError();

  char buf[256];
  int i = 0;
  while (LoRa.available() && i < (int)sizeof(buf) - 1)
  {
    buf[i++] = (char)LoRa.read();
  }
  buf[i] = '\0';
  
  buf[i] = '\0';

  Serial.print("[LORA RAW] ");
  Serial.println(buf);

  Serial.print("[LORA LEN] ");
  Serial.println(packet_len);

  Serial.print("[LORA RSSI] ");
  Serial.println(g_last_rssi);

  Serial.print("[LORA SNR] ");
  Serial.println(g_last_snr);

  float lat = NAN, lon = NAN;
  long s = -1;

  const bool ok_lat = parse_float_field(buf, "lat", &lat);
  const bool ok_lon = parse_float_field(buf, "lon", &lon);
  const bool ok_s   = parse_int_field(buf, "s", &s);

  out->ts_ms = millis();
  out->lat = lat;
  out->lon = lon;
  out->s = (s != 0) ? 1 : 0;

  LoRa.receive();

  unlock_lora();

  return (ok_lat && ok_lon && ok_s);
}

bool radio_lora_try_read_water_rx(water_module_rx_t *out)
{
  if (!g_inited || !out)
  {
    return false;
  }

  lock_lora();

  const int packet_len = LoRa.parsePacket();

  if (packet_len <= 0)
  {
    unlock_lora();
    return false;
  }

  g_last_len  = packet_len;
  g_last_rssi = LoRa.packetRssi();
  g_last_snr  = LoRa.packetSnr();
  g_last_fe   = LoRa.packetFrequencyError();

  char buf[256];
  int i = 0;

  while (
    LoRa.available() &&
    i < (int)sizeof(buf) - 1
  )
  {
    const int value = LoRa.read();

    if (value < 0)
    {
      break;
    }

    buf[i++] = (char)value;
  }

  buf[i] = '\0';

  // Descarta bytes excedentes, caso o pacote seja maior que o buffer.
  while (LoRa.available())
  {
    LoRa.read();
  }

  Serial.println();

  Serial.print("[WATER RAW] ");
  Serial.println(buf);

  Serial.print("[WATER LEN] ");
  Serial.println(packet_len);

  Serial.print("[WATER READ] ");
  Serial.println(i);

  Serial.print("[WATER RSSI] ");
  Serial.println(g_last_rssi);

  Serial.print("[WATER SNR] ");
  Serial.println(g_last_snr);

  Serial.print("[WATER FE] ");
  Serial.println(g_last_fe);

  float conductivity = NAN;
  float turbidity    = NAN;
  float ph           = NAN;
  float temperature  = NAN;
  long status        = -1;

  const bool ok_cond =
    parse_float_field(
      buf,
      "cond",
      &conductivity
    );

  const bool ok_turb =
    parse_float_field(
      buf,
      "turb",
      &turbidity
    );

  const bool ok_ph =
    parse_float_field(
      buf,
      "ph",
      &ph
    );

  const bool ok_temp =
    parse_float_field(
      buf,
      "temp",
      &temperature
    );

  const bool ok_status =
    parse_int_field(
      buf,
      "s",
      &status
    );

  if (!(
    ok_cond &&
    ok_turb &&
    ok_ph &&
    ok_temp &&
    ok_status
  ))
  {
    Serial.println(
      "[WATER] Erro ao interpretar pacote"
    );

    Serial.print("  cond encontrada: ");
    Serial.println(ok_cond ? "sim" : "nao");

    Serial.print("  turb encontrada: ");
    Serial.println(ok_turb ? "sim" : "nao");

    Serial.print("  ph encontrado: ");
    Serial.println(ok_ph ? "sim" : "nao");

    Serial.print("  temp encontrada: ");
    Serial.println(ok_temp ? "sim" : "nao");

    Serial.print("  status encontrado: ");
    Serial.println(ok_status ? "sim" : "nao");

    LoRa.receive();
    unlock_lora();

    return false;
  }

  out->ts_ms = millis();

  out->conductivity_us_cm = conductivity;
  out->turbidity_ntu      = turbidity;
  out->ph                 = ph;
  out->temperature_c      = temperature;
  out->status             = (status != 0) ? 1 : 0;

  Serial.printf(
    "[WATER RX] "
    "cond=%.2f uS/cm "
    "turb=%.2f NTU "
    "ph=%.2f "
    "temp=%.2f C "
    "status=%u\n",
    out->conductivity_us_cm,
    out->turbidity_ntu,
    out->ph,
    out->temperature_c,
    (unsigned)out->status
  );

  LoRa.receive();

  unlock_lora();

  return true;
}

bool radio_lora_send_bytes(const uint8_t *data, size_t len)
{
  if (!g_inited || !data || len == 0) return false;
  if (len > 255) return false; // limite típico do SX127x/LoRa lib

  lock_lora();

  // Para transmitir, sai de RX
  LoRa.idle();

  LoRa.beginPacket();
  LoRa.write(data, (int)len);
  const int ok = LoRa.endPacket(); // bloqueante

  // Volta para RX
  LoRa.receive();

  unlock_lora();

  return (ok == 1);
}

int radio_lora_last_packet_len() { return g_last_len; }
int radio_lora_last_rssi() { return g_last_rssi; }
float radio_lora_last_snr() { return g_last_snr; }
long radio_lora_last_freq_error() { return g_last_fe; }
