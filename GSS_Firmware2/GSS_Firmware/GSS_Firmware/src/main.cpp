#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensors.h"
#include "radio_lora.h"
#include "rasp_comm.h"
#include "gs_packet.h"

#include "wifi_comm.h"
#include "obsat_http.h"

// ===================== WIFI CONFIG =====================
#ifndef WIFI_SSID
#define WIFI_SSID "CSI-Lab"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "In@teLCS&I"
#endif

// Endpoint OBSAT (HTTPS)
static const char *OBSAT_HTTP_URL = "https://obsat.org.br/servidor_testes/envio.php";

// Flag para evitar reconnect/HTTP durante sessão crítica com a rasp
static volatile bool rasp_session_active = false;
static volatile uint32_t rasp_last_end_ms = 0;

// ===================== HW PINS / CONFIG =====================
static const uint8_t i2c_sda_pin = 8;
static const uint8_t i2c_scl_pin = 9;
static const uint32_t i2c_clock_hz = 400000;

static const gpio_num_t adc_battery_pin = GPIO_NUM_3;

static const uint8_t lora_sck_pin  = 4;
static const uint8_t lora_rst_pin  = 2;
static const uint8_t lora_miso_pin = 5;
static const uint8_t lora_mosi_pin = 6;
static const uint8_t lora_cs_pin   = 7;
static const uint8_t lora_dio0_pin = 10;

static const long lora_frequency_hz = 915000000L;
static const long lora_bandwidth_hz = 125000L;
static const uint8_t lora_spreading_factor = 7;
static const uint8_t lora_coding_rate_denom = 5;
static const uint8_t lora_preamble_len = 8;
static const uint8_t lora_sync_word = 0x4C;
static const int lora_tx_power_dbm  = 17;
static const long lora_spi_hz       = 1000000L;

static const float battery_r1_ohm = 91300.0f;
static const float battery_r2_ohm = 62000.0f;

static const float adc_max_counts   = 4095.0f;
static const float adc_full_scale_v = 3.3f;

static const float sea_level_hpa = 1013.25f;

// ===================== TASK PRIORITIES =====================
static const UBaseType_t prio_receive_radio      = 5;
static const UBaseType_t prio_sensor_reading     = 4;
static const UBaseType_t prio_send_radio         = 3;
static const UBaseType_t prio_send_http          = 2;
static const UBaseType_t prio_rasp_communication = 2;
static const UBaseType_t prio_wifi_monitor       = 1;

static const uint32_t stack_small   = 3072;
static const uint32_t stack_medium  = 4096;
static const uint32_t stack_large   = 10240;

// ===================== QUEUE HANDLES =====================
static QueueHandle_t persister_data_queue           = nullptr;
static QueueHandle_t http_sensor_data_queue         = nullptr;
static QueueHandle_t radio_sensor_data_queue        = nullptr;

static QueueHandle_t lora_payload_data_queue        = nullptr;
static QueueHandle_t http_payload_data_queue        = nullptr;

static QueueHandle_t http_module_data_queue         = nullptr;
static QueueHandle_t lora_module_data_queue         = nullptr;
static QueueHandle_t persister_module_data_queue    = nullptr;

static QueueHandle_t water_module_data_queue = nullptr;

static const UBaseType_t queue_length_default = 256;
static const UBaseType_t queue_unique_item    = 1;

// ===================== RASP UART / PROTOCOL =====================
static const int RASP_UART_RX = 20;
static const int RASP_UART_TX = 21;
static const uint32_t RASP_BAUD = 115200;

static const int RASP_HANDSHAKE_PIN = 0; // GPIO0

// ===================== TASK PROTOTYPES =====================
static void sensor_reading_task(void *pv_parameters);
static void receive_radio_task(void *pv_parameters);
static void send_http_task(void *pv_parameters);
static void wifi_monitor_task(void *pv_parameters);
static void rasp_communication_task(void *pv_parameters);
static void send_radio_task(void *pv_parameters);

// ===================== RASP COMM HANDLE =====================
static rasp_comm_t rasp_comm;

// ===================== SETUP =====================
void setup()
{
  Serial.begin(115200);
  delay(200);

  // Wi-Fi lib init
  wifi_comm_config_t wcfg = {};
  wcfg.ssid = WIFI_SSID;
  wcfg.pass = WIFI_PASS;
  wcfg.auto_reconnect = true;
  wcfg.sleep_enable = false;
  (void)wifi_comm_init(&wcfg);

  // Sensors init
  sensors_config_t sensors_cfg;
  sensors_cfg.i2c_sda_pin       = i2c_sda_pin;
  sensors_cfg.i2c_scl_pin       = i2c_scl_pin;
  sensors_cfg.i2c_clock_hz      = i2c_clock_hz;
  sensors_cfg.adc_battery_pin   = (uint8_t)adc_battery_pin;
  sensors_cfg.battery_r1_ohm    = battery_r1_ohm;
  sensors_cfg.battery_r2_ohm    = battery_r2_ohm;
  sensors_cfg.adc_max_counts    = adc_max_counts;
  sensors_cfg.adc_full_scale_v  = adc_full_scale_v;
  sensors_cfg.sea_level_hpa     = sea_level_hpa;
  (void)sensors_init(&sensors_cfg);

  analogReadResolution(12);
  analogSetPinAttenuation((int)adc_battery_pin, ADC_11db);

  // Queues
  persister_data_queue            = xQueueCreate(queue_length_default, sizeof(sensor_frame_t));
  http_sensor_data_queue          = xQueueCreate(queue_unique_item, sizeof(sensor_frame_t));
  radio_sensor_data_queue         = xQueueCreate(queue_unique_item, sizeof(sensor_frame_t));

  lora_payload_data_queue         = xQueueCreate(queue_unique_item, sizeof(payload_msg_t));
  http_payload_data_queue         = xQueueCreate(queue_unique_item, sizeof(payload_msg_t));

  http_module_data_queue          = xQueueCreate(queue_unique_item, sizeof(lora_module_rx_t));
  lora_module_data_queue          = xQueueCreate(queue_unique_item, sizeof(lora_module_rx_t));
  persister_module_data_queue     = xQueueCreate(queue_length_default, sizeof(lora_module_rx_t));

  water_module_data_queue         = xQueueCreate(queue_unique_item, sizeof(water_module_rx_t));

  // LoRa init
  radio_lora_config_t radio_cfg;
  radio_cfg.sck_pin = lora_sck_pin;
  radio_cfg.rst_pin = lora_rst_pin;
  radio_cfg.miso_pin = lora_miso_pin;
  radio_cfg.mosi_pin = lora_mosi_pin;
  radio_cfg.cs_pin = lora_cs_pin;
  radio_cfg.dio0_pin = lora_dio0_pin;

  radio_cfg.spi_hz = lora_spi_hz;

  radio_cfg.frequency_hz = lora_frequency_hz;
  radio_cfg.bandwidth_hz = lora_bandwidth_hz;
  radio_cfg.spreading_factor = lora_spreading_factor;
  radio_cfg.coding_rate_denom = lora_coding_rate_denom;
  radio_cfg.preamble_len = lora_preamble_len;
  radio_cfg.sync_word = lora_sync_word;
  radio_cfg.tx_power_dbm = lora_tx_power_dbm;

  radio_cfg.crc_enable = true;

  if (!radio_lora_init(&radio_cfg))
  {
    const uint8_t ver = radio_lora_read_version_reg();
    Serial.printf("LoRa init fail (ver=0x%02X)\n", ver);
  }
  else
  {
    const uint8_t ver = radio_lora_read_version_reg();
    Serial.printf("LoRa OK (ver=0x%02X)\n", ver);
    Serial.printf("cfg: f=%ld bw=%ld sf=%u cr=4/%u sw=0x%02X spi=%ld\n",
                  lora_frequency_hz, lora_bandwidth_hz,
                  (unsigned)lora_spreading_factor, (unsigned)lora_coding_rate_denom,
                  (unsigned)lora_sync_word, lora_spi_hz);
  }

  // UART Rasp
  Serial1.begin(RASP_BAUD, SERIAL_8N1, RASP_UART_RX, RASP_UART_TX);
  Serial1.setTimeout(50);

  pinMode(RASP_HANDSHAKE_PIN, OUTPUT);
  digitalWrite(RASP_HANDSHAKE_PIN, LOW);
  while (Serial1.available() > 0) (void)Serial1.read();

  // rasp_comm library init
  rasp_comm_config_t rcfg = {};
  rcfg.uart_rx_pin = RASP_UART_RX;
  rcfg.uart_tx_pin = RASP_UART_TX;
  rcfg.baud = RASP_BAUD;

  rcfg.handshake_pin = RASP_HANDSHAKE_PIN;

  rcfg.awk_timeout_ms      = 1500;
  rcfg.mode_ack_timeout_ms = 1500;
  rcfg.pkt_ack_timeout_ms  = 800;
  rcfg.pkt_max_retries     = 3;

  rcfg.p_start_timeout_ms  = 1500;
  rcfg.p_data_timeout_ms   = 2000;
  rcfg.p_end_timeout_ms    = 1500;

  rcfg.x_ack_timeout_ms    = 1500;

  rcfg.max_items_per_session = 255;
  rcfg.debug_log = true;

  (void)rasp_comm_init(&rasp_comm, &rcfg, Serial1);
  rasp_comm_set_queues(&rasp_comm,
                       persister_data_queue,
                       persister_module_data_queue,
                       lora_payload_data_queue,
                       http_payload_data_queue);

  // Tasks
  xTaskCreate(sensor_reading_task, "sensor_reading_task", stack_medium, nullptr, prio_sensor_reading, nullptr);
  xTaskCreate(receive_radio_task,  "receive_radio_task",  stack_medium, nullptr, prio_receive_radio,  nullptr);
  // xTaskCreate(send_http_task,      "send_http_task",      stack_large,  nullptr,  prio_send_http,          nullptr);
  // xTaskCreate(wifi_monitor_task,   "wifi_monitor_task",   stack_small,  nullptr,  prio_wifi_monitor,       nullptr);
  xTaskCreate(rasp_communication_task, "rasp_communication_task", stack_large, &rasp_comm, prio_rasp_communication, nullptr);
  xTaskCreate(send_radio_task,     "send_radio_task",     stack_medium, nullptr, prio_send_radio,         nullptr);

  Serial.println("FreeRTOS firmware basico iniciado.");
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ===================== TASKS =====================
static void sensor_reading_task(void *pv_parameters)
{
  (void)pv_parameters;

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(10UL * 1000UL);

  for (;;)
  {
    sensor_frame_t frame = sensors_read_frame();

        Serial.printf(
      "[SENSOR] ts=%lu temp=%.2f pressao=%.2f altitude=%.2f bateria=%.2f\n",
      (unsigned long)frame.ts_ms,
      frame.bmp_temp_c,
      frame.pressure_pa,
      frame.altitude_m,
      frame.battery_v
    );

    (void)xQueueOverwrite(http_sensor_data_queue, &frame);
    (void)xQueueOverwrite(radio_sensor_data_queue, &frame);
    (void)xQueueSend(persister_data_queue, &frame, 0);

    vTaskDelayUntil(&last_wake_time, frequency);
  }
}

// static void receive_radio_task(void *pv_parameters)
// {
//   (void)pv_parameters;

//   TickType_t last_wake_time = xTaskGetTickCount();
//   const TickType_t frequency = pdMS_TO_TICKS(100UL);

//   for (;;)
//   {
//     lora_module_rx_t msg;
//     if (radio_lora_try_read_module_rx(&msg))
//     {
//       (void)xQueueOverwrite(lora_module_data_queue, &msg);
//       (void)xQueueOverwrite(http_module_data_queue, &msg);
//       (void)xQueueSend(persister_module_data_queue, &msg, 0);
//     }

//     vTaskDelayUntil(&last_wake_time, frequency);
//   }
// }

static void receive_radio_task(void *pv_parameters)
{
  (void)pv_parameters;

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(20UL);

  for (;;)
  {
    water_module_rx_t water;

    if (radio_lora_try_read_water_rx(&water))
    {
      (void)xQueueOverwrite(
        water_module_data_queue,
        &water
      );

      Serial.printf(
        "[WATER QUEUE] cond=%.2f uS/cm turb=%.2f NTU status=%u\n",
        water.conductivity_us_cm,
        water.turbidity_ntu,
        (unsigned)water.status
      );
    }

    vTaskDelayUntil(
      &last_wake_time,
      frequency
    );
  }
}

static void send_http_task(void *pv_parameters)
{
  (void)pv_parameters;

  obsat_http_config_t hcfg = {};
  hcfg.url = OBSAT_HTTP_URL;
  hcfg.equipe = 60;
  hcfg.tls_insecure = true;
  hcfg.http_timeout_ms = 8000;

  sensor_frame_t   s_cache = {};
  payload_msg_t    p_cache = {};
  lora_module_rx_t m_cache = {};

  vTaskDelay(pdMS_TO_TICKS(15000UL));

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(4UL * 60UL * 1000UL);

  for (;;)
  {
    {
      const uint32_t start_wait = millis();
      while (rasp_session_active || (millis() - (uint32_t)rasp_last_end_ms) < 5000U)
      {
        vTaskDelay(pdMS_TO_TICKS(100));
        if ((millis() - start_wait) > 10000U) break;
      }
      if (rasp_session_active || (millis() - (uint32_t)rasp_last_end_ms) < 5000U)
      {
        Serial.println("[HTTP] Skipping cycle: rasp session too close/active.");
        vTaskDelayUntil(&last_wake_time, frequency);
        continue;
      }
    }

    if (!wifi_comm_try_lock(2000))
    {
      Serial.println("[HTTP] wifi_mutex busy, skipping.");
      vTaskDelayUntil(&last_wake_time, frequency);
      continue;
    }

    // Garante Wi-Fi conectado
    if (!wifi_comm_is_connected())
    {
      wifi_comm_kick_connect_nonblocking();
      (void)wifi_comm_wait_connected(3000);
    }

    if (!wifi_comm_is_connected())
    {
      wifi_comm_update_bits();
      Serial.printf("[HTTP] No Wi-Fi (status=%d). Abort.\n", (int)WiFi.status());
      wifi_comm_unlock();
      vTaskDelayUntil(&last_wake_time, frequency);
      continue;
    }

    // Pega amostras
    sensor_frame_t s_now;
    if (http_sensor_data_queue && xQueuePeek(http_sensor_data_queue, &s_now, 0) == pdTRUE) s_cache = s_now;

    payload_msg_t p_now;
    if (http_payload_data_queue && xQueuePeek(http_payload_data_queue, &p_now, 0) == pdTRUE) p_cache = p_now;

    lora_module_rx_t m_now;
    if (http_module_data_queue && xQueuePeek(http_module_data_queue, &m_now, 0) == pdTRUE) m_cache = m_now;

    char payload_buf[96];
    char json[320];

    const int json_len = obsat_http_format_json(
      json, sizeof(json),
      payload_buf, sizeof(payload_buf),
      &hcfg,
      &s_cache, &p_cache, &m_cache
    );

    if (json_len < 0)
    {
      Serial.printf("[HTTP] JSON format error: %d\n", json_len);
      wifi_comm_unlock();
      vTaskDelayUntil(&last_wake_time, frequency);
      continue;
    }

    char resp[160];
    const int code = obsat_http_post_json(&hcfg, json, (size_t)json_len, resp, sizeof(resp));

    if (code > 0)
    {
      Serial.printf("[HTTP] code=%d sent=%dB payloadLen=%u respLen=%u\n",
                    code, json_len, (unsigned)strlen(payload_buf), (unsigned)strlen(resp));
      if (resp[0] != '\0') Serial.printf("[HTTP] resp: %s\n", resp);
    }
    else
    {
      Serial.printf("[HTTP] POST failed: %d\n", code);
    }

    wifi_comm_unlock();
    vTaskDelayUntil(&last_wake_time, frequency);
  }
}

static void wifi_monitor_task(void *pv_parameters)
{
  (void)pv_parameters;

  vTaskDelay(pdMS_TO_TICKS(8000));

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(1000UL);

  bool last_connected = false;

  uint32_t next_try_ms = 0;
  uint32_t backoff_ms = 2000;
  const uint32_t backoff_max_ms = 30000;

  for (;;)
  {
    const bool connected = wifi_comm_is_connected();
    wifi_comm_update_bits();

    if (connected != last_connected)
    {
      last_connected = connected;
      if (connected)
      {
        Serial.printf("[WIFI] Connected. IP=%s RSSI=%ld dBm\n",
                      WiFi.localIP().toString().c_str(),
                      (long)WiFi.RSSI());
        backoff_ms = 2000;
        next_try_ms = 0;
      }
      else
      {
        Serial.printf("[WIFI] Disconnected. status=%d\n", (int)WiFi.status());
        next_try_ms = millis();
      }
    }

    if (!connected)
    {
      const uint32_t now = millis();
      if (!rasp_session_active && (next_try_ms == 0 || now >= next_try_ms))
      {
        if (wifi_comm_try_lock(0))
        {
          wifi_comm_kick_connect_nonblocking();
          wifi_comm_unlock();

          next_try_ms = now + backoff_ms;
          if (backoff_ms < backoff_max_ms)
          {
            backoff_ms *= 2;
            if (backoff_ms > backoff_max_ms) backoff_ms = backoff_max_ms;
          }
        }
        else
        {
          next_try_ms = now + 1000;
        }
      }
    }

    vTaskDelayUntil(&last_wake_time, frequency);
  }
}

static void rasp_communication_task(void *pv_parameters)
{
  auto *comm = (rasp_comm_t*)pv_parameters;

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(60UL * 1000UL);

  for (;;)
  {
    rasp_session_active = true;
    (void)rasp_comm_run_once(comm);
    rasp_session_active = false;
    rasp_last_end_ms = millis();

    vTaskDelayUntil(&last_wake_time, frequency);
  }
}

static void send_radio_task(void *pv_parameters)
{
  (void)pv_parameters;

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(10UL * 1000UL);

  for (;;)
  {
    uint8_t pkt[128];
    size_t pkt_len = 0;

    if (gs_packet_build_from_queues(
          radio_sensor_data_queue,
          lora_module_data_queue,
          lora_payload_data_queue,
          water_module_data_queue,
          pkt, sizeof(pkt), &pkt_len))
    {
      const bool ok = radio_lora_send_bytes(pkt, pkt_len);
      Serial.printf("LoRa TX: len=%u ok=%u flags=0x%02X\n",
                    (unsigned)pkt_len,
                    (unsigned)(ok ? 1 : 0),
                    (unsigned)gs_packet_get_flags(pkt, pkt_len));
    }

    vTaskDelayUntil(&last_wake_time, frequency);
  }
}
