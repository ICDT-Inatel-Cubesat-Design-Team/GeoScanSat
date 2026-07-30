#include "rasp_comm.h"

#include <string.h>
#include <stdlib.h>

// Helpers internos
static inline void rlog(rasp_comm_t *h, const char *msg)
{
  if (h->cfg.debug_log) Serial.println(msg);
}

static void rlogf(rasp_comm_t *h, const char *fmt, ...)
{
  if (!h->cfg.debug_log) return;

  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  Serial.print(buf);
}

static void drain_uart(HardwareSerial &s, uint32_t ms)
{
  const uint32_t t0 = millis();
  while ((millis() - t0) < ms)
  {
    while (s.available() > 0) (void)s.read();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void uart_trim(char *s)
{
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) { s[n - 1] = '\0'; n--; }
  size_t i = 0;
  while (s[i] == ' ' || s[i] == '\t') i++;
  if (i > 0) memmove(s, s + i, strlen(s + i) + 1);
}

static bool uart_read_line_nonblocking(rasp_comm_t *h, char *out, size_t out_sz)
{
  HardwareSerial &s = *h->uart;

  while (s.available() > 0)
  {
    char c = (char)s.read();
    if (c == '\r') continue;

    if (c == '\n')
    {
      h->rx_buf[h->rx_len] = '\0';
      uart_trim(h->rx_buf);

      if (h->rx_buf[0] != '\0')
      {
        strncpy(out, h->rx_buf, out_sz - 1);
        out[out_sz - 1] = '\0';
        h->rx_len = 0;
        return true;
      }

      h->rx_len = 0;
      continue;
    }

    if (c >= 32 && c <= 126)
    {
      if (h->rx_len + 1 < sizeof(h->rx_buf))
      {
        h->rx_buf[h->rx_len++] = c;
      }
      else
      {
        // overflow -> reseta
        h->rx_len = 0;
      }
    }
  }

  return false;
}

static void uart_send_line(HardwareSerial &s, const char *msg)
{
  s.print(msg);
  s.print('\n');
  s.flush();
}

static bool uart_wait_token(rasp_comm_t *h, const char *token, uint32_t timeout_ms)
{
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

  while (xTaskGetTickCount() < deadline)
  {
    if (uart_read_line_nonblocking(h, h->line_tmp, sizeof(h->line_tmp)))
    {
      if (h->cfg.debug_log) {
        rlogf(h, "[RASP] RX='%s'\n", h->line_tmp);
      }
      if (strcmp(h->line_tmp, token) == 0) return true;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return false;
}

static bool uart_wait_payload_json(rasp_comm_t *h, uint32_t timeout_ms)
{
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

  while (xTaskGetTickCount() < deadline)
  {
    if (uart_read_line_nonblocking(h, h->payload_json, sizeof(h->payload_json)))
    {
      if (h->cfg.debug_log) {
        rlogf(h, "[RASP] RX='%s'\n", h->payload_json);
      }
      if (h->payload_json[0] == '{') return true;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return false;
}

static bool send_and_wait_ack(rasp_comm_t *h, const char *msg, uint32_t timeout_ms)
{
  uart_send_line(*h->uart, msg);
  return uart_wait_token(h, "ACK", timeout_ms);
}

// JSON / Payload helpers
static bool make_json_hex(const char *src, uint32_t seq, uint32_t ts_ms,
                          const void *data, size_t data_len,
                          char *out, size_t out_sz)
{
  const uint8_t *b = (const uint8_t*)data;
  static const char HEX_DIGITS[] = "0123456789ABCDEF";

  int n = snprintf(out, out_sz, "{\"src\":\"%s\",\"seq\":%lu,\"ts_ms\":%lu,\"hex\":\"",
                   src, (unsigned long)seq, (unsigned long)ts_ms);
  if (n < 0 || (size_t)n >= out_sz) return false;

  size_t pos = (size_t)n;
  size_t room = out_sz - pos - 3;   // "\"}" + '\0'
  size_t max_bytes = room / 2;

  if (data_len > max_bytes)
  {
    snprintf(out, out_sz,
             "{\"src\":\"%s\",\"seq\":%lu,\"ts_ms\":%lu,\"err\":\"blob_too_large\",\"len\":%lu}",
             src, (unsigned long)seq, (unsigned long)ts_ms, (unsigned long)data_len);
    return true;
  }

  for (size_t i = 0; i < data_len; i++)
  {
    uint8_t v = b[i];
    out[pos++] = HEX_DIGITS[(v >> 4) & 0x0F];
    out[pos++] = HEX_DIGITS[v & 0x0F];
  }

  out[pos++] = '"';
  out[pos++] = '}';
  out[pos] = '\0';
  return true;
}

static bool parse_float_field(const char *json, const char *key, float *out)
{
  char pattern[32];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char *p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  *out = strtof(p, nullptr);
  return true;
}

static bool parse_bool_field(const char *json, const char *key, bool *out)
{
  char pattern[32];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char *p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;

  if (strncmp(p, "true", 4) == 0)  { *out = true;  return true; }
  if (strncmp(p, "false", 5) == 0) { *out = false; return true; }
  if (*p == '0') { *out = false; return true; }
  if (*p == '1') { *out = true;  return true; }
  return false;
}

static bool parse_u32_field(const char *json, const char *key, uint32_t *out)
{
  char pattern[32];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char *p = strstr(json, pattern);
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  p++;

  while (*p == ' ' || *p == '\t') p++;

  unsigned long v = strtoul(p, nullptr, 10);
  *out = (uint32_t)v;
  return true;
}

// Envio de itens das filas
static bool send_sensor_queue_items(rasp_comm_t *h)
{
  QueueHandle_t q = h->persister_data_queue;

  for (uint8_t i = 0; i < h->cfg.max_items_per_session; i++)
  {
    if (uxQueueMessagesWaiting(q) == 0) break;

    sensor_frame_t item;
    if (xQueuePeek(q, &item, 0) != pdTRUE) break;

    h->seq_t++;
    if (!make_json_hex("T", h->seq_t, (uint32_t)millis(), &item, sizeof(item), h->json_buf, sizeof(h->json_buf)))
    {
      snprintf(h->json_buf, sizeof(h->json_buf), "{\"src\":\"T\",\"seq\":%lu,\"err\":\"encode_fail\"}",
               (unsigned long)h->seq_t);
    }

    bool ok = false;
    for (uint8_t tr = 0; tr <= h->cfg.pkt_max_retries; tr++)
    {
      uart_send_line(*h->uart, h->json_buf);
      if (uart_wait_token(h, "ACK", h->cfg.pkt_ack_timeout_ms)) { ok = true; break; }
    }
    if (!ok) return false;

    sensor_frame_t drop;
    (void)xQueueReceive(q, &drop, 0);
  }
  return true;
}

static bool send_module_queue_items(rasp_comm_t *h)
{
  QueueHandle_t q = h->persister_module_data_queue;

  for (uint8_t i = 0; i < h->cfg.max_items_per_session; i++)
  {
    if (uxQueueMessagesWaiting(q) == 0) break;

    lora_module_rx_t item;
    if (xQueuePeek(q, &item, 0) != pdTRUE) break;

    h->seq_m++;
    if (!make_json_hex("M", h->seq_m, (uint32_t)millis(), &item, sizeof(item), h->json_buf, sizeof(h->json_buf)))
    {
      snprintf(h->json_buf, sizeof(h->json_buf), "{\"src\":\"M\",\"seq\":%lu,\"err\":\"encode_fail\"}",
               (unsigned long)h->seq_m);
    }

    bool ok = false;
    for (uint8_t tr = 0; tr <= h->cfg.pkt_max_retries; tr++)
    {
      uart_send_line(*h->uart, h->json_buf);
      if (uart_wait_token(h, "ACK", h->cfg.pkt_ack_timeout_ms)) { ok = true; break; }
    }
    if (!ok) return false;

    lora_module_rx_t drop;
    (void)xQueueReceive(q, &drop, 0);
  }
  return true;
}

// API pública
bool rasp_comm_init(rasp_comm_t *h, const rasp_comm_config_t *cfg, HardwareSerial &uart)
{
  if (!h || !cfg) return false;

  memset(h, 0, sizeof(*h));
  h->cfg = *cfg;
  h->uart = &uart;

  // UART e handshake
  uart.begin(h->cfg.baud, SERIAL_8N1, h->cfg.uart_rx_pin, h->cfg.uart_tx_pin);
  uart.setTimeout(50);

  pinMode(h->cfg.handshake_pin, OUTPUT);
  digitalWrite(h->cfg.handshake_pin, LOW);

  drain_uart(uart, 30);

  h->rx_len = 0;
  h->seq_t = 0;
  h->seq_m = 0;
  h->initialized = true;

  rlog(h, "rasp_comm_init: OK");
  return true;
}

void rasp_comm_set_queues(rasp_comm_t *h,
                          QueueHandle_t persister_data_queue,
                          QueueHandle_t persister_module_data_queue,
                          QueueHandle_t lora_payload_data_queue,
                          QueueHandle_t http_payload_data_queue)
{
  if (!h) return;
  h->persister_data_queue = persister_data_queue;
  h->persister_module_data_queue = persister_module_data_queue;
  h->lora_payload_data_queue = lora_payload_data_queue;
  h->http_payload_data_queue = http_payload_data_queue;
}

bool rasp_comm_run_once(rasp_comm_t *h)
{
  if (!h || !h->initialized || !h->uart) return false;

  // nada para enviar? não abre sessão
  if (uxQueueMessagesWaiting(h->persister_data_queue) == 0 &&
      uxQueueMessagesWaiting(h->persister_module_data_queue) == 0)
  {
    return false;
  }

  // 1) abre sessão
  digitalWrite(h->cfg.handshake_pin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));

  // drena no início
  h->rx_len = 0;
  drain_uart(*h->uart, 10);

  // 2) espera AWK
  if (!uart_wait_token(h, "AWK", h->cfg.awk_timeout_ms))
  {
    rlog(h, "[RASP] Timeout esperando AWK");
    digitalWrite(h->cfg.handshake_pin, LOW);
    return false;
  }

  // 3) responde ACK (não espera ACK de volta)
  uart_send_line(*h->uart, "ACK");
  vTaskDelay(pdMS_TO_TICKS(5));

  // 4) modo T
  if (!send_and_wait_ack(h, "+T", h->cfg.mode_ack_timeout_ms))
  {
    rlog(h, "[RASP] Timeout ACK +T");
    digitalWrite(h->cfg.handshake_pin, LOW);
    return false;
  }

  if (!send_sensor_queue_items(h))
  {
    rlog(h, "[RASP] Falha enviando itens T");
    digitalWrite(h->cfg.handshake_pin, LOW);
    return false;
  }

  if (!send_and_wait_ack(h, "-T", h->cfg.mode_ack_timeout_ms))
  {
    rlog(h, "[RASP] Timeout ACK -T");
    digitalWrite(h->cfg.handshake_pin, LOW);
    return false;
  }

  // 5) modo M (se houver)
  if (uxQueueMessagesWaiting(h->persister_module_data_queue) > 0)
  {
    if (!send_and_wait_ack(h, "+M", h->cfg.mode_ack_timeout_ms))
    {
      rlog(h, "[RASP] Timeout ACK +M");
      digitalWrite(h->cfg.handshake_pin, LOW);
      return false;
    }

    if (!send_module_queue_items(h))
    {
      rlog(h, "[RASP] Falha enviando itens M");
      digitalWrite(h->cfg.handshake_pin, LOW);
      return false;
    }

    if (!send_and_wait_ack(h, "-M", h->cfg.mode_ack_timeout_ms))
    {
      rlog(h, "[RASP] Timeout ACK -M");
      digitalWrite(h->cfg.handshake_pin, LOW);
      return false;
    }
  }

  // 6) payload
  if (!send_and_wait_ack(h, "PAY", h->cfg.mode_ack_timeout_ms))
  {
    rlog(h, "[RASP] Timeout ACK PAY");
    digitalWrite(h->cfg.handshake_pin, LOW);
    return false;
  }

  if (!uart_wait_token(h, "+P", h->cfg.p_start_timeout_ms))
  {
    rlog(h, "[RASP] Timeout esperando +P");
    digitalWrite(h->cfg.handshake_pin, LOW);
    return false;
  }
  uart_send_line(*h->uart, "ACK");

  if (!uart_wait_payload_json(h, h->cfg.p_data_timeout_ms))
  {
    rlog(h, "[RASP] Timeout esperando payload JSON");
    digitalWrite(h->cfg.handshake_pin, LOW);
    return false;
  }
  uart_send_line(*h->uart, "ACK");

  // parse payload
  float lat = 0.0f, lon = 0.0f;
  bool found = false;

  bool ok_lat = parse_float_field(h->payload_json, "lat", &lat) || parse_float_field(h->payload_json, "latitude", &lat);
  bool ok_lon = parse_float_field(h->payload_json, "lon", &lon) || parse_float_field(h->payload_json, "longitude", &lon);
  bool ok_fnd = parse_bool_field(h->payload_json, "found", &found) || parse_bool_field(h->payload_json, "encontrou", &found);

  if (ok_lat && ok_lon && ok_fnd)
  {
    payload_msg_t pm = {};

    uint32_t ts_payload = (uint32_t)millis();
    (void)parse_u32_field(h->payload_json, "ts_ms", &ts_payload);
    (void)parse_u32_field(h->payload_json, "ts",    &ts_payload);

    pm.ts_ms = ts_payload;
    pm.lat   = lat;
    pm.lon   = lon;
    pm.found = found ? 1 : 0;

    (void)xQueueOverwrite(h->lora_payload_data_queue, &pm);
    (void)xQueueOverwrite(h->http_payload_data_queue, &pm);

    rlogf(h, "[RASP] Payload OK ts=%lu lat=%.6f lon=%.6f found=%u\n",
          (unsigned long)pm.ts_ms, pm.lat, pm.lon, (unsigned)pm.found);
  }
  else
  {
    rlogf(h, "[RASP] Payload parse FAIL json='%s'\n", h->payload_json);
  }

  if (!uart_wait_token(h, "-P", h->cfg.p_end_timeout_ms))
  {
    rlog(h, "[RASP] Timeout esperando -P");
    digitalWrite(h->cfg.handshake_pin, LOW);
    return false;
  }
  uart_send_line(*h->uart, "ACK");

  // 7) finalização X
  if (!send_and_wait_ack(h, "X", h->cfg.x_ack_timeout_ms))
  {
    rlog(h, "[RASP] Timeout ACK X");
    digitalWrite(h->cfg.handshake_pin, LOW);
    return false;
  }

  digitalWrite(h->cfg.handshake_pin, LOW);
  return true;
}
