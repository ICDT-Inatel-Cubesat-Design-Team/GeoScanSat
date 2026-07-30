#pragma once

#include <Arduino.h>

#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include "sensors.h"
#include "radio_lora.h"
#include "rasp_comm.h"

typedef struct
{
  const char *url;
  uint16_t equipe;
  bool tls_insecure;         // true para teste (setInsecure)
  uint32_t http_timeout_ms;
} obsat_http_config_t;

// Retorna: tamanho do JSON (>=1) ou negativo em erro.
int obsat_http_format_json(char *json_out, size_t json_cap,
                           char *payload_out, size_t payload_cap,
                           const obsat_http_config_t *cfg,
                           const sensor_frame_t *s,
                           const payload_msg_t *p,
                           const lora_module_rx_t *m);

// Envia POST application/json.
// Retorna: HTTP status code (>0) ou negativo em erro.
int obsat_http_post_json(const obsat_http_config_t *cfg,
                         const char *json, size_t json_len,
                         char *resp_out, size_t resp_cap);
