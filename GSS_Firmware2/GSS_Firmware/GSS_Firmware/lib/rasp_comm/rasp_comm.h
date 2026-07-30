#pragma once

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sensors.h"
#include "radio_lora.h"

typedef struct
{
  uint32_t ts_ms;
  float    lat;
  float    lon;
  uint8_t  found;   // 0 ou 1
  uint8_t  _rsv[3];
} payload_msg_t;

typedef struct
{
  int uart_rx_pin;
  int uart_tx_pin;
  uint32_t baud;

  int handshake_pin;
  uint32_t awk_timeout_ms;
  uint32_t mode_ack_timeout_ms;
  uint32_t pkt_ack_timeout_ms;
  uint8_t  pkt_max_retries;

  uint32_t p_start_timeout_ms;
  uint32_t p_data_timeout_ms;
  uint32_t p_end_timeout_ms;

  uint32_t x_ack_timeout_ms;

  uint8_t  max_items_per_session;
  bool     debug_log;
} rasp_comm_config_t;

static constexpr size_t RASP_COMM_UART_LINE_MAX = 520;

typedef struct
{
  // config + estado
  rasp_comm_config_t cfg;
  HardwareSerial *uart;
  bool initialized;

  // filas
  QueueHandle_t persister_data_queue;        // sensor_frame_t
  QueueHandle_t persister_module_data_queue; // lora_module_rx_t
  QueueHandle_t lora_payload_data_queue;     // payload_msg_t
  QueueHandle_t http_payload_data_queue;     // payload_msg_t

  // buffers (fora da stack da task)
  const size_t UART_LINE_MAX = RASP_COMM_UART_LINE_MAX;   // 512 + margem
  char line_tmp[RASP_COMM_UART_LINE_MAX];
  char payload_json[RASP_COMM_UART_LINE_MAX];
  char json_buf[513];                        // <= 512 + '\0'

  // leitor de linha
  char rx_buf[RASP_COMM_UART_LINE_MAX];
  size_t rx_len;

  // sequências
  uint32_t seq_t;
  uint32_t seq_m;
} rasp_comm_t;

bool rasp_comm_init(rasp_comm_t *h, const rasp_comm_config_t *cfg, HardwareSerial &uart);
void rasp_comm_set_queues(rasp_comm_t *h,
                          QueueHandle_t persister_data_queue,
                          QueueHandle_t persister_module_data_queue,
                          QueueHandle_t lora_payload_data_queue,
                          QueueHandle_t http_payload_data_queue);

// Executa 1 sessão completa (se houver dados nas filas).
// Retorna:
// - true  => sessão executada com sucesso
// - false => sessão abortada por timeout/erro OU não havia nada para enviar
bool rasp_comm_run_once(rasp_comm_t *h);
