#pragma once

#include <Arduino.h>

#include <WiFi.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

typedef struct
{
  const char *ssid;
  const char *pass;

  bool auto_reconnect; // true recomendado
  bool sleep_enable;   // false recomendado (menor latência)
} wifi_comm_config_t;

static const EventBits_t WIFI_COMM_CONNECTED_BIT = BIT0;

bool wifi_comm_init(const wifi_comm_config_t *cfg);

// Estado
bool wifi_comm_is_connected();
void wifi_comm_update_bits(); // atualiza WIFI_COMM_CONNECTED_BIT baseado em WiFi.status()

// Conexão (não bloqueante / bloqueio curto)
void wifi_comm_kick_connect_nonblocking();
bool wifi_comm_wait_connected(uint32_t timeout_ms);

// Sincronização
SemaphoreHandle_t wifi_comm_mutex();
EventGroupHandle_t wifi_comm_event_group();

bool wifi_comm_try_lock(uint32_t timeout_ms);
void wifi_comm_unlock();
