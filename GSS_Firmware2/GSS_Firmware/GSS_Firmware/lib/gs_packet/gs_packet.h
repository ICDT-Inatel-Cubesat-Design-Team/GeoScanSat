#pragma once

#include <Arduino.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Formato do pacote GS
static constexpr uint8_t GS_MAGIC0  = 'G';
static constexpr uint8_t GS_MAGIC1  = 'S';
static constexpr uint8_t GS_VERSION = 1;

// Flags
static constexpr uint8_t GS_FLAG_SENSOR  = 0x01;
static constexpr uint8_t GS_FLAG_MODULE  = 0x02;
static constexpr uint8_t GS_FLAG_PAYLOAD = 0x04;
static constexpr uint8_t GS_FLAG_WATER   = 0x08;

// Tamanhos
static constexpr size_t GS_HDR_LEN = 8;

// ts(u32) + 13 floats
static constexpr size_t GS_SENSOR_BLOCK_LEN = 56;

// Cada bloco opcional possui:
// ts(u32) + valor1(f32) + valor2(f32) + status(u32)
static constexpr size_t GS_OPT_BLOCK_LEN = 16;

static constexpr size_t GS_MIN_LEN =
  GS_HDR_LEN + GS_SENSOR_BLOCK_LEN; // 64

static constexpr size_t GS_MAX_LEN =
  GS_MIN_LEN + (3 * GS_OPT_BLOCK_LEN); // 112

bool gs_packet_build_from_queues(
  QueueHandle_t sensor_q,
  QueueHandle_t module_q,
  QueueHandle_t payload_q,
  QueueHandle_t water_q,
  uint8_t* out,
  size_t out_max,
  size_t* out_len
);

// Helpers
bool gs_packet_is_gs(const uint8_t* data, size_t len);

uint8_t gs_packet_get_flags(
  const uint8_t* data,
  size_t len
);
