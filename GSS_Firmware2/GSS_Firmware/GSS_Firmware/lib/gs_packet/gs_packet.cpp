#include "gs_packet.h"

#include <string.h>

#include "sensors.h"     // sensor_frame_t
#include "radio_lora.h"  // lora_module_rx_t
#include "rasp_comm.h"   // payload_msg_t

#define GS_WATER_BLOCK_LEN 24

static inline bool buf_put(uint8_t* buf, size_t buf_max, size_t* idx, const void* src, size_t n)
{
  if (!buf || !idx || !src) return false;
  if ((*idx + n) > buf_max) return false;
  memcpy(buf + *idx, src, n);
  *idx += n;
  return true;
}

static inline bool buf_put_u8(uint8_t* buf, size_t buf_max, size_t* idx, uint8_t v)
{
  return buf_put(buf, buf_max, idx, &v, sizeof(v));
}

static inline bool buf_put_u32(uint8_t* buf, size_t buf_max, size_t* idx, uint32_t v)
{
  return buf_put(buf, buf_max, idx, &v, sizeof(v));
}

static inline bool buf_put_f32(uint8_t* buf, size_t buf_max, size_t* idx, float v)
{
  return buf_put(buf, buf_max, idx, &v, sizeof(v));
}

bool gs_packet_is_gs(const uint8_t* data, size_t len)
{
  if (!data || len < GS_HDR_LEN) return false;
  if (data[0] != GS_MAGIC0 || data[1] != GS_MAGIC1) return false;
  if (data[2] != GS_VERSION) return false;
  return true;
}

uint8_t gs_packet_get_flags(const uint8_t* data, size_t len)
{
  if (!data || len < GS_HDR_LEN) return 0;
  return data[3];
}

bool gs_packet_build_from_queues(
  QueueHandle_t sensor_q,
  QueueHandle_t module_q,
  QueueHandle_t payload_q,
  QueueHandle_t water_q,
  uint8_t* out,
  size_t out_max,
  size_t* out_len
)
{
  if (!out || !out_len) return false;
  *out_len = 0;

  if (!sensor_q) return false;

  // Sensor obrigatório
  sensor_frame_t sframe{};
  if (xQueuePeek(sensor_q, &sframe, 0) != pdTRUE)
  {
    return false; // sem telemetria ainda
  }

  // Opcionais
  lora_module_rx_t mframe{};
  const bool has_module = (module_q && (xQueuePeek(module_q, &mframe, 0) == pdTRUE));

  payload_msg_t pframe{};
  const bool has_payload = (payload_q && (xQueuePeek(payload_q, &pframe, 0) == pdTRUE));

  water_module_rx_t water_frame{};

  const bool has_water =
    water_q &&
    (xQueuePeek(water_q, &water_frame, 0) == pdTRUE);

  uint8_t flags = 0;
  flags |= GS_FLAG_SENSOR;
  if (has_module)  flags |= GS_FLAG_MODULE;
  if (has_payload) flags |= GS_FLAG_PAYLOAD;
  if (has_water) flags |= GS_FLAG_WATER;

  // Calcula tamanho requerido para evitar overflow
  size_t needed = GS_MIN_LEN;
  if (has_module)  needed += GS_OPT_BLOCK_LEN;
  if (has_payload) needed += GS_OPT_BLOCK_LEN;
  if (has_water) needed += GS_OPT_BLOCK_LEN;

  if (has_water)
  {
    needed += GS_WATER_BLOCK_LEN;
  }

  if (out_max < needed) return false;

  size_t idx = 0;

  // Header
  const uint8_t magic[2] = {GS_MAGIC0, GS_MAGIC1};
  const uint8_t version  = GS_VERSION;
  const uint32_t tx_ts   = millis();

  if (!buf_put(out, out_max, &idx, magic, sizeof(magic))) return false;
  if (!buf_put_u8(out, out_max, &idx, version)) return false;
  if (!buf_put_u8(out, out_max, &idx, flags)) return false;
  if (!buf_put_u32(out, out_max, &idx, tx_ts)) return false;

  // Sensor block: ts + 13 floats
  if (!buf_put_u32(out, out_max, &idx, sframe.ts_ms)) return false;

  if (!buf_put_f32(out, out_max, &idx, sframe.accel_m_s2_x)) return false;
  if (!buf_put_f32(out, out_max, &idx, sframe.accel_m_s2_y)) return false;
  if (!buf_put_f32(out, out_max, &idx, sframe.accel_m_s2_z)) return false;

  if (!buf_put_f32(out, out_max, &idx, sframe.gyro_rad_s_x)) return false;
  if (!buf_put_f32(out, out_max, &idx, sframe.gyro_rad_s_y)) return false;
  if (!buf_put_f32(out, out_max, &idx, sframe.gyro_rad_s_z)) return false;

  if (!buf_put_f32(out, out_max, &idx, sframe.mag_uT_x)) return false;
  if (!buf_put_f32(out, out_max, &idx, sframe.mag_uT_y)) return false;
  if (!buf_put_f32(out, out_max, &idx, sframe.mag_uT_z)) return false;

  if (!buf_put_f32(out, out_max, &idx, sframe.bmp_temp_c)) return false;
  if (!buf_put_f32(out, out_max, &idx, sframe.pressure_pa)) return false;
  if (!buf_put_f32(out, out_max, &idx, sframe.altitude_m)) return false;

  if (!buf_put_f32(out, out_max, &idx, sframe.battery_v)) return false;

  // Module block
  if (has_module)
  {
    const uint32_t mstatus = (mframe.s != 0) ? 1u : 0u;

    if (!buf_put_u32(out, out_max, &idx, mframe.ts_ms)) return false;
    if (!buf_put_f32(out, out_max, &idx, mframe.lat)) return false;
    if (!buf_put_f32(out, out_max, &idx, mframe.lon)) return false;
    if (!buf_put_u32(out, out_max, &idx, mstatus)) return false;
  }

  // Payload block
  if (has_payload)
  {
    const uint32_t pstatus = (pframe.found != 0) ? 1u : 0u;

    if (!buf_put_u32(out, out_max, &idx, pframe.ts_ms)) return false;
    if (!buf_put_f32(out, out_max, &idx, pframe.lat)) return false;
    if (!buf_put_f32(out, out_max, &idx, pframe.lon)) return false;
    if (!buf_put_u32(out, out_max, &idx, pstatus)) return false;
  }

  // Water block
  if (has_water)
  {
    const uint32_t water_status =
      water_frame.status != 0 ? 1u : 0u;

    if (!buf_put_u32(
          out,
          out_max,
          &idx,
          water_frame.ts_ms))
    {
      return false;
    }

    if (!buf_put_f32(
          out,
          out_max,
          &idx,
          water_frame.conductivity_us_cm))
    {
      return false;
    }

    if (!buf_put_f32(
          out,
          out_max,
          &idx,
          water_frame.turbidity_ntu))
    {
      return false;
    }

    if (!buf_put_f32(
          out,
          out_max,
          &idx,
          water_frame.ph))
    {
      return false;
    }

    if (!buf_put_f32(
          out,
          out_max,
          &idx,
          water_frame.temperature_c))
    {
      return false;
    }

    if (!buf_put_u32(
          out,
          out_max,
          &idx,
          water_status))
    {
      return false;
    }
  }

  // Sanity: idx deve bater com needed
  *out_len = idx;
  return true;
}
