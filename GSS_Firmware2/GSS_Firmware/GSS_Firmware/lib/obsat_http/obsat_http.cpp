#include "obsat_http.h"

static inline int16_t clamp_i16(int32_t v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static inline uint16_t clamp_u16(int32_t v)
{
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return (uint16_t)v;
}

static inline int32_t round_i32(float x)
{
  return (int32_t)lroundf(x);
}

static inline int32_t pack_deg_1e5(float deg)
{
  return round_i32(deg * 100000.0f);
}

static bool build_payload_string(char *out, size_t out_cap,
                                 const sensor_frame_t &s,
                                 const payload_msg_t &p,
                                 const lora_module_rx_t &m)
{
  static const size_t PAYLOAD_MAX = 90;

  const int32_t ts_s     = (int32_t)(s.ts_ms / 1000UL);
  const int32_t alt_dm   = round_i32(s.altitude_m * 10.0f);
  const int16_t mx_duT   = clamp_i16(round_i32(s.mag_uT_x * 10.0f));
  const int16_t my_duT   = clamp_i16(round_i32(s.mag_uT_y * 10.0f));
  const int16_t mz_duT   = clamp_i16(round_i32(s.mag_uT_z * 10.0f));

  const int32_t plat_1e5 = pack_deg_1e5(p.lat);
  const int32_t plon_1e5 = pack_deg_1e5(p.lon);
  const uint8_t pfound   = p.found ? 1U : 0U;

  const int32_t mlat_1e5 = pack_deg_1e5(m.lat);
  const int32_t mlon_1e5 = pack_deg_1e5(m.lon);
  const uint8_t ms       = m.s;

  // completo
  int n = snprintf(out, out_cap,
                   "[%ld,%ld,%d,%d,%d,%ld,%ld,%u,%ld,%ld,%u]",
                   (long)ts_s, (long)alt_dm,
                   (int)mx_duT, (int)my_duT, (int)mz_duT,
                   (long)plat_1e5, (long)plon_1e5, (unsigned)pfound,
                   (long)mlat_1e5, (long)mlon_1e5, (unsigned)ms);
  if (n > 0 && (size_t)n <= PAYLOAD_MAX) return true;

  // médio
  n = snprintf(out, out_cap,
               "[%ld,%ld,%d,%d,%d,%ld,%ld,%u]",
               (long)ts_s, (long)alt_dm,
               (int)mx_duT, (int)my_duT, (int)mz_duT,
               (long)plat_1e5, (long)plon_1e5, (unsigned)pfound);
  if (n > 0 && (size_t)n <= PAYLOAD_MAX) return true;

  // mínimo útil
  n = snprintf(out, out_cap,
               "[%ld,%ld,%ld,%u]",
               (long)ts_s, (long)plat_1e5, (long)plon_1e5, (unsigned)pfound);
  if (n > 0 && (size_t)n <= PAYLOAD_MAX) return true;

  // fallback final
  n = snprintf(out, out_cap, "[%ld,%u]", (long)ts_s, (unsigned)pfound);
  if (n > 0 && (size_t)n <= PAYLOAD_MAX) return true;

  strncpy(out, "[]", out_cap);
  out[out_cap - 1] = '\0';
  return false;
}

int obsat_http_format_json(char *json_out, size_t json_cap,
                           char *payload_out, size_t payload_cap,
                           const obsat_http_config_t *cfg,
                           const sensor_frame_t *s,
                           const payload_msg_t *p,
                           const lora_module_rx_t *m)
{
  if (!json_out || json_cap < 64 || !cfg || !s || !p || !m)
  {
    return -1;
  }

  char payload_buf_local[96];
  char *payload_buf = payload_out ? payload_out : payload_buf_local;
  const size_t payload_buf_cap = payload_out ? payload_cap : sizeof(payload_buf_local);

  (void)build_payload_string(payload_buf, payload_buf_cap, *s, *p, *m);

  const uint16_t bateria_mv = clamp_u16(round_i32(s->battery_v * 1000.0f));
  const uint16_t temp_mC    = clamp_u16(round_i32(s->bmp_temp_c * 1000.0f));
  const uint16_t press_hpa  = clamp_u16(round_i32(s->pressure_pa / 100.0f));

  const int16_t gx_mrad = clamp_i16(round_i32(s->gyro_rad_s_x * 1000.0f));
  const int16_t gy_mrad = clamp_i16(round_i32(s->gyro_rad_s_y * 1000.0f));
  const int16_t gz_mrad = clamp_i16(round_i32(s->gyro_rad_s_z * 1000.0f));

  const int16_t ax_mms2 = clamp_i16(round_i32(s->accel_m_s2_x * 1000.0f));
  const int16_t ay_mms2 = clamp_i16(round_i32(s->accel_m_s2_y * 1000.0f));
  const int16_t az_mms2 = clamp_i16(round_i32(s->accel_m_s2_z * 1000.0f));

  const int n = snprintf(
    json_out, json_cap,
    "{\"equipe\":%u,\"bateria\":%u,\"temperatura\":%u,\"pressao\":%u,"
    "\"giroscopio\":\"%d,%d,%d\",\"acelerometro\":\"%d,%d,%d\",\"payload\":%s}",
    (unsigned)cfg->equipe,
    (unsigned)bateria_mv,
    (unsigned)temp_mC,
    (unsigned)press_hpa,
    (int)gx_mrad, (int)gy_mrad, (int)gz_mrad,
    (int)ax_mms2, (int)ay_mms2, (int)az_mms2,
    payload_buf
  );

  if (n <= 0 || (size_t)n >= json_cap)
  {
    return -2;
  }

  return n;
}

int obsat_http_post_json(const obsat_http_config_t *cfg,
                         const char *json, size_t json_len,
                         char *resp_out, size_t resp_cap)
{
  if (!cfg || !cfg->url || !json || json_len == 0)
  {
    return -1;
  }

  WiFiClientSecure client;
  if (cfg->tls_insecure) client.setInsecure();
  client.setTimeout((int)(cfg->http_timeout_ms / 1000U));

  HTTPClient https;
  https.setTimeout(cfg->http_timeout_ms);

  if (!https.begin(client, cfg->url))
  {
    return -2;
  }

  https.addHeader("Content-Type", "application/json");
  const int code = https.POST((uint8_t*)json, json_len);

  if (resp_out && resp_cap > 0)
  {
    resp_out[0] = '\0';
    if (code > 0)
    {
      String resp = https.getString();
      const size_t copy_n = (resp.length() < (resp_cap - 1)) ? resp.length() : (resp_cap - 1);
      memcpy(resp_out, resp.c_str(), copy_n);
      resp_out[copy_n] = '\0';
    }
  }

  https.end();
  return code;
}
