#pragma once

#include <Arduino.h>

typedef struct
{
  uint32_t ts_ms;

  float accel_m_s2_x;
  float accel_m_s2_y;
  float accel_m_s2_z;

  float gyro_rad_s_x;
  float gyro_rad_s_y;
  float gyro_rad_s_z;

  float mag_uT_x;
  float mag_uT_y;
  float mag_uT_z;

  float bmp_temp_c;
  float pressure_pa;
  float altitude_m;

  float battery_v;
} sensor_frame_t;

typedef struct
{
  uint8_t i2c_sda_pin;
  uint8_t i2c_scl_pin;
  uint32_t i2c_clock_hz;
  uint8_t adc_battery_pin;

  float battery_r1_ohm;
  float battery_r2_ohm;
  float adc_max_counts;
  float adc_full_scale_v;

  float sea_level_hpa;
} sensors_config_t;

bool sensors_init(const sensors_config_t *cfg);
sensor_frame_t sensors_read_frame();
bool sensors_mpu_ok();
bool sensors_bmp_ok();
bool sensors_qmc_ok();
