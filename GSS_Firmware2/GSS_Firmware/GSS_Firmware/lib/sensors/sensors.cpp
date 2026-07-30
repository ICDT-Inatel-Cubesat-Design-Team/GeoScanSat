#include "sensors.h"

#include <Wire.h>
#include <math.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>

static sensors_config_t cfg_local;

static Adafruit_MPU6050 mpu6050;
static Adafruit_BMP280  bmp280;
static QMC5883LCompass  qmc5883l;

static bool mpu6050_ok  = false;
static bool bmp280_ok   = false;
static bool qmc5883l_ok = false;

static bool i2c_ping(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static void mpu6050_enable_i2c_bypass()
{
  const uint8_t mpu_addr = 0x68;

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6A);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
}

static float read_battery_voltage_v()
{
  const float divider_ratio = (cfg_local.battery_r1_ohm + cfg_local.battery_r2_ohm) / cfg_local.battery_r2_ohm;

  int raw = analogRead((int)cfg_local.adc_battery_pin);
  float v_adc = (raw / cfg_local.adc_max_counts) * cfg_local.adc_full_scale_v;
  return v_adc * divider_ratio;
}

bool sensors_init(const sensors_config_t *cfg)
{
  if (cfg == nullptr)
  {
    return false;
  }

  cfg_local = *cfg;

  Wire.begin(cfg_local.i2c_sda_pin, cfg_local.i2c_scl_pin);
  Wire.setClock(cfg_local.i2c_clock_hz);
  Wire.setTimeout(50);

  analogReadResolution(12);
  analogSetPinAttenuation((int)cfg_local.adc_battery_pin, ADC_11db);

  mpu6050_ok = mpu6050.begin();
  if (mpu6050_ok)
  {
    mpu6050.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu6050_enable_i2c_bypass();
    delay(10);
  }

  bmp280_ok = bmp280.begin(0x76);
  if (!bmp280_ok)
  {
    bmp280_ok = bmp280.begin(0x77);
  }

  if (bmp280_ok)
  {
    bmp280.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_500
    );
  }

  qmc5883l_ok = i2c_ping(0x0D);
  if (qmc5883l_ok)
  {
    qmc5883l.init();
  }

  return (mpu6050_ok || bmp280_ok || qmc5883l_ok);
}

sensor_frame_t sensors_read_frame()
{
  sensor_frame_t frame;

  frame.ts_ms = millis();

  frame.accel_m_s2_x = NAN; frame.accel_m_s2_y = NAN; frame.accel_m_s2_z = NAN;
  frame.gyro_rad_s_x = NAN; frame.gyro_rad_s_y = NAN; frame.gyro_rad_s_z = NAN;

  frame.mag_uT_x = NAN; frame.mag_uT_y = NAN; frame.mag_uT_z = NAN;

  frame.bmp_temp_c  = NAN;
  frame.pressure_pa = NAN;
  frame.altitude_m  = NAN;

  frame.battery_v = read_battery_voltage_v();

  if (mpu6050_ok)
  {
    sensors_event_t accel_event, gyro_event, temp_event;
    mpu6050.getEvent(&accel_event, &gyro_event, &temp_event);

    frame.accel_m_s2_x = accel_event.acceleration.x;
    frame.accel_m_s2_y = accel_event.acceleration.y;
    frame.accel_m_s2_z = accel_event.acceleration.z;

    frame.gyro_rad_s_x = gyro_event.gyro.x;
    frame.gyro_rad_s_y = gyro_event.gyro.y;
    frame.gyro_rad_s_z = gyro_event.gyro.z;
  }

  if (qmc5883l_ok)
  {
    qmc5883l.read();
    const int x = qmc5883l.getX();
    const int y = qmc5883l.getY();
    const int z = qmc5883l.getZ();

    const float lsb_per_uT = 120.0f;
    frame.mag_uT_x = (float)x / lsb_per_uT;
    frame.mag_uT_y = (float)y / lsb_per_uT;
    frame.mag_uT_z = (float)z / lsb_per_uT;
  }

  if (bmp280_ok)
  {
    frame.bmp_temp_c  = bmp280.readTemperature();
    frame.pressure_pa = bmp280.readPressure();
    frame.altitude_m  = bmp280.readAltitude(cfg_local.sea_level_hpa);
  }

  return frame;
}

bool sensors_mpu_ok() { return mpu6050_ok; }
bool sensors_bmp_ok() { return bmp280_ok; }
bool sensors_qmc_ok() { return qmc5883l_ok; }
