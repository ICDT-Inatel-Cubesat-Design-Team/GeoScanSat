#include "wifi_comm.h"

static wifi_comm_config_t cfg_local = {};
static bool cfg_ok = false;

static SemaphoreHandle_t wifi_mutex_local = nullptr;
static EventGroupHandle_t wifi_event_group_local = nullptr;

static void wifi_set_connected_bit(bool connected)
{
  if (wifi_event_group_local == nullptr) return;

  if (connected) xEventGroupSetBits(wifi_event_group_local, WIFI_COMM_CONNECTED_BIT);
  else           xEventGroupClearBits(wifi_event_group_local, WIFI_COMM_CONNECTED_BIT);
}

static void wifi_on_event(WiFiEvent_t event)
{
  switch (event)
  {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifi_set_connected_bit(true);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      wifi_set_connected_bit(false);
      break;
    default:
      break;
  }
}

bool wifi_comm_init(const wifi_comm_config_t *cfg)
{
  if (cfg == nullptr || cfg->ssid == nullptr || cfg->pass == nullptr)
  {
    return false;
  }

  cfg_local = *cfg;
  cfg_ok = true;

  if (wifi_mutex_local == nullptr) wifi_mutex_local = xSemaphoreCreateMutex();
  if (wifi_event_group_local == nullptr) wifi_event_group_local = xEventGroupCreate();

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(cfg_local.auto_reconnect);
  WiFi.setSleep(cfg_local.sleep_enable);

  WiFi.onEvent(wifi_on_event);

  wifi_comm_update_bits();
  return true;
}

bool wifi_comm_is_connected()
{
  return (WiFi.status() == WL_CONNECTED);
}

void wifi_comm_update_bits()
{
  wifi_set_connected_bit(wifi_comm_is_connected());
}

void wifi_comm_kick_connect_nonblocking()
{
  if (!cfg_ok) return;

  if (WiFi.getMode() != WIFI_STA) WiFi.mode(WIFI_STA);

  const bool has_ssid = (WiFi.SSID().length() > 0);
  if (has_ssid)
  {
    (void)WiFi.reconnect();
  }
  else
  {
    WiFi.begin(cfg_local.ssid, cfg_local.pass);
  }
}

bool wifi_comm_wait_connected(uint32_t timeout_ms)
{
  const uint32_t t0 = millis();
  while (!wifi_comm_is_connected())
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    if ((millis() - t0) >= timeout_ms) break;
  }
  wifi_comm_update_bits();
  return wifi_comm_is_connected();
}

SemaphoreHandle_t wifi_comm_mutex()
{
  return wifi_mutex_local;
}

EventGroupHandle_t wifi_comm_event_group()
{
  return wifi_event_group_local;
}

bool wifi_comm_try_lock(uint32_t timeout_ms)
{
  if (wifi_mutex_local == nullptr) return false;
  return (xSemaphoreTake(wifi_mutex_local, pdMS_TO_TICKS(timeout_ms)) == pdTRUE);
}

void wifi_comm_unlock()
{
  if (wifi_mutex_local == nullptr) return;
  xSemaphoreGive(wifi_mutex_local);
}
