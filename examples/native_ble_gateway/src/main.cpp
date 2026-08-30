/**
 * @file main.cpp
 * @brief SensESP BLE gateway on an ESP32 with a native BLE radio.
 *
 * The same source builds for ESP32, ESP32-S3 and ESP32-C3. Each of
 * them has an on-chip BLE controller, so NativeBLE talks to Bluedroid
 * directly and no companion chip is involved.
 *
 * Enter the WiFi credentials and the Signal K server address in the
 * SensESP configuration web UI after the first boot.
 */

#include "sensesp_app_builder.h"
#include "sensesp_ble_gateway/ble_signalk_gateway.h"
#include "sensesp_ble_gateway/native_bluedroid_ble.h"

using namespace sensesp;

// File-static so the heartbeat lambda below can read their counters.
static std::shared_ptr<NativeBLE> g_ble;
static std::shared_ptr<BLESignalKGateway> g_gateway;

void setup() {
  SetupLogging(ESP_LOG_INFO);

  SensESPAppBuilder builder;
  auto app = builder.set_hostname(GATEWAY_HOSTNAME)
                 ->enable_ota("ble-gw-ota")
                 ->get_app();

  g_ble = std::make_shared<NativeBLE>();

  BLESignalKGatewayConfig gw_cfg;
  gw_cfg.max_gatt_sessions = g_ble->max_gatt_connections();
#ifdef CONFIG_IDF_TARGET_ESP32C3
  // The C3 has a single 400 kB SRAM bank that WiFi, Bluedroid and
  // TLS all draw from. A smaller advertisement buffer and no control
  // WebSocket keep the HTTP POST path clear of the heap floor.
  gw_cfg.max_pending_ads = 50;
  gw_cfg.enable_control_ws = false;
#endif

  g_gateway =
      std::make_shared<BLESignalKGateway>(g_ble, app->get_ws_client(), gw_cfg);
  g_gateway->start();

  event_loop()->onRepeat(5000, []() {
    ESP_LOGI("GW",
             "alive — uptime=%lus heap=%u ble_hits=%u ble_scan=%d gw_rx=%u "
             "gw_posted=%u gw_dropped=%u post_ok=%u post_fail=%u ws_up=%d",
             (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap(),
             (unsigned)(g_ble ? g_ble->scan_hit_count() : 0),
             (int)(g_ble ? g_ble->is_scanning() : false),
             (unsigned)(g_gateway ? g_gateway->advertisements_received() : 0),
             (unsigned)(g_gateway ? g_gateway->advertisements_posted() : 0),
             (unsigned)(g_gateway ? g_gateway->advertisements_dropped() : 0),
             (unsigned)(g_gateway ? g_gateway->http_post_success() : 0),
             (unsigned)(g_gateway ? g_gateway->http_post_fail() : 0),
             (int)(g_gateway ? g_gateway->control_ws_connected() : false));
  });
}

void loop() { event_loop()->tick(); }
