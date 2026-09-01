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

#include <esp_heap_caps.h>

#include "sensesp_app_builder.h"
#include "sensesp_ble_gateway/ble_signalk_gateway.h"
#include "sensesp_ble_gateway/native_bluedroid_ble.h"

using namespace sensesp;

// File-static so the heartbeat lambda below can read their counters.
static std::shared_ptr<NativeBLE> g_ble;
static std::shared_ptr<BLESignalKGateway> g_gateway;

// ESP.getFreeHeap() omits MALLOC_CAP_8BIT, so it counts the IRAM-only region
// that cannot hold data — about 31 kB on an ESP32.
static uint32_t usable_free_heap() {
  return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

// Free heap alone hides fragmentation: it rises while the largest block
// collapses, and the block is what decides whether a TLS session sets up.
static uint32_t largest_free_block() {
  return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                          MALLOC_CAP_8BIT);
}

void setup() {
  SetupLogging(ESP_LOG_INFO);

  SensESPAppBuilder builder;
  auto app = builder.set_hostname(GATEWAY_HOSTNAME)
                 ->enable_ota("ble-gw-ota")
                 ->get_app();

  // Passive scanning at roughly a 9% duty cycle. The library's own
  // defaults are an active scan with the radio always on, which suits a
  // chip whose only job is BLE; here WiFi wants the same radio and the
  // same heap, and in a busy anchorage the full-rate scan buffers
  // advertisements faster than they can be forwarded.
  NativeBLEConfig ble_cfg;
  ble_cfg.active_scan = false;
  ble_cfg.scan_interval_ms = 320;
  ble_cfg.scan_window_ms = 30;
  g_ble = std::make_shared<NativeBLE>(ble_cfg);

  BLESignalKGatewayConfig gw_cfg;
  gw_cfg.post_interval_ms = 3000;
  gw_cfg.max_gatt_sessions = g_ble->max_gatt_connections();
#ifdef CONFIG_IDF_TARGET_ESP32C3
  // The C3 has a single 400 kB SRAM bank that WiFi, Bluedroid and TLS
  // all draw from, and the control WebSocket is a second long-lived
  // connection on top of the Signal K one. Advertisements still flow
  // over the HTTP POST channel without it.
  gw_cfg.enable_control_ws = false;
#endif

  g_gateway =
      std::make_shared<BLESignalKGateway>(g_ble, app->get_ws_client(), gw_cfg);
  g_gateway->start();

  event_loop()->onRepeat(5000, []() {
    ESP_LOGI("GW",
             "alive — uptime=%lus heap=%u lfb=%u ble_hits=%u ble_scan=%d "
             "gw_rx=%u gw_posted=%u gw_dropped=%u post_ok=%u post_fail=%u "
             "ws_up=%d",
             (unsigned long)(millis() / 1000), (unsigned)usable_free_heap(),
             (unsigned)largest_free_block(),
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

void loop() {
  event_loop()->tick();
  // tick() returns immediately when no event is due, so a bare loop
  // never lets the idle task on this core feed the task watchdog.
  delay(1);
}
