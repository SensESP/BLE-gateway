#ifndef SENSESP_NET_BLE_BLE_SIGNALK_GATEWAY_H_
#define SENSESP_NET_BLE_BLE_SIGNALK_GATEWAY_H_

#include <Arduino.h>
#include <ArduinoJson.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "esp_http_client.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sensesp_ble_gateway/gatt_session.h"

#include "sensesp_ble_gateway/ble_advertisement.h"
#include "sensesp_ble_gateway/ble_provisioner.h"
#include "sensesp/signalk/signalk_ws_client.h"

namespace sensesp {

/**
 * @brief Configuration for BLESignalKGateway.
 *
 * Defined at namespace scope rather than nested inside the class so
 * that default member initializers on its fields are fully available
 * when the BLESignalKGateway constructor declaration is parsed.
 * (C++ forbids a class's default member initializers from being used
 * inside the enclosing class's other default-argument expressions
 * while the class is still incomplete.)
 */
struct BLESignalKGatewayConfig {
  /// HTTP POST interval in ms. Default 2000.
  uint32_t post_interval_ms = 2000;

  /// Control WS status-message interval in ms. Default 30000.
  uint32_t status_interval_ms = 30000;

  /// Max pending advertisements buffered between POST flushes.
  /// If more arrive than this before the next flush, the buffer
  /// is drained to drop_oldest_keep_newest to avoid unbounded
  /// memory growth. Default 500.
  size_t max_pending_ads = 500;

  /// Gateway firmware version string sent in the hello message.
  /// Empty string means use the SensESP library version.
  String firmware_version = "";

  /// Max GATT sessions advertised in the hello message.
  int max_gatt_sessions = 0;

  /// Enable the control WebSocket (hello, status, GATT commands).
  /// Disable on memory-constrained chips (e.g. C5 with WiFi+BLE)
  /// where the WS reconnect can exhaust internal RAM. Advertisements
  /// still flow via HTTP POST when the WS is disabled.
  bool enable_control_ws = true;

  /// Smallest contiguous block of **internal** memory, in bytes, that
  /// must remain free before an advertisement is buffered. Below it the
  /// advertisement is dropped and counted.
  ///
  /// Buffering copies the advertisement, which means several small heap
  /// allocations inside std::vector and String, on the Bluetooth host's
  /// callback. This build compiles without exceptions, so an allocation
  /// that cannot be satisfied aborts the device rather than throwing,
  /// and those containers offer no failure-reporting form to check.
  ///
  /// This threshold is therefore an admission check, not a guarantee: it
  /// declines work when memory is already low. It neither reserves the
  /// memory nor rules out a failure between the check and the
  /// allocation, so it lowers the odds of an abort rather than removing
  /// them.
  ///
  /// This field is a floor, not the threshold. required_free_block()
  /// returns the larger of it and the buffer's own growth step,
  /// max_pending_ads * sizeof(BLEAdvertisement), and at the default 500
  /// that growth term is around 30 kB — so the effective check is
  /// several times this number. Worth knowing before tuning either
  /// field, because raising max_pending_ads raises the bar for
  /// buffering anything at all.
  ///
  /// It is not a handshake reservation either way. A fresh TLS
  /// handshake needed a 32.8 kB contiguous block when measured on an
  /// ESP32, and nothing here reserves that; the connection is opened
  /// before scanning starts precisely because it cannot be obtained
  /// afterwards.
  ///
  /// Measured against internal RAM specifically: PSRAM is byte
  /// addressable and would otherwise mask the shortage on the boards
  /// that have it, which are the boards where internal RAM is tightest.
  ///
  /// New fields go after this one. This struct is a public aggregate,
  /// so inserting a field in the middle silently rebinds any positional
  /// brace initialization in consumer code.
  size_t min_largest_free_block = 8192;
};

/**
 * @brief Bridge between a BLEProvisioner and signalk-server's
 *        ble-provider-api.
 *
 * Takes a BLEProvisioner (the source of advertisements and — in
 * future — GATT sessions) plus an SKWSClient (the main SensESP SK
 * delta connection, used only for credentials piggybacking) and
 * speaks the signalk-server ble-provider-api dual-channel protocol:
 *
 *   * HTTP POST /signalk/v2/api/ble/gateway/advertisements
 *     Bearer-authenticated JSON batches of recently-seen BLE
 *     advertisements. Sent periodically from a background task.
 *
 *   * WebSocket /signalk/v2/api/ble/gateway/ws?token=<JWT>
 *     Separate long-lived WebSocket for the gateway control
 *     protocol: hello handshake on connect, periodic status frames,
 *     and (in future) gatt_subscribe / gatt_write / gatt_close
 *     commands from the server.
 *
 * Both channels reuse the JWT that SKWSClient obtained via the
 * main access-request flow (via SKWSClient::get_auth_token()) so the
 * gateway does not have to run a second parallel access request.
 *
 * ## Lifecycle
 *
 * Construct with a BLEProvisioner and SKWSClient shared_ptr. Call
 * start() to begin gateway services. Gateway services are gated on
 * SK connection state: the background HTTP POST task only posts
 * when the main SK websocket is connected, and the control
 * WebSocket is (re)connected whenever the main SK websocket comes
 * up, disconnected when it goes down.
 *
 * Advertisement collection is independent of SK state — the
 * provisioner emits advertisements into an internal buffer as soon
 * as the scanner is running, and they wait there until the HTTP
 * POST task gets a chance to forward them (or drops them if the
 * buffer grows too large).
 *
 * ## GATT session handling
 *
 * gatt_subscribe / gatt_write / gatt_close are implemented on the
 * Bluedroid provisioners, which carry a session state machine with
 * notification subscriptions plus poll and periodic-write timers. The
 * NimBLE provisioner relays advertisements only, and GATT commands
 * reach the base class's no-op implementations there.
 *
 * The optional `with_response` and `write_before_read` fields of the
 * server's GATT commands are not honoured.
 *
 * A gateway that only relays advertisements — Ruuvi and Victron
 * beacons need no GATT connection — can run with the control WS off
 * and use the HTTP POST channel alone.
 */
class BLESignalKGateway {
 public:
  using Config = BLESignalKGatewayConfig;

  BLESignalKGateway(std::shared_ptr<BLEProvisioner> ble,
                    std::shared_ptr<SKWSClient> sk_client,
                    BLESignalKGatewayConfig config = BLESignalKGatewayConfig{});
  ~BLESignalKGateway();

  BLESignalKGateway(const BLESignalKGateway&) = delete;
  BLESignalKGateway& operator=(const BLESignalKGateway&) = delete;

  /**
   * @brief Start gateway services.
   *
   * Attaches observers to the BLE provisioner (to receive
   * advertisements) and to the SKWSClient's connection state
   * producer (to start/stop the control WS and HTTP POST task).
   *
   * Scanning does not begin here. The POST task opens the delivery
   * connection first and starts the scan once it is established, or
   * after roughly twenty seconds if it cannot be. Over TLS that
   * ordering is what makes the connection possible at all: the
   * handshake needs a contiguous block of tens of kilobytes, and the
   * advertisements the scan produces fragment the heap within seconds
   * of being buffered.
   *
   * Safe to call once; subsequent calls are no-ops.
   */
  void start();

  /**
   * @brief Stop gateway services.
   *
   * Stops the HTTP POST task, tears down the control WS, detaches
   * observers, and stops the BLE scan. The BLE provisioner itself
   * is not destroyed — it remains usable by other subscribers.
   */
  void stop();

  // --- debug counters ---
  uint32_t advertisements_received() const { return adv_received_count_; }
  uint32_t advertisements_posted() const { return adv_posted_count_; }
  uint32_t advertisements_dropped() const { return adv_dropped_count_; }
  uint32_t http_post_success() const { return http_post_success_; }
  uint32_t http_post_fail() const { return http_post_fail_; }
  uint32_t control_ws_connected_count() const { return ws_connected_count_; }
  bool control_ws_connected() const { return ws_connected_.load(); }

 private:
  // Attached to ble_provisioner_'s ValueProducer<BLEAdvertisement>
  // via attach(). Called synchronously from the provisioner's GAP
  // event callback for each advertisement.
  void on_advertisement();

  // Starts the control WS if it is not already running and the SK
  // server address + token are available.
  void init_control_ws();

  // Tears down the control WS.
  void destroy_control_ws();

  // Build and send a hello JSON message on the control WS.
  void send_hello();

  // Build and send a status JSON message on the control WS.
  void send_status();

  // Dispatch an incoming control WS message.
  void handle_control_ws_message(uint8_t* payload, size_t length);

  // GATT command handlers.
  void handle_gatt_subscribe(JsonDocument& doc);
  void handle_gatt_write(JsonDocument& doc);
  void handle_gatt_close(JsonDocument& doc);

  // Send a JSON message on the control WS (thread-safe).
  void send_control_json(JsonDocument& doc);

  // GATT session state machine progression.
  void gatt_run_init_writes(GATTSession* session);
  void gatt_run_subscribes(GATTSession* session);
  void gatt_start_timers(GATTSession* session);
  void gatt_cleanup_session(const String& session_id);

  // FreeRTOS timer callbacks for poll and periodic write.
  static void poll_timer_cb(TimerHandle_t timer);
  static void periodic_write_timer_cb(TimerHandle_t timer);

  /**
   * @brief Drain pending_ads_ and POST them to signalk-server.
   *
   * @param allow_empty Send a batch even with nothing buffered. Used to
   *        open the connection before the scanner starts; in normal
   *        operation an empty buffer is skipped instead.
   * @return true if the server accepted the batch, or there was nothing
   *         to send. False means the delivery path is not working.
   */
  bool post_pending_advertisements(bool allow_empty = false);

  // FreeRTOS task that runs post_pending_advertisements() on a
  // timer. Started in start(), stopped in stop().
  static void post_task_entry(void* arg);
  void post_task_loop();

  // esp_websocket_client event handler trampoline + instance method.
  static void control_ws_event_trampoline(void* handler_args,
                                          esp_event_base_t base,
                                          int32_t event_id, void* event_data);
  void handle_control_ws_event(int32_t event_id, void* event_data);

  /// How this gateway's own two channels should reach the server.
  enum class Transport {
    kPlaintext,    ///< SSL is off; connect over http:// and ws://.
    kTls,          ///< SSL is on and a usable trust anchor is available.
    kUnavailable,  ///< SSL is on but nothing can verify the server.
  };

  /**
   * @brief Decide how this gateway's own two channels reach the server.
   *
   * The gateway talks to signalk-server on its own HTTP and WebSocket
   * connections, separate from SKWSClient's delta socket, but to the
   * same server and carrying the same bearer token. So it follows
   * SKWSClient's SSL setting rather than deciding independently, and
   * borrows the trust anchor SKWSClient pinned on first use instead of
   * running a second, competing exchange.
   *
   * Borrowing the anchor is not enough on its own. SKWSClient pins an
   * issuing CA and then binds the leaf's DNS SAN set in its own verify
   * callback — without that binding, any leaf the CA ever signed is
   * accepted, which is the whole risk with a public CA. A separate
   * connection cannot inherit that callback, so this reproduces the
   * binding by passing the pinned SAN as the expected certificate name.
   *
   * kUnavailable covers three cases the caller must not connect through,
   * because both channels carry the Signal K token: no anchor pinned
   * yet, an anchor pinned in SKWSClient's leaf-fingerprint mode, which
   * these clients have no way to express, and a pinned CA with no
   * stored identity, which would authorize every leaf that CA signed.
   *
   * @param ca_pem      Receives the pinned CA in PEM form for kTls.
   * @param common_name Receives the certificate name to require. Never
   *                    empty for kTls: an anchor without a bindable
   *                    identity resolves to kUnavailable instead.
   */
  Transport resolve_transport(String& ca_pem, String& common_name) const;

  /// Contiguous internal memory that must be free to buffer safely.
  size_t required_free_block() const;

  /**
   * @brief Make sure the advertisement POST client exists and matches
   *        the given endpoint.
   *
   * The client is kept alive between batches instead of being built and
   * torn down around each one. Over TLS that matters a great deal: a
   * handshake needs a contiguous block of tens of kilobytes, and once
   * advertisements are being buffered the heap is too fragmented to
   * supply one on demand, so a client that reconnects every batch never
   * connects at all. Holding one connection open means the cost is paid
   * once, while the heap is still clean.
   *
   * Rebuilds the client if the URL or the CA has changed since it was
   * created; otherwise reuses it as is.
   *
   * @return true if post_client_ is usable.
   */
  bool ensure_post_client(const String& url, const String& ca_pem,
                          const String& common_name);

  /// Tear down the advertisement POST client, if any.
  void destroy_post_client();

  std::shared_ptr<BLEProvisioner> ble_provisioner_;
  std::shared_ptr<SKWSClient> sk_client_;
  Config config_;

  std::atomic<bool> started_{false};
  std::atomic<bool> sk_connected_{false};
  std::atomic<bool> ws_connected_{false};

  // Pending advertisements waiting to be POSTed. Guarded by
  // pending_ads_mutex_ because the producer (BLE GAP event) and the
  // consumer (background HTTP POST task) run on different FreeRTOS
  // tasks.
  std::vector<BLEAdvertisement> pending_ads_;
  SemaphoreHandle_t pending_ads_mutex_ = nullptr;

  // Control WebSocket client. esp_websocket_client keeps the pointer it
  // is handed rather than copying the PEM, so the string has to outlive
  // the client.
  esp_websocket_client_handle_t control_ws_ = nullptr;
  String control_ws_ca_pem_;
  String control_ws_cn_;
  SemaphoreHandle_t control_ws_mutex_ = nullptr;

  // Advertisement POST client, kept open across batches. Like the
  // control WS, esp_http_client keeps the PEM pointer it is handed, so
  // the string has to outlive the client. Touched only by the POST task
  // and by that task's own shutdown path.
  esp_http_client_handle_t post_client_ = nullptr;
  String post_url_;
  String post_ca_pem_;
  String post_cn_;

  // Set once the unsupported-anchor case has been reported, so a
  // permanent condition is not logged every post interval.
  mutable std::atomic<bool> tls_unavailable_logged_{false};

  // Consecutive failed POST attempts. A kept-alive connection that the
  // server or the network has killed cannot recover on its own, so the
  // client is rebuilt once the streak crosses a threshold.
  int post_failure_streak_ = 0;

  // Background POST task handle. post_task_exited_ lets stop() wait for
  // the task to leave the loop before the object's state is torn down;
  // without it a stop()/start() pair can run two tasks against one
  // esp_http_client handle, and the destructor can free the mutexes
  // under a live task.
  TaskHandle_t post_task_ = nullptr;
  std::atomic<bool> post_task_should_run_{false};
  std::atomic<bool> post_task_exited_{true};

  // Active GATT sessions keyed by session_id.
  std::map<String, std::unique_ptr<GATTSession>> gatt_sessions_;
  SemaphoreHandle_t gatt_sessions_mutex_ = nullptr;
  // Suppress scan watchdog while GATT connections are being established.
  std::atomic<bool> scan_suppressed_{false};

  // Counters.
  std::atomic<uint32_t> adv_received_count_{0};
  std::atomic<uint32_t> adv_posted_count_{0};
  std::atomic<uint32_t> adv_dropped_count_{0};
  std::atomic<uint32_t> http_post_success_{0};
  std::atomic<uint32_t> http_post_fail_{0};
  std::atomic<uint32_t> ws_connected_count_{0};
};

}  // namespace sensesp

#endif  // SENSESP_NET_BLE_BLE_SIGNALK_GATEWAY_H_
