#include "sensesp_ble_gateway/ble_signalk_gateway.h"

#include "esp_heap_caps.h"
#include "esp_http_client.h"

#include "esp_idf_version.h"
#include "esp_log.h"
#include "sensesp/sensesp_version.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/signalk/signalk_ws_client.h"
#include "sensesp_app.h"

namespace sensesp {

namespace {
constexpr const char* kTag = "ble_gw";

// Arduino's ESP.getFreeHeap() asks for MALLOC_CAP_INTERNAL without
// MALLOC_CAP_8BIT, so it counts the IRAM-only region — about 31 kB on an
// ESP32 — which is 32-bit-access-only and can never hold a buffer. Report what
// an allocation can actually come from.
uint32_t usable_free_heap() {
  return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

#ifdef BLE_GW_HEAP_TRACE
// Largest contiguous internal block, which is what a TLS handshake needs and
// what the advertisement buffer competes with. Free heap alone hides
// fragmentation, so both are reported.
void heap_probe(const char* label, size_t batch, size_t pending_capacity) {
  ESP_LOGW(kTag, "HEAPTRACE %s largest=%u free=%u batch=%u cap=%u", label,
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                                      MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL |
                                             MALLOC_CAP_8BIT),
           (unsigned)batch, (unsigned)pending_capacity);
}
#define BLE_GW_HEAP_PROBE(label, batch, cap) heap_probe(label, batch, cap)
#else
#define BLE_GW_HEAP_PROBE(label, batch, cap) ((void)0)
#endif

constexpr const char* kAdvertisementsPath =
    "/signalk/v2/api/ble/gateway/advertisements";
constexpr const char* kControlWsPathPrefix =
    "/signalk/v2/api/ble/gateway/ws?token=";

// Ceiling for the post-interval backoff applied after an HTTP status
// failure. A rejected token on a boat can stand for days; this keeps the
// retry rate low without ever giving up on it.
constexpr uint32_t kPostBackoffCapMs = 300000;

// Floor for timer periods that arrive over the control WebSocket.
constexpr uint32_t kMinGattIntervalMs = 100;

// Capacity handed to the pending buffer for the short window between draining
// a batch and restoring the full reservation. Without it the buffer sits at
// zero capacity and the GAP callback grows it from nothing, one doubling at a
// time, in a build with exceptions disabled. Sized for the window rather than
// the buffer: at the ~15 advertisements/s a busy anchorage produces and a
// window measured at 100 ms, this is two orders of magnitude of headroom, and
// it costs about 2 kB against the full array a second reservation would take.
constexpr size_t kDrainWindowReserve = 32;

// SKWSClient stores the leaf's DNS SANs as a normalized, sorted,
// comma-joined set. esp-tls verifies against a single name, so bind to
// the first one.
String first_san(const String& san_set) {
  const int comma = san_set.indexOf(',');
  return comma < 0 ? san_set : san_set.substring(0, comma);
}

String bytes_to_hex(const std::vector<uint8_t>& data) {
  String out;
  out.reserve(data.size() * 2);
  for (uint8_t b : data) {
    char tmp[3];
    snprintf(tmp, sizeof(tmp), "%02X", b);
    out += tmp;
  }
  return out;
}

}  // namespace

BLESignalKGateway::BLESignalKGateway(std::shared_ptr<BLEProvisioner> ble,
                                     std::shared_ptr<SKWSClient> sk_client,
                                     Config config)
    : ble_provisioner_(std::move(ble)),
      sk_client_(std::move(sk_client)),
      config_(config) {
  pending_ads_mutex_ = xSemaphoreCreateMutex();
  control_ws_mutex_ = xSemaphoreCreateMutex();
  gatt_sessions_mutex_ = xSemaphoreCreateMutex();
  pending_ads_.reserve(config_.max_pending_ads);
}

BLESignalKGateway::~BLESignalKGateway() {
  stop();
  if (pending_ads_mutex_ != nullptr) {
    vSemaphoreDelete(pending_ads_mutex_);
    pending_ads_mutex_ = nullptr;
  }
  if (control_ws_mutex_ != nullptr) {
    vSemaphoreDelete(control_ws_mutex_);
    control_ws_mutex_ = nullptr;
  }
  if (gatt_sessions_mutex_ != nullptr) {
    vSemaphoreDelete(gatt_sessions_mutex_);
    gatt_sessions_mutex_ = nullptr;
  }
}

void BLESignalKGateway::start() {
  if (started_.exchange(true)) {
    return;
  }
  if (!ble_provisioner_ || !sk_client_) {
    ESP_LOGE(kTag, "start() called with null provisioner or SK client");
    started_.store(false);
    return;
  }

  ESP_LOGI(kTag, "Starting BLE SignalK gateway services");

  // Attach an observer to the BLE provisioner's ValueProducer. Each
  // received advertisement triggers on_advertisement(), which reads
  // via get() and buffers into pending_ads_ for the HTTP POST task.
  ble_provisioner_->attach([this]() { this->on_advertisement(); });

  // connect_to() below attaches without replaying the current value, so seed
  // the state directly. Independent of the control WS: with it disabled,
  // a gateway started while SK is already up would otherwise never post.
  sk_connected_.store(sk_client_->is_connected());

  // The POST task before the control WebSocket, deliberately. Its stack is one
  // 8 kB contiguous allocation and delivery is what the gateway is for, while
  // the control WebSocket is optional and costs a whole TLS session. Started
  // the other way round on a device where Signal K is already connected, the
  // control channel takes the memory and task creation fails.
  post_task_should_run_.store(true);
  post_task_exited_.store(false);
  if (xTaskCreate(&BLESignalKGateway::post_task_entry, "ble_gw_post", 8192,
                  this, 1, &post_task_) != pdPASS) {
    // The task owns delivery, the scan watchdog and the scan start, so
    // losing it would otherwise leave a gateway that reports itself
    // running and does nothing at all. Scan anyway: a device that
    // collects and drops is diagnosable, a silent one is not.
    ESP_LOGE(kTag, "Failed to create the POST task — scanning without it");
    post_task_ = nullptr;
    post_task_should_run_.store(false);
    post_task_exited_.store(true);
    if (!ble_provisioner_->is_scanning()) {
      ble_provisioner_->start_scan();
    }
    return;
  }

  // Hook the SK connection state producer. SKWSClient inherits from
  // ValueProducer<SKWSConnectionState> so we can connect_to() it
  // directly. When SK connects we want to (re)start the control WS;
  // when it disconnects we want to tear it down so it does not keep
  // trying to reach a dead server.
  sk_client_->connect_to(
      new LambdaConsumer<SKWSConnectionState>([this](SKWSConnectionState s) {
        bool connected = (s == SKWSConnectionState::kSKWSConnected);
        sk_connected_.store(connected);
        if (config_.enable_control_ws && connected && control_ws_ == nullptr) {
          ESP_LOGI(kTag,
                   "SK main WS connected — starting BLE gateway control WS");
          init_control_ws();
        }
      }));
  // Re-seed after attaching: the POST task is already running by now, and a
  // connection that completed between the first seed and this attach would
  // otherwise leave it waiting on a state nobody will publish again.
  sk_connected_.store(sk_client_->is_connected());
  if (config_.enable_control_ws && sk_connected_.load()) {
    init_control_ws();
  }

  // Scanning is started by the POST task, not here — see
  // post_task_loop(), which opens the delivery connection first.
}

void BLESignalKGateway::stop() {
  if (!started_.exchange(false)) {
    return;
  }

  ESP_LOGI(kTag, "Stopping BLE SignalK gateway services");

  // Stop POST task. The task loop checks post_task_should_run_ on
  // each iteration and exits cleanly when it flips to false.
  post_task_should_run_.store(false);
  // The task deletes itself at the end of post_task_loop(); wait for it
  // to get there. Returning early would let a following start() run a
  // second task against the same esp_http_client handle, and would let
  // the destructor free the mutexes while this one is still using them.
  // Bounded so a task wedged in a socket timeout cannot hang the caller.
  for (int i = 0; i < 100 && !post_task_exited_.load(); ++i) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  if (!post_task_exited_.load()) {
    ESP_LOGW(kTag, "POST task did not exit within 10 s");
  }
  post_task_ = nullptr;

  destroy_control_ws();

  // The BLE provisioner outlives the gateway — we only stop scanning
  // if we were the ones who started it. For simplicity in this first
  // cut, always stop the scanner on gateway stop. Users who want
  // scanning to continue after stop() can call ble->start_scan()
  // again afterwards.
  if (ble_provisioner_ && ble_provisioner_->is_scanning()) {
    ble_provisioner_->stop_scan();
  }

  // Note: we do not detach the ble_provisioner_ observer. The
  // Observable::attach API does not currently support detaching by
  // functor identity (only by int ID returned from attach), and the
  // cost of letting a dead-gateway callback run once is minimal —
  // on_advertisement() will see started_==false and early-return.
}

void BLESignalKGateway::on_advertisement() {
  if (!started_.load()) {
    return;
  }
  adv_received_count_.fetch_add(1, std::memory_order_relaxed);

  if (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                      MALLOC_CAP_8BIT) <
      config_.min_largest_free_block) {
    adv_dropped_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  // A zero-length buffer means buffer nothing, and saying so here is what
  // makes it mean that. Falling through would let the first push allocate
  // the element array on this callback, after which the buffer would behave
  // as if it held one.
  if (config_.max_pending_ads == 0) {
    adv_dropped_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  const BLEAdvertisement& ad = ble_provisioner_->get();

  if (xSemaphoreTake(pending_ads_mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
    // Could not grab the buffer mutex quickly enough — drop this
    // advertisement rather than block the GAP event callback.
    adv_dropped_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  // Bound by the capacity actually reserved, not only by the configured
  // maximum. Between a drain and restore_pending_capacity() the array is
  // reserved for the window rather than the whole buffer, and pushing past
  // that would reallocate here — on the Bluetooth host's callback, in a
  // build without exceptions, where a failed allocation aborts the device.
  // Trimming instead keeps the element array off this callback's allocation
  // path. The copy itself still allocates the advertisement's name string
  // and payload vectors, as it always has; only the array is covered here.
  const size_t capacity = pending_ads_.capacity() < config_.max_pending_ads
                              ? pending_ads_.capacity()
                              : config_.max_pending_ads;
  if (pending_ads_.size() >= capacity) {
    // Buffer full — drop oldest to keep newest. Cheap approximation:
    // drop the first half so we do not ping-pong on every new ad.
    const size_t keep = capacity / 2;
    const size_t dropped = pending_ads_.size() - keep;
    pending_ads_.erase(pending_ads_.begin(), pending_ads_.begin() + dropped);
    adv_dropped_count_.fetch_add(dropped, std::memory_order_relaxed);
  }
  pending_ads_.push_back(ad);
  xSemaphoreGive(pending_ads_mutex_);
}

BLESignalKGateway::Transport BLESignalKGateway::resolve_transport(
    String& ca_pem, String& common_name) const {
  ca_pem = "";
  common_name = "";

  if (!sk_client_->is_ssl_enabled()) {
    return Transport::kPlaintext;
  }
  if (!sk_client_->has_tofu_ca()) {
    if (!tls_unavailable_logged_.exchange(true)) {
      if (sk_client_->has_tofu_fingerprint()) {
        ESP_LOGE(kTag,
                 "SSL is on and the Signal K client pinned a leaf "
                 "fingerprint rather than a CA. The gateway's own channels "
                 "cannot express that anchor, so they stay disabled. Give "
                 "the server a certificate with its issuing CA in the chain "
                 "to enable them.");
      } else {
        ESP_LOGE(kTag,
                 "SSL is on but no trust anchor is pinned; the gateway's own "
                 "channels stay disabled rather than send the Signal K token "
                 "to a server they cannot verify.");
      }
    }
    return Transport::kUnavailable;
  }

  const String san = first_san(sk_client_->get_tofu_san());
  if (san.length() == 0) {
    if (!tls_unavailable_logged_.exchange(true)) {
      ESP_LOGE(kTag,
               "SSL is on and a CA is pinned, but no leaf identity was stored "
               "with it, so there is no name to bind the certificate to and "
               "the CA would authorize any leaf it ever signed. The gateway's "
               "own channels stay disabled. Reset the pinned trust anchor on "
               "a trusted network to re-pin both.");
    }
    return Transport::kUnavailable;
  }

  ca_pem = sk_client_->get_tofu_ca();
  common_name = san;
  return Transport::kTls;
}

void BLESignalKGateway::destroy_post_client() {
  if (post_client_ != nullptr) {
    esp_http_client_cleanup(post_client_);
    post_client_ = nullptr;
  }
  post_url_ = "";
  post_ca_pem_ = "";
  post_cn_ = "";
}

bool BLESignalKGateway::ensure_post_client(const String& url,
                                           const String& ca_pem,
                                           const String& common_name) {
  if (post_client_ != nullptr && post_url_ == url && post_ca_pem_ == ca_pem &&
      post_cn_ == common_name) {
    return true;
  }
  destroy_post_client();

  post_url_ = url;
  post_ca_pem_ = ca_pem;
  post_cn_ = common_name;

  esp_http_client_config_t cfg = {};
  cfg.url = post_url_.c_str();
  cfg.method = HTTP_METHOD_POST;
  cfg.timeout_ms = 3000;
  cfg.keep_alive_enable = true;
  // The advertisements endpoint is a fixed path on a known server. A
  // redirect would carry the bearer token to whatever host it names, and
  // a redirect to http:// would carry it in the clear.
  cfg.disable_auto_redirect = true;
  if (post_ca_pem_.length() > 0) {
    cfg.cert_pem = post_ca_pem_.c_str();
    // resolve_transport() reports kUnavailable rather than hand out a CA with
    // no identity to bind it to, so the name check is never skipped here.
    // Without it the pinned CA would authorize any leaf it ever signed, and
    // the POST URL is built from an mDNS name anyone on the LAN can answer for.
    cfg.common_name = post_cn_.c_str();
  }

  post_client_ = esp_http_client_init(&cfg);
  if (post_client_ == nullptr) {
    ESP_LOGW(kTag, "esp_http_client_init failed");
    post_url_ = "";
    post_ca_pem_ = "";
    post_cn_ = "";
    return false;
  }
  return true;
}

void BLESignalKGateway::init_control_ws() {
  if (xSemaphoreTake(control_ws_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }

  // Create if absent. Both callers mean "make sure it exists", and rebuilding a
  // live client would drop a working control channel and pay for a second TLS
  // handshake to replace it. Tested here rather than at the call sites so two
  // callers racing cannot both build one; destroy_control_ws() owns teardown.
  if (control_ws_ != nullptr) {
    xSemaphoreGive(control_ws_mutex_);
    return;
  }

  String token = sk_client_->get_auth_token();
  String addr = sk_client_->get_server_address();
  uint16_t port = sk_client_->get_server_port();

  if (addr.length() == 0 || port == 0) {
    ESP_LOGW(kTag, "SK server address not available; deferring control WS");
    xSemaphoreGive(control_ws_mutex_);
    return;
  }

  const Transport transport =
      resolve_transport(control_ws_ca_pem_, control_ws_cn_);
  if (transport == Transport::kUnavailable) {
    xSemaphoreGive(control_ws_mutex_);
    return;
  }
  const bool use_tls = transport == Transport::kTls;
  const char* scheme = use_tls ? "wss://" : "ws://";

  String url = String(scheme) + addr + ":" + String(port) +
               kControlWsPathPrefix + token;

  ESP_LOGI(kTag, "Connecting control WS to %s%s:%u%s?token=<redacted>", scheme,
           addr.c_str(), static_cast<unsigned>(port),
           "/signalk/v2/api/ble/gateway/ws");

  esp_websocket_client_config_t cfg = {};
  cfg.uri = url.c_str();
  cfg.task_stack = 4096;
  cfg.buffer_size = 1024;
  if (use_tls) {
    cfg.cert_pem = control_ws_ca_pem_.c_str();
    // The device dials the server by address, so the name on the certificate
    // is checked against the identity pinned at first use rather than against
    // the address. Without this the pinned CA would authorize any leaf it
    // ever signed. resolve_transport() guarantees the name is present.
    cfg.cert_common_name = control_ws_cn_.c_str();
  }
  // reconnect_timeout_ms and network_timeout_ms are only available
  // in the IDF component version of esp_websocket_client, not in the
  // Arduino-ESP32 prebuilt. Guard with a version check.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  cfg.reconnect_timeout_ms = 5000;
  cfg.network_timeout_ms = 10000;
#endif

  control_ws_ = esp_websocket_client_init(&cfg);
  if (control_ws_ == nullptr) {
    ESP_LOGE(kTag, "esp_websocket_client_init failed");
    xSemaphoreGive(control_ws_mutex_);
    return;
  }

  esp_err_t err = esp_websocket_register_events(
      control_ws_, WEBSOCKET_EVENT_ANY,
      &BLESignalKGateway::control_ws_event_trampoline, this);
  if (err == ESP_OK) {
    err = esp_websocket_client_start(control_ws_);
  }
  if (err != ESP_OK) {
    // Keeping a client that never started would make the failure permanent:
    // the create-if-absent test above sees a non-null handle, and nothing
    // rebuilds it. Starting allocates a task, so this fails exactly when the
    // device is short of memory and the next attempt matters most.
    ESP_LOGE(kTag, "control WS start failed: %s", esp_err_to_name(err));
    esp_websocket_client_destroy(control_ws_);
    control_ws_ = nullptr;
  }

  xSemaphoreGive(control_ws_mutex_);
}

void BLESignalKGateway::destroy_control_ws() {
  if (xSemaphoreTake(control_ws_mutex_, pdMS_TO_TICKS(200)) != pdTRUE) {
    return;
  }
  if (control_ws_ != nullptr) {
    esp_websocket_client_stop(control_ws_);
    esp_websocket_client_destroy(control_ws_);
    control_ws_ = nullptr;
  }
  ws_connected_.store(false);
  xSemaphoreGive(control_ws_mutex_);
}

void BLESignalKGateway::send_hello() {
  if (!ws_connected_.load() || control_ws_ == nullptr) {
    return;
  }
  JsonDocument doc;
  doc["type"] = "hello";
  doc["gateway_id"] = SensESPBaseApp::get_hostname();
  doc["firmware"] = config_.firmware_version.length() > 0
                        ? config_.firmware_version
                        : String(kSensESPVersion);
  doc["max_gatt_connections"] =
      ble_provisioner_ ? ble_provisioner_->max_gatt_connections()
                       : config_.max_gatt_sessions;
  doc["active_gatt_connections"] =
      ble_provisioner_ ? ble_provisioner_->active_gatt_connections() : 0;
  // signalk-server validates mac against a six-octet pattern and drops
  // the entire frame if it fails, taking the session with it. The
  // controller has no address until its stack is up, so omit the field
  // rather than send an empty string.
  if (ble_provisioner_) {
    const String mac = ble_provisioner_->mac_address();
    if (mac.length() > 0) {
      doc["mac"] = mac;
    }
  }
  doc["hostname"] = SensESPBaseApp::get_hostname();

  // IP address from the network provisioner so the server can show
  // it in the BLE manager UI.
  auto app = SensESPApp::get();
  if (app) {
    auto provisioner = app->get_network_provisioner();
    if (provisioner) {
      doc["ip_address"] = provisioner->local_ip().toString();
    }
  }

  String msg;
  serializeJson(doc, msg);
  esp_websocket_client_send_text(control_ws_, msg.c_str(), msg.length(),
                                 portMAX_DELAY);
  ESP_LOGI(kTag, "Sent hello");
}

void BLESignalKGateway::send_status() {
  if (!ws_connected_.load() || control_ws_ == nullptr) {
    return;
  }
  JsonDocument doc;
  doc["type"] = "status";
  doc["gateway_id"] = SensESPBaseApp::get_hostname();
  doc["uptime"] = millis() / 1000;
  doc["free_heap"] = usable_free_heap();
  doc["active_gatt_connections"] =
      ble_provisioner_ ? ble_provisioner_->active_gatt_connections() : 0;
  doc["max_gatt_connections"] =
      ble_provisioner_ ? ble_provisioner_->max_gatt_connections()
                       : config_.max_gatt_sessions;
  doc["scan_hits"] = ble_provisioner_ ? ble_provisioner_->scan_hit_count() : 0;
  doc["post_success"] = http_post_success_.load();
  doc["post_fail"] = http_post_fail_.load();

  String msg;
  serializeJson(doc, msg);
  esp_websocket_client_send_text(control_ws_, msg.c_str(), msg.length(),
                                 portMAX_DELAY);
}

void BLESignalKGateway::handle_control_ws_message(uint8_t* payload,
                                                  size_t length) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    ESP_LOGW(kTag, "Control WS JSON parse error: %s", err.c_str());
    return;
  }
  const char* type = doc["type"];
  if (type == nullptr) {
    return;
  }
  if (strcmp(type, "hello_ack") == 0) {
    ESP_LOGI(kTag, "Hello acknowledged by server");
  } else if (strcmp(type, "gatt_subscribe") == 0) {
    handle_gatt_subscribe(doc);
  } else if (strcmp(type, "gatt_write") == 0) {
    handle_gatt_write(doc);
  } else if (strcmp(type, "gatt_close") == 0) {
    handle_gatt_close(doc);
  } else {
    ESP_LOGD(kTag, "Unhandled control message type: %s", type);
  }
}

void BLESignalKGateway::restore_pending_capacity() {
  if (xSemaphoreTake(pending_ads_mutex_, pdMS_TO_TICKS(200)) != pdTRUE) {
    // Next cycle restores it. The buffer still works at zero capacity; it
    // just regrows in steps until then.
    return;
  }
  pending_ads_.reserve(config_.max_pending_ads);
  BLE_GW_HEAP_PROBE("capacity.restored", 0, pending_ads_.capacity());
  xSemaphoreGive(pending_ads_mutex_);
}

bool BLESignalKGateway::post_pending_advertisements(bool allow_empty) {
  if (!sk_connected_.load()) {
    return false;
  }

  String token = sk_client_->get_auth_token();
  String addr = sk_client_->get_server_address();
  uint16_t port = sk_client_->get_server_port();
  if (addr.length() == 0 || port == 0) {
    return false;
  }
  // An empty token is valid when the server has security disabled
  // (signalk-server returns 404 on access-request and SKWSClient
  // proceeds without a token). In that case we still POST but
  // without the Authorization header.

  // Drain the pending buffer under the mutex.
  BLE_GW_HEAP_PROBE("drain.pre", pending_ads_.size(), pending_ads_.capacity());
  // Declared before to_post so it destructs after it: the batch's array is
  // already freed by the time the buffer's is allocated, so only one of the
  // two is ever live. Covers every return path out of this function.
  struct CapacityRestorer {
    BLESignalKGateway* self;
    ~CapacityRestorer() { self->restore_pending_capacity(); }
  } capacity_restorer{this};

  std::vector<BLEAdvertisement> to_post;
  if (xSemaphoreTake(pending_ads_mutex_, pdMS_TO_TICKS(200)) != pdTRUE) {
    return false;
  }
  to_post.swap(pending_ads_);
  // Enough capacity to absorb the advertisements that arrive before the full
  // reservation is restored, so the GAP callback does not have to grow the
  // element array. Each buffered advertisement still allocates its own name
  // string and payload vector there, as it always has; only the array is
  // covered here.
  pending_ads_.reserve(config_.max_pending_ads < kDrainWindowReserve
                           ? config_.max_pending_ads
                           : kDrainWindowReserve);
  BLE_GW_HEAP_PROBE("drain.swapped", to_post.size(), pending_ads_.capacity());
  xSemaphoreGive(pending_ads_mutex_);
  // The full reservation is deliberately deferred until the batch has been
  // serialized and freed; restore_pending_capacity() below does that. Taking it
  // here instead would hold two full element arrays at once, two contiguous
  // blocks of max_pending_ads * sizeof(BLEAdvertisement) on a heap that has
  // around 8 kB contiguous once a scan is running. The allocation then throws
  // with exceptions disabled, which is an abort() and a reboot loop.

  // Counted once here so the accounting survives releasing the batch.
  const size_t batch_size = to_post.size();

  if (to_post.empty() && !allow_empty) {
    return true;
  }

  // Backing off after an HTTP status failure. Drain anyway rather than
  // returning early: the buffer's contents are what fragment the heap, and
  // an advertisement held for minutes is stale presence data by the time
  // the endpoint starts accepting batches again.
  if (post_backoff_ms_ > 0 &&
      static_cast<long>(millis() - post_backoff_until_ms_) < 0) {
    adv_dropped_count_.fetch_add(batch_size, std::memory_order_relaxed);
    return false;
  }

  // Build the JSON batch. The document is scoped so it is destroyed the moment
  // it has been serialized: it holds a copy of every string in the batch, and
  // the POST that follows needs the space more than a tree nobody reads again.
  String body;
  BLE_GW_HEAP_PROBE("doc.pre", batch_size, pending_ads_.capacity());
  {
    JsonDocument doc;
    doc["gateway_id"] = SensESPBaseApp::get_hostname();
    doc["hostname"] = SensESPBaseApp::get_hostname();
    doc["firmware"] = config_.firmware_version.length() > 0
                          ? config_.firmware_version
                          : String(kSensESPVersion);
    doc["uptime"] = millis() / 1000;
    doc["free_heap"] = usable_free_heap();
    if (ble_provisioner_) {
      String mac = ble_provisioner_->mac_address();
      if (mac.length() > 0) {
        doc["mac"] = mac;
      }
    }
    JsonArray devices = doc["devices"].to<JsonArray>();
    for (const auto& ad : to_post) {
      JsonObject dev = devices.add<JsonObject>();
      dev["mac"] = ad.address;
      dev["rssi"] = ad.rssi;
      if (ad.name.length() > 0) {
        dev["name"] = ad.name;
      }
      if (!ad.adv_data.empty()) {
        dev["adv_data"] = bytes_to_hex(ad.adv_data);
      }
    }

    serializeJson(doc, body);
    BLE_GW_HEAP_PROBE("doc.live", batch_size, pending_ads_.capacity());
  }
  BLE_GW_HEAP_PROBE("body.built", batch_size, pending_ads_.capacity());

  // The batch is serialized into body and nothing downstream reads it, so
  // release it and restore the reservation now rather than at function exit.
  // That returns the element array, its strings and its payload vectors before
  // the POST needs buffers of its own, and it ends the reduced-capacity window
  // here instead of after a POST that can take seconds. The guard above still
  // covers the early returns that never reach this point.
  { std::vector<BLEAdvertisement>().swap(to_post); }
  restore_pending_capacity();

  String ca_pem;
  String common_name;
  const Transport transport = resolve_transport(ca_pem, common_name);
  if (transport == Transport::kUnavailable) {
    http_post_fail_.fetch_add(1, std::memory_order_relaxed);
    adv_dropped_count_.fetch_add(batch_size, std::memory_order_relaxed);
    return false;
  }
  const bool use_tls = transport == Transport::kTls;

  String url = String(use_tls ? "https://" : "http://") + addr + ":" +
               String(port) + kAdvertisementsPath;

  if (!ensure_post_client(url, ca_pem, common_name)) {
    http_post_fail_.fetch_add(1, std::memory_order_relaxed);
    adv_dropped_count_.fetch_add(batch_size, std::memory_order_relaxed);
    return false;
  }

  esp_http_client_set_header(post_client_, "Content-Type", "application/json");
  if (token.length() > 0) {
    String auth = String("Bearer ") + token;
    esp_http_client_set_header(post_client_, "Authorization", auth.c_str());
  } else {
    // Headers live on the handle, and the handle now outlives the
    // request. Without this an old bearer is replayed after the token
    // is cleared.
    esp_http_client_delete_header(post_client_, "Authorization");
  }
  esp_http_client_set_post_field(post_client_, body.c_str(), body.length());

  const esp_err_t err = esp_http_client_perform(post_client_);
  const int code =
      err == ESP_OK ? esp_http_client_get_status_code(post_client_) : -err;

  if (code == 200) {
    post_backoff_ms_ = 0;
    adv_posted_count_.fetch_add(batch_size, std::memory_order_relaxed);
    http_post_success_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGI(kTag, "POST: forwarded %u adv, heap=%u",
             static_cast<unsigned>(batch_size),
             static_cast<unsigned>(usable_free_heap()));
    return true;
  }

  http_post_fail_.fetch_add(1, std::memory_order_relaxed);
  adv_dropped_count_.fetch_add(batch_size, std::memory_order_relaxed);

  if (err != ESP_OK) {
    ESP_LOGW(kTag, "POST failed: %s, heap=%u", esp_err_to_name(err),
             static_cast<unsigned>(usable_free_heap()));
    // A kept-alive handle whose socket has died stays dead: esp_http_client
    // does not reset its state machine after a mid-request failure. Drop it
    // now rather than spending further intervals, and their timeouts, posting
    // through a handle that cannot succeed.
    destroy_post_client();
    return false;
  }

  if (code == 401 || code == 403) {
    // Deliberately does not restart the Signal K client. That client is
    // shared with the rest of the device, SKWSClient documents restart()
    // as event-loop-only, and a server that authenticates the delta
    // socket while refusing this endpoint would otherwise have every
    // sensor on the device torn down once per post interval.
    ESP_LOGW(kTag,
             "POST: auth rejected (HTTP %d); the token is not accepted on "
             "the BLE provider endpoint",
             code);
  } else {
    ESP_LOGW(kTag, "POST failed: HTTP %d, heap=%u", code,
             static_cast<unsigned>(usable_free_heap()));
  }

  // The server answered, so the transport is healthy and rebuilding it would
  // only spend a fresh handshake on a rejection that stands regardless. Slow
  // the cadence instead.
  post_backoff_ms_ = post_backoff_ms_ == 0
                         ? config_.post_interval_ms * 2
                         : post_backoff_ms_ * 2;
  if (post_backoff_ms_ > kPostBackoffCapMs) {
    post_backoff_ms_ = kPostBackoffCapMs;
  }
  post_backoff_until_ms_ = millis() + post_backoff_ms_;
  ESP_LOGW(kTag, "POST: next attempt in %u ms",
           static_cast<unsigned>(post_backoff_ms_));
  return false;
}

// -----------------------------------------------------------------
// GATT session management
// -----------------------------------------------------------------

void BLESignalKGateway::send_control_json(JsonDocument& doc) {
  if (!ws_connected_.load() || control_ws_ == nullptr) return;
  String msg;
  serializeJson(doc, msg);
  esp_websocket_client_send_text(control_ws_, msg.c_str(), msg.length(),
                                 portMAX_DELAY);
}

namespace {
// Hex-decode a string like "0102ab" into bytes.
std::vector<uint8_t> hex_decode(const char* hex) {
  std::vector<uint8_t> out;
  if (!hex) return out;
  size_t len = strlen(hex);
  out.reserve(len / 2);
  for (size_t i = 0; i + 1 < len; i += 2) {
    char buf[3] = {hex[i], hex[i + 1], 0};
    out.push_back(static_cast<uint8_t>(strtoul(buf, nullptr, 16)));
  }
  return out;
}

String hex_encode(const uint8_t* data, size_t len) {
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    char tmp[3];
    snprintf(tmp, sizeof(tmp), "%02x", data[i]);
    out += tmp;
  }
  return out;
}
}  // namespace

// Context struct passed to FreeRTOS timer callbacks via pvTimerGetTimerID().
struct GATTTimerContext {
  BLESignalKGateway* gateway;
  String session_id;
  String char_uuid;
  std::vector<uint8_t> write_data;  // Only for periodic_write timers.
};

void BLESignalKGateway::handle_gatt_subscribe(JsonDocument& doc) {
  const char* session_id = doc["session_id"];
  const char* mac = doc["mac"];
  const char* service = doc["service"];

  if (!session_id || !mac || !service) {
    ESP_LOGW(kTag, "gatt_subscribe: missing required fields");
    return;
  }
  if (!ble_provisioner_) {
    ESP_LOGE(kTag, "gatt_subscribe: no BLE provisioner");
    return;
  }

  int max = ble_provisioner_->max_gatt_connections();
  if (max <= 0) {
    JsonDocument err;
    err["type"] = "gatt_error";
    err["session_id"] = session_id;
    err["error"] = "GATT not supported on this gateway";
    send_control_json(err);
    return;
  }

  if (xSemaphoreTake(gatt_sessions_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }
  if (static_cast<int>(gatt_sessions_.size()) >= max) {
    xSemaphoreGive(gatt_sessions_mutex_);
    JsonDocument err;
    err["type"] = "gatt_error";
    err["session_id"] = session_id;
    err["error"] = "No GATT slots available";
    send_control_json(err);
    return;
  }

  std::unique_ptr<GATTSession> session(new GATTSession());
  session->session_id = session_id;
  session->mac = mac;
  session->service_uuid = service;

  // Parse notify array.
  JsonArray notify = doc["notify"];
  for (JsonVariant v : notify) {
    session->notify_uuids.push_back(String(v.as<const char*>()));
  }

  // Parse init writes.
  JsonArray init = doc["init"];
  for (JsonObject obj : init) {
    InitWrite iw;
    iw.char_uuid = String(obj["uuid"].as<const char*>());
    iw.data = hex_decode(obj["data"].as<const char*>());
    session->init_writes.push_back(std::move(iw));
  }

  // Parse poll descriptors.
  JsonArray poll = doc["poll"];
  for (JsonObject obj : poll) {
    PollDescriptor pd;
    pd.char_uuid = String(obj["uuid"].as<const char*>());
    pd.interval_ms = obj["interval_ms"];
    session->polls.push_back(std::move(pd));
  }

  // Parse periodic write descriptors.
  JsonArray pw = doc["periodic_write"];
  for (JsonObject obj : pw) {
    PeriodicWriteDescriptor pwd;
    pwd.char_uuid = String(obj["uuid"].as<const char*>());
    pwd.data = hex_decode(obj["data"].as<const char*>());
    pwd.interval_ms = obj["interval_ms"];
    session->periodic_writes.push_back(std::move(pwd));
  }

  String sid = session->session_id;
  GATTSession* raw_session = session.get();
  gatt_sessions_[sid] = std::move(session);
  xSemaphoreGive(gatt_sessions_mutex_);

  // Stop scanning during connection establishment.
  scan_suppressed_.store(true);
  ble_provisioner_->stop_scan();

  ESP_LOGI(kTag, "GATT subscribe: session=%s mac=%s service=%s",
           sid.c_str(), mac, service);

  GATTConnectionCallbacks callbacks;

  callbacks.on_connected = [this, sid]() {
    ESP_LOGI(kTag, "GATT connected: session=%s", sid.c_str());
    if (xSemaphoreTake(gatt_sessions_mutex_, pdMS_TO_TICKS(100)) != pdTRUE)
      return;
    auto it = gatt_sessions_.find(sid);
    if (it == gatt_sessions_.end()) {
      xSemaphoreGive(gatt_sessions_mutex_);
      return;
    }
    GATTSession* s = it->second.get();
    s->state = GATTSessionState::kInitializing;
    xSemaphoreGive(gatt_sessions_mutex_);
    gatt_run_init_writes(s);
  };

  callbacks.on_disconnected = [this, sid](const String& reason) {
    ESP_LOGW(kTag, "GATT disconnected: session=%s reason=%s",
             sid.c_str(), reason.c_str());
    JsonDocument msg;
    msg["type"] = "gatt_disconnected";
    msg["session_id"] = sid;
    msg["reason"] = reason;
    send_control_json(msg);
    gatt_cleanup_session(sid);
  };

  callbacks.on_notify = [this, sid](const String& char_uuid,
                                    const uint8_t* data, size_t len) {
    JsonDocument msg;
    msg["type"] = "gatt_data";
    msg["session_id"] = sid;
    msg["uuid"] = char_uuid;
    msg["data"] = hex_encode(data, len);
    send_control_json(msg);
  };

  callbacks.on_read = [this, sid](const String& char_uuid,
                                  const uint8_t* data, size_t len) {
    JsonDocument msg;
    msg["type"] = "gatt_data";
    msg["session_id"] = sid;
    msg["uuid"] = char_uuid;
    msg["data"] = hex_encode(data, len);
    send_control_json(msg);
  };

  callbacks.on_write_complete = [this, sid](const String& char_uuid,
                                            bool success) {
    if (!success) {
      ESP_LOGW(kTag, "GATT write failed: session=%s char=%s",
               sid.c_str(), char_uuid.c_str());
    }
    // Advance init write sequence if in init phase.
    if (xSemaphoreTake(gatt_sessions_mutex_, pdMS_TO_TICKS(50)) != pdTRUE)
      return;
    auto it = gatt_sessions_.find(sid);
    if (it != gatt_sessions_.end() &&
        it->second->state == GATTSessionState::kInitializing) {
      GATTSession* s = it->second.get();
      s->init_write_index++;
      xSemaphoreGive(gatt_sessions_mutex_);
      gatt_run_init_writes(s);
    } else {
      xSemaphoreGive(gatt_sessions_mutex_);
    }
  };

  callbacks.on_error = [this, sid](const String& error) {
    ESP_LOGE(kTag, "GATT error: session=%s: %s", sid.c_str(), error.c_str());
    JsonDocument msg;
    msg["type"] = "gatt_error";
    msg["session_id"] = sid;
    msg["error"] = error;
    send_control_json(msg);
    gatt_cleanup_session(sid);
  };

  int handle = ble_provisioner_->gatt_connect(
      String(mac), 0, String(service), std::move(callbacks));
  if (handle < 0) {
    JsonDocument err;
    err["type"] = "gatt_error";
    err["session_id"] = sid;
    err["error"] = "gatt_connect failed";
    send_control_json(err);
    gatt_cleanup_session(sid);
    return;
  }
  raw_session->conn_handle = handle;
}

void BLESignalKGateway::gatt_run_init_writes(GATTSession* session) {
  if (session->init_write_index >= session->init_writes.size()) {
    // All init writes done — move to subscribing.
    session->state = GATTSessionState::kSubscribing;
    session->subscribe_index = 0;
    gatt_run_subscribes(session);
    return;
  }
  auto& iw = session->init_writes[session->init_write_index];
  ESP_LOGI(kTag, "GATT init write %u/%u: %s",
           (unsigned)(session->init_write_index + 1),
           (unsigned)session->init_writes.size(),
           iw.char_uuid.c_str());
  ble_provisioner_->gatt_write(session->conn_handle, iw.char_uuid,
                               iw.data.data(), iw.data.size());
  // on_write_complete callback will advance init_write_index and call us again.
}

void BLESignalKGateway::gatt_run_subscribes(GATTSession* session) {
  if (session->subscribe_index >= session->notify_uuids.size()) {
    // All subscriptions done — session is active.
    session->state = GATTSessionState::kActive;
    gatt_start_timers(session);

    // Resume scanning.
    scan_suppressed_.store(false);
    if (ble_provisioner_ && !ble_provisioner_->is_scanning()) {
      ble_provisioner_->start_scan();
    }

    // Notify server.
    JsonDocument msg;
    msg["type"] = "gatt_connected";
    msg["session_id"] = session->session_id;
    msg["mac"] = session->mac;
    send_control_json(msg);

    ESP_LOGI(kTag, "GATT session %s fully active",
             session->session_id.c_str());
    return;
  }
  const String& uuid = session->notify_uuids[session->subscribe_index];
  ESP_LOGI(kTag, "GATT subscribe notify %u/%u: %s",
           (unsigned)(session->subscribe_index + 1),
           (unsigned)session->notify_uuids.size(), uuid.c_str());
  ble_provisioner_->gatt_subscribe_notify(session->conn_handle, uuid);
  session->subscribe_index++;
  // Notifications are async — just proceed to the next one immediately.
  // The Bluedroid REG_FOR_NOTIFY callback is fire-and-forget.
  gatt_run_subscribes(session);
}

void BLESignalKGateway::gatt_start_timers(GATTSession* session) {
  // Poll timers — periodically read a characteristic.
  for (auto& pd : session->polls) {
    if (pd.interval_ms < kMinGattIntervalMs) {
      // Comes straight off the control WebSocket. A zero period trips
      // FreeRTOS's own assertion and reboots the device.
      ESP_LOGW(kTag, "Ignoring poll descriptor with interval_ms=%u",
               static_cast<unsigned>(pd.interval_ms));
      continue;
    }
    auto* ctx = new GATTTimerContext{this, session->session_id,
                                    pd.char_uuid, {}};
    TimerHandle_t t = xTimerCreate(
        "gatt_poll", pdMS_TO_TICKS(pd.interval_ms), pdTRUE, ctx,
        &BLESignalKGateway::poll_timer_cb);
    if (t) {
      xTimerStart(t, 0);
      session->timers.push_back(t);
    } else {
      // Only the timer owns ctx, and cleanup reaches it through
      // session->timers.
      delete ctx;
    }
  }

  // Periodic write timers.
  for (auto& pw : session->periodic_writes) {
    if (pw.interval_ms < kMinGattIntervalMs) {
      ESP_LOGW(kTag, "Ignoring periodic write with interval_ms=%u",
               static_cast<unsigned>(pw.interval_ms));
      continue;
    }
    auto* ctx = new GATTTimerContext{this, session->session_id,
                                    pw.char_uuid, pw.data};
    TimerHandle_t t = xTimerCreate(
        "gatt_pw", pdMS_TO_TICKS(pw.interval_ms), pdTRUE, ctx,
        &BLESignalKGateway::periodic_write_timer_cb);
    if (t) {
      xTimerStart(t, 0);
      session->timers.push_back(t);
    } else {
      delete ctx;
    }
  }
}

void BLESignalKGateway::poll_timer_cb(TimerHandle_t timer) {
  auto* ctx = static_cast<GATTTimerContext*>(pvTimerGetTimerID(timer));
  if (!ctx || !ctx->gateway || !ctx->gateway->ble_provisioner_) return;

  if (xSemaphoreTake(ctx->gateway->gatt_sessions_mutex_,
                     pdMS_TO_TICKS(50)) != pdTRUE)
    return;
  auto it = ctx->gateway->gatt_sessions_.find(ctx->session_id);
  if (it == ctx->gateway->gatt_sessions_.end() ||
      it->second->state != GATTSessionState::kActive) {
    xSemaphoreGive(ctx->gateway->gatt_sessions_mutex_);
    return;
  }
  int handle = it->second->conn_handle;
  xSemaphoreGive(ctx->gateway->gatt_sessions_mutex_);

  ctx->gateway->ble_provisioner_->gatt_read(handle, ctx->char_uuid);
}

void BLESignalKGateway::periodic_write_timer_cb(TimerHandle_t timer) {
  auto* ctx = static_cast<GATTTimerContext*>(pvTimerGetTimerID(timer));
  if (!ctx || !ctx->gateway || !ctx->gateway->ble_provisioner_) return;

  if (xSemaphoreTake(ctx->gateway->gatt_sessions_mutex_,
                     pdMS_TO_TICKS(50)) != pdTRUE)
    return;
  auto it = ctx->gateway->gatt_sessions_.find(ctx->session_id);
  if (it == ctx->gateway->gatt_sessions_.end() ||
      it->second->state != GATTSessionState::kActive) {
    xSemaphoreGive(ctx->gateway->gatt_sessions_mutex_);
    return;
  }
  int handle = it->second->conn_handle;
  xSemaphoreGive(ctx->gateway->gatt_sessions_mutex_);

  ctx->gateway->ble_provisioner_->gatt_write(
      handle, ctx->char_uuid, ctx->write_data.data(),
      ctx->write_data.size());
}

void BLESignalKGateway::handle_gatt_write(JsonDocument& doc) {
  const char* session_id = doc["session_id"];
  const char* uuid = doc["uuid"];
  const char* data_hex = doc["data"];
  if (!session_id || !uuid || !data_hex) return;

  if (xSemaphoreTake(gatt_sessions_mutex_, pdMS_TO_TICKS(100)) != pdTRUE)
    return;
  auto it = gatt_sessions_.find(String(session_id));
  if (it == gatt_sessions_.end()) {
    xSemaphoreGive(gatt_sessions_mutex_);
    return;
  }
  int handle = it->second->conn_handle;
  xSemaphoreGive(gatt_sessions_mutex_);

  auto data = hex_decode(data_hex);
  ble_provisioner_->gatt_write(handle, String(uuid), data.data(),
                               data.size());
}

void BLESignalKGateway::handle_gatt_close(JsonDocument& doc) {
  const char* session_id = doc["session_id"];
  if (!session_id) return;

  String sid(session_id);
  ESP_LOGI(kTag, "GATT close: session=%s", sid.c_str());

  if (xSemaphoreTake(gatt_sessions_mutex_, pdMS_TO_TICKS(100)) != pdTRUE)
    return;
  auto it = gatt_sessions_.find(sid);
  if (it == gatt_sessions_.end()) {
    xSemaphoreGive(gatt_sessions_mutex_);
    return;
  }
  int handle = it->second->conn_handle;
  it->second->state = GATTSessionState::kDisconnecting;
  xSemaphoreGive(gatt_sessions_mutex_);

  if (ble_provisioner_) {
    ble_provisioner_->gatt_disconnect(handle);
  }

  JsonDocument msg;
  msg["type"] = "gatt_disconnected";
  msg["session_id"] = sid;
  msg["reason"] = "closed_by_server";
  send_control_json(msg);

  gatt_cleanup_session(sid);
}

void BLESignalKGateway::gatt_cleanup_session(const String& session_id) {
  if (xSemaphoreTake(gatt_sessions_mutex_, pdMS_TO_TICKS(200)) != pdTRUE)
    return;
  auto it = gatt_sessions_.find(session_id);
  if (it != gatt_sessions_.end()) {
    // Delete timers and free timer contexts.
    for (auto t : it->second->timers) {
      auto* ctx = static_cast<GATTTimerContext*>(pvTimerGetTimerID(t));
      xTimerStop(t, 0);
      xTimerDelete(t, 0);
      delete ctx;
    }
    it->second->timers.clear();
    gatt_sessions_.erase(it);
  }
  xSemaphoreGive(gatt_sessions_mutex_);

  // Resume scanning if no GATT sessions remain.
  if (gatt_sessions_.empty()) {
    scan_suppressed_.store(false);
    if (ble_provisioner_ && !ble_provisioner_->is_scanning()) {
      ble_provisioner_->start_scan();
    }
  }
}

// -----------------------------------------------------------------

void BLESignalKGateway::post_task_entry(void* arg) {
  static_cast<BLESignalKGateway*>(arg)->post_task_loop();
}

void BLESignalKGateway::post_task_loop() {
  // Open the delivery connection before the scanner runs. Over TLS the
  // handshake needs a contiguous block of tens of kilobytes, and the
  // advertisement flood fragments the heap within seconds, so a client
  // that first connects afterwards may never connect at all. Priming it
  // here costs one empty batch and, with keep-alive, holds for the life
  // of the device.
  //
  // Bounded: on a server that never accepts us, scanning still starts,
  // because a gateway that scans and drops is more useful for
  // diagnosis than one that does nothing at all.
  static constexpr int kPrimeAttempts = 20;
  for (int attempt = 0; attempt < kPrimeAttempts; ++attempt) {
    if (!post_task_should_run_.load()) {
      break;
    }
    if (sk_connected_.load() && post_pending_advertisements(true)) {
      ESP_LOGI(kTag, "Delivery connection established — starting scan");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  // Only reached on a normal exit from the priming loop: a gateway
  // stopped mid-prime must not turn the radio on behind stop()'s back.
  if (post_task_should_run_.load() && ble_provisioner_ &&
      !ble_provisioner_->is_scanning()) {
    if (!ble_provisioner_->start_scan()) {
      ESP_LOGE(kTag, "start_scan() failed");
    }
  }

  // Set once a controller reset actually succeeds. Only the esp_hosted
  // provisioner implements those hooks; the native ones inherit no-ops,
  // and rebooting because a quiet anchorage produced no advertisements
  // is not a recovery.
  bool provisioner_can_reset = false;

  unsigned long last_status_ms = 0;
  uint32_t last_known_hits = 0;
  unsigned long last_hit_change_ms = millis();
  static constexpr unsigned long kScanWatchdogMs = 30000;
  int consecutive_restarts = 0;

  while (post_task_should_run_.load()) {
    vTaskDelay(pdMS_TO_TICKS(config_.post_interval_ms));
    post_pending_advertisements();

    unsigned long now = millis();

    // Scan watchdog: if no new advertisements have arrived in
    // kScanWatchdogMs, escalate recovery:
    //   Level 1: scan stop/start (HCI command level)
    //   Level 2: BT controller reset via RPC to C6
    //   Level 3: GPIO hard-reset of the C6 chip (power-cycles it)
    //   Level 4: full ESP.restart()
    if (ble_provisioner_) {
      uint32_t current_hits = ble_provisioner_->scan_hit_count();
      if (current_hits != last_known_hits) {
        last_known_hits = current_hits;
        last_hit_change_ms = now;
        consecutive_restarts = 0;
      } else if (now - last_hit_change_ms > kScanWatchdogMs &&
                 ble_provisioner_->is_scanning() &&
                 !scan_suppressed_.load()) {
        consecutive_restarts++;
        if (consecutive_restarts == 1) {
          // Level 1: simple scan restart (HCI level).
          ESP_LOGW(kTag,
                   "Scan watchdog: no hits in %lu ms — restarting scan "
                   "(level 1)",
                   now - last_hit_change_ms);
          ble_provisioner_->stop_scan();
          vTaskDelay(pdMS_TO_TICKS(500));
          ble_provisioner_->start_scan();
        } else if (consecutive_restarts == 2) {
          // Level 2: BT controller reset via RPC to C6.
          ESP_LOGW(kTag,
                   "Scan watchdog: level 1 failed — RPC BT controller "
                   "reset (level 2)");
          ble_provisioner_->stop_scan();
          vTaskDelay(pdMS_TO_TICKS(200));
          if (ble_provisioner_->reset_bt_controller()) {
            provisioner_can_reset = true;
            vTaskDelay(pdMS_TO_TICKS(500));
          } else {
            // Only the esp_hosted provisioner implements this; the
            // native ones inherit a no-op. Leaving the scan stopped
            // here would be permanent, because the escalation below is
            // guarded on is_scanning() and could never fire again.
            ESP_LOGE(kTag, "BT controller reset unavailable or failed");
          }
          ble_provisioner_->start_scan();
        } else if (consecutive_restarts == 3) {
          // Level 3: GPIO hard-reset of the C6 chip. This power-
          // cycles the entire C6, clearing whatever stuck HCI state
          // is silently eating advertising reports.
          ESP_LOGW(kTag,
                   "Scan watchdog: level 2 failed — GPIO hard-reset "
                   "of C6 (level 3)");
          if (ble_provisioner_->hard_reset_c6()) {
            provisioner_can_reset = true;
            vTaskDelay(pdMS_TO_TICKS(500));
          } else {
            ESP_LOGE(kTag, "Co-processor hard reset unavailable or failed");
          }
          ble_provisioner_->start_scan();
        } else {
          // Level 4: reboot. Only worth doing where the escalation
          // above could actually act — on a provisioner with no reset
          // hooks nothing has been tried but a scan restart, and an
          // environment with no advertisers in range is not a fault.
          if (!provisioner_can_reset) {
            ESP_LOGW(kTag,
                     "Scan watchdog: %d restarts with no advertisements; "
                     "the provisioner has no controller reset, so not "
                     "rebooting",
                     consecutive_restarts);
            consecutive_restarts = 0;
          } else {
            ESP_LOGE(kTag,
                     "Scan watchdog: %d consecutive failures — rebooting "
                     "(level 4)",
                     consecutive_restarts);
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP.restart();
          }
        }
        last_hit_change_ms = now;
      }
    }

    if (now - last_status_ms >= config_.status_interval_ms) {
      send_status();
      last_status_ms = now;
    }
  }
  // Released here rather than in stop() so the client is only ever
  // touched by this task.
  destroy_post_client();

  // Publish the exit before self-deleting so stop() can stop waiting.
  post_task_exited_.store(true);

  // Self-delete so we do not leak a FreeRTOS task handle after stop().
  vTaskDelete(nullptr);
}

void BLESignalKGateway::control_ws_event_trampoline(void* handler_args,
                                                    esp_event_base_t /*base*/,
                                                    int32_t event_id,
                                                    void* event_data) {
  static_cast<BLESignalKGateway*>(handler_args)
      ->handle_control_ws_event(event_id, event_data);
}

void BLESignalKGateway::handle_control_ws_event(int32_t event_id,
                                                void* event_data) {
  auto* data = static_cast<esp_websocket_event_data_t*>(event_data);
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI(kTag, "Control WS connected");
      ws_connected_.store(true);
      ws_connected_count_.fetch_add(1, std::memory_order_relaxed);
      send_hello();
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGW(kTag, "Control WS disconnected — will reconnect");
      ws_connected_.store(false);
      break;
    case WEBSOCKET_EVENT_CLOSED:
      ESP_LOGW(kTag, "Control WS closed — will reconnect");
      ws_connected_.store(false);
      break;
    case WEBSOCKET_EVENT_DATA:
      if (data->op_code == 0x01 && data->data_len > 0) {
        // data_ptr is const char*; cast away the const for the JSON
        // parser which does not mutate the buffer.
        handle_control_ws_message(
            reinterpret_cast<uint8_t*>(const_cast<char*>(data->data_ptr)),
            data->data_len);
      }
      break;
    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGW(kTag, "Control WS error");
      ws_connected_.store(false);
      break;
    default:
      break;
  }
}

}  // namespace sensesp
