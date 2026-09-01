#ifndef SENSESP_BLE_GATEWAY_ADVERTISEMENT_BATCH_H_
#define SENSESP_BLE_GATEWAY_ADVERTISEMENT_BATCH_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <cstddef>
#include <string>

#include "sensesp_ble_gateway/json_escape.h"

namespace sensesp {

/// Widest text a batch number can produce, including a sign and a terminator.
/// Sized for a 64-bit value so the host build, where long is wider than on the
/// targets, cannot format past the end.
constexpr size_t kBatchNumberMax = 24;

/// Renders value and returns its length. Writes into out when given one, which
/// is what lets the sizing pass and the writing pass share this function.
/// out must have room for kBatchNumberMax bytes.
inline size_t batch_number_text(uint64_t value, bool negative, char* out) {
  char buffer[kBatchNumberMax];
  const int n = negative
                    ? snprintf(buffer, sizeof(buffer), "-%llu",
                               static_cast<unsigned long long>(value))
                    : snprintf(buffer, sizeof(buffer), "%llu",
                               static_cast<unsigned long long>(value));
  // snprintf reports what it would have written, so a truncating call would
  // otherwise hand back a length past the end of buffer.
  size_t length = 0;
  if (n > 0) {
    length = static_cast<size_t>(n);
    if (length >= sizeof(buffer)) {
      length = sizeof(buffer) - 1;
    }
  }
  if (out != nullptr) {
    memcpy(out, buffer, length);
  }
  return length;
}

/// Counts the bytes write_batch() would emit, so the caller can size its
/// buffer before allocating anything.
class BatchCounter {
 public:
  template <size_t N>
  void raw(const char (&)[N]) {
    length_ += N - 1;
  }
  void quoted(const char* value, size_t length) {
    length_ += json_quoted_length(value, length);
  }
  void number(uint64_t value, bool negative) {
    length_ += batch_number_text(value, negative, nullptr);
  }
  void hex(const uint8_t*, size_t length) { length_ += 2 * length; }
  size_t length() const { return length_; }

 private:
  size_t length_ = 0;
};

/// Writes the bytes BatchCounter counted.
class BatchAppender {
 public:
  explicit BatchAppender(std::string& out) : out_(out) {}
  template <size_t N>
  void raw(const char (&literal)[N]) {
    out_.append(literal, N - 1);
  }
  void quoted(const char* value, size_t length) {
    json_append_quoted(out_, value, length);
  }
  void number(uint64_t value, bool negative) {
    char buffer[kBatchNumberMax];
    out_.append(buffer, batch_number_text(value, negative, buffer));
  }
  void hex(const uint8_t* bytes, size_t length) {
    static constexpr char kDigits[] = "0123456789ABCDEF";
    for (size_t i = 0; i < length; ++i) {
      out_.push_back(kDigits[bytes[i] >> 4]);
      out_.push_back(kDigits[bytes[i] & 0x0f]);
    }
  }

 private:
  std::string& out_;
};

/// Batch-level fields. Pointers must outlive both passes; a zero length omits
/// the optional mac.
struct BatchHeader {
  const char* gateway_id;
  size_t gateway_id_length;
  const char* firmware;
  size_t firmware_length;
  const char* mac;
  size_t mac_length;
  uint32_t uptime_s;
  uint32_t free_heap;
};

/// One advertisement. A zero length omits the optional name or adv_data.
struct BatchDevice {
  const char* mac;
  size_t mac_length;
  int rssi;
  const char* name;
  size_t name_length;
  const uint8_t* adv_data;
  size_t adv_data_length;
};

/// Writes the advertisement batch the Signal K BLE provider endpoint expects.
///
/// Run once with a BatchCounter to size the buffer, then with a BatchAppender
/// to fill it. Both passes take this one path, so the field order and the
/// optional-field decisions cannot differ between them. The byte counts can:
/// BatchCounter::quoted and BatchAppender::quoted call different functions,
/// and so do the two hex(). json_quoted_length() against json_append_quoted()
/// is what the host tests pin, and the caller compares the two totals.
///
/// device_at(i) returns the i-th BatchDevice. Taking a callback rather than a
/// container keeps this free of Arduino and IDF types, so it can be tested on
/// the host.
template <typename Sink, typename DeviceAt>
void write_batch(Sink& sink, const BatchHeader& header, size_t device_count,
                 DeviceAt device_at) {
  sink.raw("{\"gateway_id\":");
  sink.quoted(header.gateway_id, header.gateway_id_length);
  sink.raw(",\"hostname\":");
  sink.quoted(header.gateway_id, header.gateway_id_length);
  sink.raw(",\"firmware\":");
  sink.quoted(header.firmware, header.firmware_length);
  sink.raw(",\"uptime\":");
  sink.number(header.uptime_s, false);
  sink.raw(",\"free_heap\":");
  sink.number(header.free_heap, false);
  if (header.mac_length > 0) {
    sink.raw(",\"mac\":");
    sink.quoted(header.mac, header.mac_length);
  }
  sink.raw(",\"devices\":[");
  for (size_t i = 0; i < device_count; ++i) {
    if (i > 0) {
      sink.raw(",");
    }
    const BatchDevice device = device_at(i);
    sink.raw("{\"mac\":");
    sink.quoted(device.mac, device.mac_length);
    sink.raw(",\"rssi\":");
    const bool negative = device.rssi < 0;
    sink.number(negative ? static_cast<uint64_t>(-static_cast<int64_t>(
                               device.rssi))
                         : static_cast<uint64_t>(device.rssi),
                negative);
    if (device.name_length > 0) {
      sink.raw(",\"name\":");
      sink.quoted(device.name, device.name_length);
    }
    if (device.adv_data_length > 0) {
      sink.raw(",\"adv_data\":\"");
      sink.hex(device.adv_data, device.adv_data_length);
      sink.raw("\"");
    }
    sink.raw("}");
  }
  sink.raw("]}");
}

}  // namespace sensesp

#endif  // SENSESP_BLE_GATEWAY_ADVERTISEMENT_BATCH_H_
