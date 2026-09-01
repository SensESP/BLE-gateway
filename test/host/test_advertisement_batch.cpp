// The golden bodies are what holds the batch shape: the writer emits JSON as
// literals, so nothing else would catch a lost comma or brace. Every optional
// field and the device separator get a case.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "sensesp_ble_gateway/advertisement_batch.h"

namespace {

int failures = 0;

void expect_eq(const std::string& got, const std::string& want,
               const char* what) {
  if (got == want) return;
  ++failures;
  printf("FAIL %s\n  got  %s\n  want %s\n", what, got.c_str(), want.c_str());
}

sensesp::BatchHeader header_of(const std::string& id, const std::string& fw,
                               const std::string& mac, uint32_t uptime,
                               uint32_t heap) {
  return sensesp::BatchHeader{id.data(),  id.size(), fw.data(), fw.size(),
                              mac.data(), mac.size(), uptime,   heap};
}

// Runs both passes and fails if they disagree, which is the invariant the
// caller relies on to size its buffer.
std::string build(const sensesp::BatchHeader& header,
                  const std::vector<sensesp::BatchDevice>& devices,
                  const char* what) {
  const auto at = [&devices](size_t i) { return devices[i]; };
  sensesp::BatchCounter counter;
  sensesp::write_batch(counter, header, devices.size(), at);
  std::string body;
  sensesp::BatchAppender appender(body);
  sensesp::write_batch(appender, header, devices.size(), at);
  if (body.size() != counter.length()) {
    ++failures;
    printf("FAIL %s: counted %zu, wrote %zu\n", what, counter.length(),
           body.size());
  }
  return body;
}

void test_empty_batch() {
  const std::string id = "gw", fw = "1.2.3", mac = "";
  const auto h = header_of(id, fw, mac, 5, 1000);
  expect_eq(build(h, {}, "empty batch"),
            "{\"gateway_id\":\"gw\",\"hostname\":\"gw\",\"firmware\":\"1.2.3\","
            "\"uptime\":5,\"free_heap\":1000,\"devices\":[]}",
            "empty batch, no gateway mac");
}

void test_gateway_mac_present() {
  const std::string id = "gw", fw = "1.2.3", mac = "AA:BB:CC:DD:EE:FF";
  const auto h = header_of(id, fw, mac, 0, 0);
  expect_eq(build(h, {}, "mac present"),
            "{\"gateway_id\":\"gw\",\"hostname\":\"gw\",\"firmware\":\"1.2.3\","
            "\"uptime\":0,\"free_heap\":0,\"mac\":\"AA:BB:CC:DD:EE:FF\","
            "\"devices\":[]}",
            "gateway mac included when non-empty");
}

void test_minimal_device() {
  const std::string id = "g", fw = "f", mac = "";
  const std::string dmac = "11:22:33:44:55:66";
  const auto h = header_of(id, fw, mac, 1, 2);
  std::vector<sensesp::BatchDevice> d{
      {dmac.data(), dmac.size(), -70, nullptr, 0, nullptr, 0}};
  expect_eq(build(h, d, "minimal device"),
            "{\"gateway_id\":\"g\",\"hostname\":\"g\",\"firmware\":\"f\","
            "\"uptime\":1,\"free_heap\":2,\"devices\":["
            "{\"mac\":\"11:22:33:44:55:66\",\"rssi\":-70}]}",
            "device with neither name nor adv_data, negative rssi");
}

void test_full_device() {
  const std::string id = "g", fw = "f", mac = "";
  const std::string dmac = "11:22:33:44:55:66", name = "Buoy";
  const std::vector<uint8_t> adv{0x02, 0x01, 0x06, 0xff};
  const auto h = header_of(id, fw, mac, 1, 2);
  std::vector<sensesp::BatchDevice> d{{dmac.data(), dmac.size(), 0,
                                       name.data(), name.size(), adv.data(),
                                       adv.size()}};
  expect_eq(build(h, d, "full device"),
            "{\"gateway_id\":\"g\",\"hostname\":\"g\",\"firmware\":\"f\","
            "\"uptime\":1,\"free_heap\":2,\"devices\":["
            "{\"mac\":\"11:22:33:44:55:66\",\"rssi\":0,\"name\":\"Buoy\","
            "\"adv_data\":\"020106FF\"}]}",
            "name and uppercase hex adv_data");
}

// The separator between devices is hand-written, so it gets its own case.
void test_two_devices() {
  const std::string id = "g", fw = "f", mac = "";
  const std::string a = "AA", b = "BB";
  const auto h = header_of(id, fw, mac, 0, 0);
  std::vector<sensesp::BatchDevice> d{
      {a.data(), a.size(), 1, nullptr, 0, nullptr, 0},
      {b.data(), b.size(), 2, nullptr, 0, nullptr, 0}};
  expect_eq(build(h, d, "two devices"),
            "{\"gateway_id\":\"g\",\"hostname\":\"g\",\"firmware\":\"f\","
            "\"uptime\":0,\"free_heap\":0,\"devices\":["
            "{\"mac\":\"AA\",\"rssi\":1},{\"mac\":\"BB\",\"rssi\":2}]}",
            "comma between devices, none trailing");
}

// The reason the escaper exists, exercised through the batch rather than
// directly: a peer-chosen name must not be able to close the string.
void test_hostile_name_in_batch() {
  const std::string id = "g", fw = "f", mac = "";
  const std::string dmac = "AA";
  const std::string name = std::string("x\",\"rssi\":0,\"z\":\"") + '\x01';
  const auto h = header_of(id, fw, mac, 0, 0);
  std::vector<sensesp::BatchDevice> d{
      {dmac.data(), dmac.size(), -1, name.data(), name.size(), nullptr, 0}};
  const std::string body = build(h, d, "hostile name");
  expect_eq(body,
            "{\"gateway_id\":\"g\",\"hostname\":\"g\",\"firmware\":\"f\","
            "\"uptime\":0,\"free_heap\":0,\"devices\":["
            "{\"mac\":\"AA\",\"rssi\":-1,"
            "\"name\":\"x\\\",\\\"rssi\\\":0,\\\"z\\\":\\\"\\u0001\"}]}",
            "peer name cannot inject structure");
}

void test_extremes() {
  const std::string id = "g", fw = "f", mac = "";
  const std::string dmac = "AA";
  const auto h = header_of(id, fw, mac, 4294967295u, 4294967295u);
  std::vector<sensesp::BatchDevice> d{
      {dmac.data(), dmac.size(), -128, nullptr, 0, nullptr, 0}};
  expect_eq(build(h, d, "extremes"),
            "{\"gateway_id\":\"g\",\"hostname\":\"g\",\"firmware\":\"f\","
            "\"uptime\":4294967295,\"free_heap\":4294967295,\"devices\":["
            "{\"mac\":\"AA\",\"rssi\":-128}]}",
            "uint32 maxima stay unsigned, rssi floor");
}

// A count that disagrees with what is written makes the caller's buffer
// reallocate, which is the whole thing the two-pass design avoids.
void test_counter_matches_appender_over_many_batches() {
  uint32_t seed = 12345;
  const auto next = [&seed]() {
    seed = seed * 1103515245u + 12345u;
    return (seed >> 16) & 0x7fff;
  };
  for (int round = 0; round < 500; ++round) {
    const size_t count = next() % 8;
    std::vector<std::string> macs(count), names(count);
    std::vector<std::vector<uint8_t>> advs(count);
    std::vector<sensesp::BatchDevice> devices;
    for (size_t i = 0; i < count; ++i) {
      const size_t nlen = next() % 32;
      for (size_t j = 0; j < nlen; ++j) {
        names[i].push_back(static_cast<char>(next() & 0xff));
      }
      macs[i] = "AA:BB:CC:DD:EE:FF";
      const size_t alen = next() % 32;
      for (size_t j = 0; j < alen; ++j) {
        advs[i].push_back(static_cast<uint8_t>(next() & 0xff));
      }
      devices.push_back({macs[i].data(), macs[i].size(),
                         static_cast<int>(next() % 256) - 128,
                         names[i].data(), names[i].size(), advs[i].data(),
                         advs[i].size()});
    }
    const std::string id = "gw", fw = "1.0", mac = "AA:BB:CC:DD:EE:FF";
    const auto h = header_of(id, fw, mac, next(), next() * 1000u);
    build(h, devices, "randomised batch");
  }
}

void test_number_text_clamps() {
  char buffer[sensesp::kBatchNumberMax];
  memset(buffer, 0x7e, sizeof(buffer));
  const size_t n = sensesp::batch_number_text(18446744073709551615ull, false,
                                              buffer);
  if (n >= sensesp::kBatchNumberMax) {
    ++failures;
    printf("FAIL batch_number_text returned %zu, buffer holds %zu\n", n,
           sensesp::kBatchNumberMax);
  }
  expect_eq(std::string(buffer, n), "18446744073709551615",
            "widest unsigned value fits");
}

}  // namespace

int main() {
  test_empty_batch();
  test_gateway_mac_present();
  test_minimal_device();
  test_full_device();
  test_two_devices();
  test_hostile_name_in_batch();
  test_extremes();
  test_counter_matches_appender_over_many_batches();
  test_number_text_clamps();
  if (failures == 0) {
    printf("advertisement_batch: all tests passed\n");
    return EXIT_SUCCESS;
  }
  printf("advertisement_batch: %d failure(s)\n", failures);
  return EXIT_FAILURE;
}
