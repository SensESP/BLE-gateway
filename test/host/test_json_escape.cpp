// Host tests for the JSON string escaper. Built and run by test/host/run.sh.
//
// The escaper exists because a BLE peer names itself and the gateway puts that
// name in a POST body, so these cases are reachable from outside the device.

#include <cstdio>
#include <cstdlib>
#include <string>

#include "sensesp_ble_gateway/json_escape.h"

namespace {

int failures = 0;

void expect_eq(const std::string& got, const std::string& want,
               const char* what) {
  if (got == want) return;
  ++failures;
  printf("FAIL %s\n  got  %s\n  want %s\n", what, got.c_str(), want.c_str());
}

std::string quoted(const std::string& in) {
  std::string out;
  sensesp::json_append_quoted(out, in.data(), in.size());
  return out;
}

void test_plain() {
  expect_eq(quoted(""), "\"\"", "empty string");
  expect_eq(quoted("Sensor 1"), "\"Sensor 1\"", "plain ascii");
  // 0x7F is legal unescaped in JSON, and / needs no escape.
  expect_eq(quoted("a/b\x7f"), "\"a/b\x7f\"", "slash and DEL pass through");
}

void test_required_escapes() {
  expect_eq(quoted("\""), "\"\\\"\"", "quote");
  expect_eq(quoted("\\"), "\"\\\\\"", "backslash");
  expect_eq(quoted("\b\t\n\f\r"), "\"\\b\\t\\n\\f\\r\"", "short escapes");
}

// The bug this escaper exists to fix: ArduinoJson emits these raw.
void test_every_control_character_is_escaped() {
  for (int c = 0x00; c < 0x20; ++c) {
    const std::string in(1, static_cast<char>(c));
    const std::string out = quoted(in);
    if (out.find(static_cast<char>(c)) != std::string::npos) {
      ++failures;
      printf("FAIL control 0x%02X emitted raw: %s\n", c, out.c_str());
      continue;
    }
    if (out.size() < 4 || out[1] != '\\') {
      ++failures;
      printf("FAIL control 0x%02X not escaped: %s\n", c, out.c_str());
    }
  }
  expect_eq(quoted(std::string(1, '\x00')), "\"\\u0000\"", "NUL");
  expect_eq(quoted("\x01"), "\"\\u0001\"", "0x01");
  expect_eq(quoted("\x0b"), "\"\\u000b\"", "0x0B");
  expect_eq(quoted("\x1f"), "\"\\u001f\"", "0x1F");
}

void test_utf8_passes_through() {
  // Multi-byte UTF-8 must not be mangled; the escaper is byte-oriented.
  expect_eq(quoted("Bojen\xc3\xa4"), "\"Bojen\xc3\xa4\"", "utf-8 kept intact");
}

void test_hostile_name() {
  // A name that closes the string, injects a field and starts a comment.
  const std::string attack = "x\",\"rssi\":0,\"a\":\"\x01";
  const std::string out = quoted(attack);
  expect_eq(out, "\"x\\\",\\\"rssi\\\":0,\\\"a\\\":\\\"\\u0001\"",
            "injection attempt is inert");
}

// A caller sizes its buffer from json_quoted_length(), so a disagreement
// between the two would either truncate or reallocate.
void test_length_matches_output() {
  std::string cases[] = {"", "plain", "\"", "\\", "\b\t\n\f\r"};
  for (int c = 0; c < 0x100; ++c) {
    const std::string in(1, static_cast<char>(c));
    std::string out;
    sensesp::json_append_quoted(out, in.data(), in.size());
    const size_t want = sensesp::json_quoted_length(in.data(), in.size());
    if (out.size() != want) {
      ++failures;
      printf("FAIL length for byte 0x%02X: appended %zu, reported %zu\n", c,
             out.size(), want);
    }
  }
  for (const std::string& in : cases) {
    std::string out;
    sensesp::json_append_quoted(out, in.data(), in.size());
    const size_t want = sensesp::json_quoted_length(in.data(), in.size());
    if (out.size() != want) {
      ++failures;
      printf("FAIL length for \"%s\": appended %zu, reported %zu\n", in.c_str(),
             out.size(), want);
    }
  }
}

void test_appends_rather_than_replaces() {
  std::string out = "{\"name\":";
  sensesp::json_append_quoted(out, "x", 1);
  out += "}";
  expect_eq(out, "{\"name\":\"x\"}", "appends to existing content");
}

}  // namespace

int main() {
  test_plain();
  test_required_escapes();
  test_every_control_character_is_escaped();
  test_utf8_passes_through();
  test_hostile_name();
  test_length_matches_output();
  test_appends_rather_than_replaces();
  if (failures == 0) {
    printf("json_escape: all tests passed\n");
    return EXIT_SUCCESS;
  }
  printf("json_escape: %d failure(s)\n", failures);
  return EXIT_FAILURE;
}
