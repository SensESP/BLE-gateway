#include "json_escape.h"

namespace sensesp {
namespace {

// Short escape for a control character, or 0 if it has none. Indexed by the
// character itself, so the five JSON short forms cost no branching.
constexpr char kShortEscape[0x20] = {
    0, 0, 0, 0, 0, 0, 0, 0, 'b', 't', 'n', 0, 'f', 'r', 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0,   0,   0,   0, 0,   0,   0, 0};

constexpr char kHexDigits[] = "0123456789abcdef";

}  // namespace

void json_append_quoted(std::string& out, const char* value, size_t length) {
  out.push_back('"');
  for (size_t i = 0; i < length; ++i) {
    const unsigned char c = static_cast<unsigned char>(value[i]);
    if (c == '"' || c == '\\') {
      out.push_back('\\');
      out.push_back(static_cast<char>(c));
    } else if (c >= 0x20) {
      out.push_back(static_cast<char>(c));
    } else if (kShortEscape[c] != 0) {
      out.push_back('\\');
      out.push_back(kShortEscape[c]);
    } else {
      out.append("\\u00");
      out.push_back(kHexDigits[c >> 4]);
      out.push_back(kHexDigits[c & 0x0f]);
    }
  }
  out.push_back('"');
}

size_t json_quoted_length(const char* value, size_t length) {
  size_t n = 2;  // the quotes
  for (size_t i = 0; i < length; ++i) {
    const unsigned char c = static_cast<unsigned char>(value[i]);
    if (c == '"' || c == '\\' || (c < 0x20 && kShortEscape[c] != 0)) {
      n += 2;
    } else if (c < 0x20) {
      n += 6;  // \u00xx
    } else {
      n += 1;
    }
  }
  return n;
}

}  // namespace sensesp
