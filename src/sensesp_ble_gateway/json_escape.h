#ifndef SENSESP_BLE_GATEWAY_JSON_ESCAPE_H_
#define SENSESP_BLE_GATEWAY_JSON_ESCAPE_H_

#include <cstddef>
#include <string>

namespace sensesp {

/**
 * @brief Append @p value to @p out as a JSON string, quotes included.
 *
 * Escapes everything RFC 8259 requires: the quote, the backslash, and every
 * character below 0x20 — the five with short forms as `\b`, `\t`, `\n`, `\f`,
 * `\r`, the rest as `\u00xx`. Bytes at or above 0x20 are copied unchanged, so
 * UTF-8 passes through and is neither validated nor re-encoded.
 *
 * ArduinoJson is not used for this. Its serializer leaves 0x01-0x07, 0x0B and
 * 0x0E-0x1F unescaped, which a conforming parser rejects; a BLE peer chooses
 * its own advertised name, so that is reachable from outside.
 */
void json_append_quoted(std::string& out, const char* value, size_t length);

/// Bytes json_append_quoted() would append for the same input, quotes
/// included. Lets a caller size a buffer exactly instead of growing it.
size_t json_quoted_length(const char* value, size_t length);

}  // namespace sensesp

#endif  // SENSESP_BLE_GATEWAY_JSON_ESCAPE_H_
