# BLE-gateway

BLE gateway library for [SensESP](https://github.com/SignalK/SensESP). It scans
for Bluetooth Low Energy devices and feeds what it hears to
[Signal K server](https://github.com/SignalK/signalk-server)'s BLE provider API,
so a Signal K server without its own Bluetooth radio — or one out of radio range
of the devices — can still read Ruuvi tags, Victron beacons and similar
BLE sensors.

The gateway speaks two channels:

- **HTTP POST** of batched advertisements to
  `/signalk/v2/api/ble/gateway/advertisements`.
- **Control WebSocket** at `/signalk/v2/api/ble/gateway/ws` for the hello
  handshake, periodic status frames and GATT commands.

Both reuse the JWT that SensESP's `SKWSClient` already obtained through the
normal Signal K access request, so there is no second pairing step.

## Requirements

- SensESP 3.5.0 or later
- Signal K server 2.31.0 or later, which is where the BLE provider API landed
- PlatformIO with the [pioarduino](https://github.com/pioarduino/platform-espressif32)
  platform, built as `framework = espidf, arduino` — the library calls ESP-IDF
  Bluetooth APIs directly and needs the Bluetooth host turned on in `sdkconfig`

## Quick start

Add the library to your `platformio.ini`:

```ini
lib_deps =
    SignalK/SensESP @ ^3.5.0
    https://github.com/SensESP/BLE-gateway.git
```

The control WebSocket is built on `esp_websocket_client`, which is an ESP-IDF
component rather than a PlatformIO library. Declare it in your project's
`src/idf_component.yml` so its headers land on the include path of the whole
build, SensESP included:

```yaml
dependencies:
  idf: '>=5.3'
  espressif/esp_websocket_client: '^1.3'
```

On an ESP32, ESP32-S3 or ESP32-C3 the BLE radio is on the chip itself, so
`NativeBLE` is all you need:

```cpp
#include "sensesp_app_builder.h"
#include "sensesp_ble_gateway/ble_signalk_gateway.h"
#include "sensesp_ble_gateway/native_bluedroid_ble.h"

using namespace sensesp;

SensESPAppBuilder builder;
auto app = builder.set_hostname("signalk-ble-gw")->get_app();

auto ble = std::make_shared<NativeBLE>();
auto gateway = std::make_shared<BLESignalKGateway>(ble, app->get_ws_client());
gateway->start();
```

See [`examples/native_ble_gateway/`](examples/native_ble_gateway/) for the
complete firmware, including the ESP-IDF settings the Bluetooth host needs.

## BLE stacks

Pick the provisioner that matches the chip. All three implement the same
`BLEProvisioner` interface, so the gateway code above does not change.

| Provisioner | Header | Bluetooth host | Use it on |
|---|---|---|---|
| `NativeBLE` | `native_bluedroid_ble.h` | Bluedroid | ESP32, ESP32-S3, ESP32-C3 and other chips with an on-chip BLE controller |
| `EspHostedBluedroidBLE` | `esp_hosted_bluedroid_ble.h` | Bluedroid over esp_hosted | ESP32-P4, whose BLE radio lives on a companion ESP32-C6 reached over SDIO |
| `NimBLEProvisioner` | `nimble_ble.h` | NimBLE | Chips where Bluedroid and WiFi together do not fit in internal SRAM, such as the ESP32-C5 |

GATT client operations — connect, discover, subscribe, read, write — are
implemented on the Bluedroid provisioners. `NimBLEProvisioner` scans and
forwards advertisements only.

`EspHostedBluedroidBLE` additionally runs a four-level scan watchdog that
escalates from restarting the scan, through an RPC controller reset and a GPIO
hard reset of the companion chip, to a reboot.

## Hardware tested

| Board | Provisioner | Network | Status |
|---|---|---|---|
| Waveshare ESP32-P4-WIFI6-POE-ETH | `EspHostedBluedroidBLE` | Ethernet | Advertisements, control WebSocket and GATT |
| Waveshare ESP32-C5-WIFI6-KIT | `NimBLEProvisioner` | WiFi | Advertisements only; control WebSocket off to save RAM |

The ESP32, ESP32-S3 and ESP32-C3 environments are built in CI but have not been
run against a Signal K server on real hardware. Reports welcome.

## Examples

| Example | Targets |
|---|---|
| [`native_ble_gateway`](examples/native_ble_gateway/) | ESP32, ESP32-S3, ESP32-C3 — Bluedroid, WiFi |
| [`p4_ble_gateway`](examples/p4_ble_gateway/) | ESP32-P4 — Bluedroid via the onboard C6, RMII Ethernet |
| [`c5_ble_gateway`](examples/c5_ble_gateway/) | ESP32-C5 — NimBLE, WiFi |

Each example is a standalone PlatformIO project that picks the library up
through `symlink://../../`. Build one with `pio run -d examples/<name>`.

### A note on memory

Bluedroid, WiFi and TLS all draw on internal SRAM, and on the smaller chips they
add up. The ESP32-C3 example therefore trims `max_pending_ads` and turns the
control WebSocket off; the C5 example does the same and drops to a ~9% scan duty
cycle. If a build runs out of heap, those are the three knobs to reach for.

Flash is tight as well. Built `-Os` into a 1.9 MB OTA slot, the native example
uses 89% of it on the ESP32 and the ESP32-S3 and 97% on the ESP32-C3, whose
RISC-V code and smaller ROM library both cost space. The example drops the
RainMaker, Insights, Zigbee and DSP components that Arduino-ESP32 pulls in by
default; if you add much to the C3 build, expect to give it a partition table
with larger app slots.

### A note on the ESP32-P4 C6 antenna

The Waveshare ESP32-P4-WIFI6-POE-ETH carries the ESP32-C6-MINI-**1U** module,
which has no PCB antenna. Connect an external 2.4 GHz antenna to its IPEX
connector or the board will hear nothing.

## Origin

This library was written by [Dirk Wahrheit](https://github.com/dirkwa) and
published at `dirkwa/sensesp-ble-gateway` under the Apache License 2.0. That
repository was later relicensed to source-available terms and archived. The
SensESP project adopted the last Apache-2.0 revision, commit `ae35180`, and
continues it here. Nothing published under the later license is included.

## License

Apache License 2.0. See [LICENSE](LICENSE).
