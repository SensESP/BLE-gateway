#!/bin/bash
# Build and run the host tests. No framework and no PlatformIO: these cover
# code that has no Arduino or IDF dependency, so a compiler is the only
# requirement.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$here/../.."
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT

status=0
for test_src in "$here"/test_*.cpp; do
  name="$(basename "$test_src" .cpp)"
  # Compile the unit under test alongside its test; the list stays explicit so
  # a test cannot quietly start depending on the whole library.
  case "$name" in
    test_json_escape) units=("$root/src/sensesp_ble_gateway/json_escape.cpp") ;;
    *) echo "no unit list for $name"; exit 1 ;;
  esac
  echo "== $name"
  g++ -std=c++17 -Wall -Wextra -Werror -I "$root/src" \
      -o "$out/$name" "$test_src" "${units[@]}"
  "$out/$name" || status=1
done
exit $status
