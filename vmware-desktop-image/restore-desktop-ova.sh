#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
output="vehicle-ros-ubuntu22.04-desktop-5f949a5.ova"
cat "${output}.part-"* > "$output"
sha256sum -c "${output}.sha256"
printf 'Desktop OVA restored: %s/%s\n' "$PWD" "$output"
