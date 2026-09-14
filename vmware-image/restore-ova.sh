#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
output="vehicle-ros-ubuntu22.04-5f949a5.ova"
cat "${output}.part-"* > "$output"
sha256sum -c "${output}.sha256"
printf 'Restored: %s/%s\n' "$PWD" "$output"
