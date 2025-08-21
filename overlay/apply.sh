#!/usr/bin/env sh

set -e

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <path>"
  exit 1
fi

DTS="$1"
TREE="/sys/kernel/config/device-tree/overlays/sx1280"

dtc -@ -I dts -O dtb -o /tmp/overlay.dtbo overlay.dts
mkdir -p "$TREE"
cat /tmp/overlay.dtbo > "$TREE/dtbo"
