#!/bin/bash
# Merge bootloader + partitions + boot_app0 + app into one image flashable at 0x0, plus copy the app binary.
#   firmware/tools/merge_bin.sh <pio env> <out dir>
set -euo pipefail
ENV=$1; OUT=$(mkdir -p "$2" && cd "$2" && pwd)
FW=$(cd "$(dirname "$0")/.." && pwd); B="$FW/.pio/build/$ENV"
case "$ENV" in e1_nano*) CHIP=esp32s3; BOOT=0x0 ;; *) CHIP=esp32; BOOT=0x1000 ;; esac
APP0=$(find ~/.platformio/packages -name boot_app0.bin | head -1)
esptool --chip "$CHIP" merge-bin -o "$OUT/$ENV-merged.bin" \
  "$BOOT" "$B/bootloader.bin" 0x8000 "$B/partitions.bin" 0xe000 "$APP0" 0x10000 "$B/firmware.bin"
cp "$B/firmware.bin" "$OUT/$ENV-app.bin"
echo "$OUT/$ENV-merged.bin (flash at 0x0), $OUT/$ENV-app.bin (0x10000)"
