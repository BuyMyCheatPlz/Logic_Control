#!/usr/bin/env bash
set -euo pipefail

# 一键构建并烧录脚本
# 用法: ./scripts/build_and_flash.sh [Debug|Release]

BUILD_TYPE="${1:-Debug}"
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build/$BUILD_TYPE"
PROJECT_NAME="Logic_Control"
ELF="$BUILD_DIR/${PROJECT_NAME}.elf"
BIN="$BUILD_DIR/${PROJECT_NAME}.bin"
HEX="$BUILD_DIR/${PROJECT_NAME}.hex"

echo "Project: $PROJECT_NAME"
echo "Build type: $BUILD_TYPE"
echo "Build dir: $BUILD_DIR"

echo "\n==> Configuring (cmake preset: $BUILD_TYPE)"
cmake --preset "$BUILD_TYPE"

echo "\n==> Building"
cmake --build "$BUILD_DIR" -- -j"$(nproc)"

if [ ! -f "$ELF" ]; then
  echo "ERROR: build output not found: $ELF" >&2
  exit 2
fi

echo "\n==> Build succeeded. Files:"
ls -l "$ELF" "$BIN" "$HEX" 2>/dev/null || true

echo "\n==> Flashing target"

if command -v st-flash >/dev/null 2>&1; then
  echo "Found st-flash -> writing binary to device"
  if [ ! -f "$BIN" ]; then
    echo "Binary $BIN not found, attempting to generate using objcopy"
    if command -v ${TOOLCHAIN_PREFIX:-arm-none-eabi-}objcopy >/dev/null 2>&1; then
      ${TOOLCHAIN_PREFIX:-arm-none-eabi-}objcopy -O binary "$ELF" "$BIN"
    else
      echo "objcopy not found. Cannot produce binary." >&2
      exit 3
    fi
  fi
  st-flash write "$BIN" 0x08000000
  echo "st-flash finished."
  exit 0
fi

if command -v openocd >/dev/null 2>&1; then
  echo "Found openocd -> programming ELF via openocd"
  openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program \"$ELF\" verify reset exit"
  echo "openocd finished."
  exit 0
fi

echo "No flashing tool (st-flash or openocd) found. Build artifacts are in: $BUILD_DIR"
echo "If you have an ST-Link, install 'stlink' (provides st-flash) or OpenOCD."

exit 0
