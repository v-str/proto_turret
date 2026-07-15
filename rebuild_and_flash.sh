#!/bin/bash
set -e

BOARD_SERIAL="usb-STMicroelectronics_STM32_STLink_066CFF545052844887234815-if02"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
TARGET="proto_turret"

echo "=== Checking STM32 board connection ==="
if [ -L "/dev/serial/by-id/$BOARD_SERIAL" ]; then
    DEVICE=$(readlink -f "/dev/serial/by-id/$BOARD_SERIAL")
    echo "  Board found: $DEVICE"
elif [ -e /dev/ttyACM0 ]; then
    echo "  Warning: board detected on /dev/ttyACM0 but serial ID mismatch (different board?)"
    DEVICE=/dev/ttyACM0
else
    echo "  ERROR: No STM32 board found (expected serial: $BOARD_SERIAL)"
    echo "  Checked: /dev/serial/by-id/$BOARD_SERIAL"
    ls /dev/serial/by-id/* 2>/dev/null || echo "  No serial devices found"
    exit 1
fi

echo ""
echo "=== Rebuilding firmware ==="
cd "$PROJECT_DIR"
make -j4
echo "  Build OK"

echo ""
echo "=== Flashing via OpenOCD ==="
openocd -f board/st_nucleo_f4.cfg \
       -c "program $BUILD_DIR/$TARGET.elf verify reset exit"
echo "  Flash OK"

echo ""
echo "=== Done ==="
