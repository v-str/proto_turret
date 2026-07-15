#!/bin/bash
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
TARGET="proto_turret"
HEX="$BUILD_DIR/$TARGET.hex"
OPENOCD_CFG="board/st_nucleo_f4.cfg"

cd "$PROJECT_DIR"

echo "=== Проверка инструментов ==="
if ! command -v openocd &>/dev/null; then
    echo "  Ошибка: openocd не установлен"
    exit 1
fi
echo "  openocd найден"

echo ""
echo "=== Проверка .hex ==="
if [ ! -f "$HEX" ]; then
    echo "  Ошибка: $HEX не найден"
    echo "  Сначала выполни make"
    exit 1
fi
echo "  $HEX найден"

echo ""
echo "=== Проверка подключения платы ==="
if ! openocd -f "$OPENOCD_CFG" -c "init; exit" 2>&1 | grep -q "Cortex-M4"; then
    echo "  Ошибка: плата STM32 не обнаружена"
    echo "  Проверь USB-подключение через ST-Link"
    exit 1
fi
echo "  Плата обнаружена"

echo ""
echo "=== Прошивка ==="
openocd -f "$OPENOCD_CFG" \
       -c "program $HEX verify reset exit"
echo "  Прошивка завершена"

echo ""
echo "=== Готово ==="
