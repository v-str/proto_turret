#!/bin/bash
set -e

AGENT_WS="$HOME/Ros2/micro_ros_agent"
DEVICE="/dev/ttyACM0"

echo "=== Проверка устройства $DEVICE ==="
if [ ! -e "$DEVICE" ]; then
    echo "  Ошибка: $DEVICE не найден"
    exit 1
fi
echo "  OK"

echo ""
echo "=== Проверка доступа к $DEVICE ==="
if [ ! -r "$DEVICE" ] || [ ! -w "$DEVICE" ]; then
    echo "  Нет прав на чтение/запись. Добавь пользователя в группу plugdev:"
    echo "    sudo usermod -aG plugdev $USER"
    echo "  и перезайди в систему"
    exit 1
fi
echo "  OK"

echo ""
echo "=== Проверка собранного агента ==="
if [ ! -f "$AGENT_WS/install/setup.bash" ]; then
    echo "  Агент не собран. Собери сначала:"
    echo "    cd $AGENT_WS"
    echo "    colcon build"
    exit 1
fi
echo "  OK"

echo ""
echo "=== Запуск micro-ROS agent ==="
echo "  Устройство: $DEVICE"
echo "  Выход: Ctrl+C"
echo ""

source /opt/ros/lyrical/setup.bash
source "$AGENT_WS/install/setup.bash"
exec micro_ros_agent serial --dev "$DEVICE" -v6
