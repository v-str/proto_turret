#!/bin/bash
set -e

AGENT_WS="$HOME/Ros2/micro_ros_agent"
AGENT_BIN="$AGENT_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
DEVICE="/dev/ttyACM0"

[ -e "$DEVICE" ] || { echo "Ошибка: $DEVICE не найден"; exit 1; }
[ -r "$DEVICE" ] && [ -w "$DEVICE" ] || { echo "Нет прав на $DEVICE. Добавь себя в plugdev."; exit 1; }
[ -f "$AGENT_BIN" ] || { echo "Агент не собран. Собери: cd $AGENT_WS && colcon build"; exit 1; }

. /opt/ros/lyrical/setup.bash
. "$AGENT_WS/install/setup.bash"

exec "$AGENT_BIN" serial --dev "$DEVICE" -v5
