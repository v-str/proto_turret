#!/bin/bash
set -e

IMAGE="microros/micro-ros-agent:kilted"
DEVICE="/dev/ttyACM0"
CONTAINER_NAME="micro_ros_agent"

echo "=== Проверка Docker ==="
if ! command -v docker &>/dev/null; then
    echo "  Ошибка: docker не установлен"
    exit 1
fi
echo "  OK"

echo ""
echo "=== Проверка устройства $DEVICE ==="
if [ ! -e "$DEVICE" ]; then
    echo "  Ошибка: $DEVICE не найден"
    ls /dev/ttyACM* 2>/dev/null || echo "  Нет устройств /dev/ttyACM*"
    exit 1
fi
echo "  OK"

echo ""
echo "=== Проверка прав на $DEVICE ==="
if [ ! -r "$DEVICE" ] || [ ! -w "$DEVICE" ]; then
    echo "  Нет прав на чтение/запись. Добавь пользователя в группу dialout:"
    echo "    sudo usermod -aG dialout $USER"
    echo "  Или запусти скрипт через sudo"
    exit 1
fi
echo "  OK"

echo ""
echo "=== Проверка образа $IMAGE ==="
if ! docker image inspect "$IMAGE" &>/dev/null; then
    echo "  Образ не найден, качаю..."
    docker pull "$IMAGE"
fi
echo "  OK"

echo ""
echo "=== Проверка запущенного контейнера ==="
EXISTING=$(docker ps -q -f name="$CONTAINER_NAME" 2>/dev/null)
if [ -n "$EXISTING" ]; then
    echo "  Контейнер $CONTAINER_NAME уже запущен, останавливаю..."
    docker rm -f "$CONTAINER_NAME"
fi
echo "  OK"

echo ""
echo "=== Запуск micro-ROS agent ==="
echo "  Устройство: $DEVICE"
echo "  Образ: $IMAGE"
echo "  Выход: Ctrl+C"
echo ""
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --net=host \
    --privileged \
    -v /dev:/dev \
    -e ROS_LOCALHOST_ONLY=1 \
    "$IMAGE" \
    serial --dev "$DEVICE" -v6
