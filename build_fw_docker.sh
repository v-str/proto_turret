#!/bin/bash

PROJECT_DIR=$(pwd)
IMAGE_NAME="microros/micro_ros_static_library_builder:humble-cached"

docker container prune -f

docker run -i --rm \
  -v $PROJECT_DIR:/project \
  -w /project \
  --entrypoint /bin/bash \
  $IMAGE_NAME \
  -c '
    make clean
    make -j
  '

# Проверяем результат после сборки
echo "=== Проверка файлов прошивки: ==="
ls -la build/proto_turret.elf build/proto_turret.bin build/proto_turret.hex