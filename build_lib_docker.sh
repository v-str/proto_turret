#!/bin/bash

set -e

PROJECT_DIR=$(pwd)
IMAGE_NAME="microros/micro_ros_static_library_builder:humble-cached"

echo "=== Полная пересборка библиотеки micro-ROS (с очисткой кеша) ==="

# 1. Удаляем старую библиотеку и кеш colcon.
#    Файлы принадлежат текущему пользователю, sudo не нужен (и без TTY он не
#    может запросить пароль).
rm -rfv micro_ros_stm32cubemx_utils/microros_static_library/libmicroros
rm -rfv micro_ros_stm32cubemx_utils/microros_static_library/library_generation/install
rm -rfv micro_ros_stm32cubemx_utils/microros_static_library/library_generation/build
rm -rfv micro_ros_stm32cubemx_utils/microros_static_library/library_generation/log

# 2. Запускаем контейнер и собираем библиотеку из правильной папки
yes | docker container prune
docker run -i \
  --name micro_ros_builder \
  -v $PROJECT_DIR:/project \
  -w /project \
  --entrypoint /bin/bash \
  $IMAGE_NAME \
  -c '
    echo "=== Запуск сборки библиотеки ==="
    source /opt/ros/humble/setup.bash
    export TOOLCHAIN_PREFIX=arm-none-eabi-
    export MICROROS_LIBRARY_FOLDER="micro_ros_stm32cubemx_utils/microros_static_library"
    pushd micro_ros_stm32cubemx_utils/microros_static_library/library_generation
    ./library_generation.sh
    popd
  '

# 3. Проверка результата
echo "=== Проверка собранной библиотеки ==="
ls -la micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
