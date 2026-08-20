#!/bin/bash
set -e

IMAGE_NAME="microros/micro_ros_static_library_builder:humble-cached"

docker container prune

docker run -i --rm \
  -v $(pwd):/project \
  -w /project \
  --entrypoint /bin/bash \
  $IMAGE_NAME
