cmake_minimum_required(VERSION 3.5)
project(proto_turret_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(interfaces
  "msg/TurretCommand.msg"
  "msg/TurretStatus.msg"
  "srv/TurretCalibrate.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interfaces}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
