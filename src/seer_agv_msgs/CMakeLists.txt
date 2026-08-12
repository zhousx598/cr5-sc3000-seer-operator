cmake_minimum_required(VERSION 3.8)
project(seer_agv_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConfirmLocalization.srv"
  "srv/LoadMap.srv"
  "srv/NavigateToStation.srv"
  "srv/Relocalize.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
