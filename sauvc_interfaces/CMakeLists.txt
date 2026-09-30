cmake_minimum_required(VERSION 3.8)
project(sauvc_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GateDetection.msg"
  "msg/ControlSetpoint.msg"
  "msg/FloorClearance.msg"
  "action/MoveToRelativePose.action"
  "action/ExecuteYawRotation.action"
  "srv/GetGatePose.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
