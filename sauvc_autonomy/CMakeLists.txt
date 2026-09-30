cmake_minimum_required(VERSION 3.8)
project(sauvc_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sauvc_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Build the main C++ BT Executor
add_executable(sauvc_bt_executor src/sauvc_bt_executor.cpp)

# Include the headers directory
target_include_directories(sauvc_bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link all the ROS 2 and BT.CPP dependencies
ament_target_dependencies(sauvc_bt_executor
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  behaviortree_cpp
  action_msgs
  tf2_ros
  sauvc_interfaces
)

# Install the C++ executable
install(TARGETS sauvc_bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python Mock Nodes
install(PROGRAMS
  scripts/mock_perception_node
  scripts/mock_navigation_node
  scripts/mock_controls_node
  scripts/mock_state_validator
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch and Config files
install(DIRECTORY
  launch
  bt_trees config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
