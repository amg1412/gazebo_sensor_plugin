cmake_minimum_required(VERSION 3.8)
project(multi_robot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── C++ Lifecycle Node ───────────────────────────────────────────────────────
add_executable(robot_lifecycle_node
  src/robot_lifecycle_node.cpp
)
ament_target_dependencies(robot_lifecycle_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_msgs
  action_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(robot_lifecycle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS robot_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python scripts ───────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/task_allocator.py
  scripts/robot_bringup.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch files & config ────────────────────────────────────────────────────
install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
