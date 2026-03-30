cmake_minimum_required(VERSION 3.8)
project(gazebo_sensor_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS 2 packages (required)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Gazebo Harmonic packages (optional for package metadata validation)
# These will be required at runtime but are optional for package build validation
find_package(gz-sim QUIET)
find_package(gz-common QUIET)
find_package(gz-transport QUIET)
find_package(gz-utils QUIET)
find_package(gz-msgs QUIET)

if(NOT gz-sim_FOUND OR NOT gz-transport_FOUND)
  message(WARNING "Gazebo Harmonic packages not found. Plugin will not be built.")
  message(WARNING "To build the plugin, please install: sudo apt install libgz-sim-dev libgz-transport-dev")
endif()

# ── Build Plugin Library ─────────────────────────────────────────────────────
# Only build plugin if Gazebo packages are available
if(gz-sim_FOUND AND gz-transport_FOUND)
  # Create shared library for the Gazebo plugin
  add_library(noise_imu_sensor_plugin SHARED
    src/noise_imu_sensor.cpp
  )

  ament_target_dependencies(noise_imu_sensor_plugin
    rclcpp
    sensor_msgs
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    geometry_msgs
  )

  # Link Gazebo libraries
  target_link_libraries(noise_imu_sensor_plugin
    gz-sim::gz-sim
    gz-common::gz-common
    gz-transport::gz-transport
    gz-utils::gz-utils
    gz-msgs::gz-msgs
  )

  # Include directories for this library
  target_include_directories(noise_imu_sensor_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # ── Install Plugin Library ───────────────────────────────────────────────────
  install(TARGETS noise_imu_sensor_plugin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # Install headers
  install(DIRECTORY include/
    DESTINATION include
  )

  message(STATUS "Gazebo sensor plugin will be built and installed.")
else()
  message(STATUS "Skipping plugin library build (Gazebo packages not found).")
endif()

# ── Install Configuration and Launch files ───────────────────────────────────
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ── Python Test Setup ────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# Install Python test files
install(PROGRAMS
  test/test_imu_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Linting ──────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ── Finalize Package ─────────────────────────────────────────────────────────
ament_package()
