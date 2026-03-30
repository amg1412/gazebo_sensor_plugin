#!/usr/bin/env python3
"""
Launch file for Gazebo Sensor Plugin Demo.

This launch file starts:
1. Gazebo Harmonic simulator with the test world and IMU plugin
2. ros_gz_bridge to bridge IMU data from Gazebo to ROS 2
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    
    # Get package directory
    pkg_dir = get_package_share_directory("gazebo_sensor_plugin")
    world_file = os.path.join(pkg_dir, "config", "robot_world.sdf")
    bridge_config = os.path.join(pkg_dir, "config", "bridge.yaml")
    
    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_file,
        description="Path to Gazebo world file",
    )
    
    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Gazebo in headless mode (no GUI)",
    )
    
    # Get launch configurations
    world_config = LaunchConfiguration("world")
    
    # Build Gazebo command with proper arguments
    gazebo_cmd = [
        "gz",
        "sim",
        "-r",  # Real-time simulation
        world_config,  # World file path
    ]
    
    # Gazebo Harmonic process
    gazebo_process = ExecuteProcess(
        cmd=gazebo_cmd,
        output="screen",
        shell=False,
        on_exit=[RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=None,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )],
    )
    
    # ros_gz_bridge node for IMU data
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p", f"config_file:={bridge_config}",
        ],
        output="screen",
    )
    
    # Launch description
    ld = LaunchDescription([
        declare_world_arg,
        declare_headless_arg,
        gazebo_process,
        bridge_node,
    ])
    
    return ld


if __name__ == "__main__":
    generate_launch_description()
