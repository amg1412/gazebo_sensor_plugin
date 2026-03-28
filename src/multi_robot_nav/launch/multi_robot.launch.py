"""
multi_robot.launch.py  (fixed)
──────────────────────────────
Fixes applied vs original:
  1. robot_state_publisher now receives robot_description via xacro
  2. Nav2 uses the TurtleBot3 map that ships with turtlebot3_navigation2
  3. bt_navigator plugin list trimmed to plugins that exist in Humble
"""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


# ── Robot spawn positions ──────────────────────────────────────────────────
ROBOTS = [
    {"name": "robot1", "x":  0.0, "y":  0.0, "yaw": 0.0},
    {"name": "robot2", "x":  1.5, "y":  0.0, "yaw": 0.0},
    {"name": "robot3", "x": -1.5, "y":  0.0, "yaw": 0.0},
]


def get_robot_description() -> str:
    """Run xacro to get the TurtleBot3 Burger URDF as a string."""
    tb3_desc_pkg = get_package_share_directory("turtlebot3_description")
    xacro_file = os.path.join(tb3_desc_pkg, "urdf", "turtlebot3_burger.urdf")
    if not os.path.exists(xacro_file):
        xacro_file = os.path.join(tb3_desc_pkg, "urdf", "turtlebot3_burger.urdf.xacro")
    result = subprocess.run(
        ["xacro", xacro_file],
        capture_output=True, text=True, check=True
    )
    return result.stdout


def get_map_yaml() -> str:
    """Return path to the bundled TurtleBot3 map."""
    try:
        nav2_pkg = get_package_share_directory("turtlebot3_navigation2")
        candidate = os.path.join(nav2_pkg, "map", "map.yaml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    try:
        nav2_bringup = get_package_share_directory("nav2_bringup")
        candidate = os.path.join(nav2_bringup, "maps", "tb3_sandbox.yaml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    raise RuntimeError(
        "Could not find map.yaml. Run: "
        "sudo apt install ros-humble-turtlebot3-navigation2"
    )


def generate_robot_group(robot: dict, robot_description: str, map_yaml: str) -> GroupAction:
    ns  = robot["name"]
    x   = str(robot["x"])
    y   = str(robot["y"])
    yaw = str(robot["yaw"])

    nav2_pkg = FindPackageShare("nav2_bringup")
    this_pkg = FindPackageShare("multi_robot_nav")

    return GroupAction([
        PushRosNamespace(ns),

        # FIX 1: pass robot_description explicitly
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "use_sim_time":      True,
                "robot_description": robot_description,
                "frame_prefix":      ns + "/",
            }],
            remappings=[("/tf",        f"/{ns}/tf"),
                        ("/tf_static", f"/{ns}/tf_static")],
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity",          ns,
                "-topic",           f"/{ns}/robot_description",
                "-x",               x,
                "-y",               y,
                "-z",               "0.01",
                "-Y",               yaw,
                "-robot_namespace", ns,
            ],
            output="screen",
        ),

        # FIX 2: pass real map path
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        nav2_pkg, "/launch/bringup_launch.py"
                    ]),
                    launch_arguments={
                        "use_sim_time":  "true",
                        "namespace":     ns,
                        "use_namespace": "true",
                        "params_file":   PathJoinSubstitution([
                            this_pkg, "config", "nav2_params.yaml"
                        ]),
                        "map":           map_yaml,
                    }.items(),
                )
            ],
        ),

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="multi_robot_nav",
                    executable="robot_lifecycle_node",
                    name="robot_lifecycle_node",
                    namespace=ns,
                    parameters=[{
                        "robot_namespace": ns,
                        "use_sim_time":    True,
                    }],
                    output="screen",
                )
            ],
        ),
    ])


def generate_launch_description():
    robot_description = get_robot_description()
    map_yaml          = get_map_yaml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"
        ]),
        launch_arguments={
            "world":   PathJoinSubstitution([
                FindPackageShare("multi_robot_nav"), "worlds", "multi_robot_world.world"
            ]),
            "verbose": "false",
            "gui":     "true",
            "headless": "false",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        
        # Use software rendering for graphics compatibility
        SetEnvironmentVariable("LIBGL_ALWAYS_INDIRECT", "1"),
        SetEnvironmentVariable("GAZEBO_VERBOSE", "0"),
        SetEnvironmentVariable("IGN_GAZEBO_VERBOSE", "0"),
        
        gazebo,
        *[generate_robot_group(r, robot_description, map_yaml) for r in ROBOTS],
    ])
