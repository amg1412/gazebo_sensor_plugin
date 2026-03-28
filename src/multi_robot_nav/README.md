# multi_robot_nav

A ROS 2 (Humble) package that orchestrates navigation for **3 TurtleBot3 Burger robots**
running simultaneously in Gazebo Classic.

## What this project demonstrates

| Component | File | Key concept |
|-----------|------|-------------|
| C++ Lifecycle Node | `src/robot_lifecycle_node.cpp` | Lifecycle state machine, Nav2 action client, namespaced topics |
| Python Task Allocator | `scripts/task_allocator.py` | Round-robin waypoint assignment, async action clients |
| Python Bringup Helper | `scripts/robot_bringup.py` | Driving lifecycle transitions via service calls |
| Launch file | `launch/multi_robot.launch.py` | Namespacing, multi-robot spawn, Nav2 per robot |
| Gazebo World | `worlds/multi_robot_world.world` | Custom enclosed world with obstacles |

---

## Prerequisites

```bash
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-simulations \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-gazebo-ros-pkgs

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## Build

```bash
cd ~/multi_robot_ws
colcon build --packages-select multi_robot_nav --symlink-install
source install/setup.bash
```

---

## Run

### Terminal 1 — Gazebo + all robots
```bash
source ~/multi_robot_ws/install/setup.bash
ros2 launch multi_robot_nav multi_robot.launch.py
```

Wait until you see 3 robots spawned in Gazebo and Nav2 active for each.

### Terminal 2 — Drive lifecycle transitions
```bash
source ~/multi_robot_ws/install/setup.bash
ros2 run multi_robot_nav robot_bringup.py
```

Expected output:
```
[robot_bringup] Phase 1: Configuring all robots
[robot1] Transition CONFIGURE SUCCESS
[robot2] Transition CONFIGURE SUCCESS
[robot3] Transition CONFIGURE SUCCESS
[robot_bringup] Phase 2: Activating all robots
[robot1] Transition ACTIVATE SUCCESS
...
All robots ACTIVE — run task_allocator.py to start navigation.
```

### Terminal 3 — Start task allocation
```bash
source ~/multi_robot_ws/install/setup.bash
ros2 run multi_robot_nav task_allocator.py
```

You will see waypoints assigned round-robin across the 3 robots.

---

## Observing the namespace fix

### The bug (without namespacing)
If you remove `PushRosNamespace` from the launch file and spawn robots without `-robot_namespace`,
all three robots publish to the same `/cmd_vel`, `/odom`, and `/scan` topics.
Nav2 picks up the wrong robot's odometry and the simulation breaks immediately.

### The fix
Each robot is launched under `/robot1`, `/robot2`, `/robot3`.
Topics become `/robot1/cmd_vel`, `/robot2/odom`, etc.
Nav2 and the lifecycle node both resolve topics relative to their namespace.

Verify isolation:
```bash
# Should show 3 separate cmd_vel topics
ros2 topic list | grep cmd_vel
# /robot1/cmd_vel
# /robot2/cmd_vel
# /robot3/cmd_vel
```

---

## Package structure

```
multi_robot_nav/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── nav2_params.yaml          # Nav2 params for TurtleBot3
├── launch/
│   └── multi_robot.launch.py     # Main launch file
├── multi_robot_nav/
│   └── __init__.py
├── scripts/
│   ├── task_allocator.py         # Python waypoint allocator
│   └── robot_bringup.py          # Python lifecycle driver
├── src/
│   └── robot_lifecycle_node.cpp  # C++ lifecycle node
└── worlds/
    └── multi_robot_world.world   # Custom Gazebo world
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `Nav2 action server not available` | Nav2 hasn't finished starting | Increase the `TimerAction` delay in the launch file from 5 s to 10 s |
| Robots overlap on spawn | Spawn positions clash | Edit `ROBOTS` list in launch file |
| `colcon build` fails on `nav2_msgs` | Package not installed | `sudo apt install ros-humble-nav2-msgs` |
| Task allocator exits immediately | Lifecycle nodes not active yet | Run `robot_bringup.py` first |
