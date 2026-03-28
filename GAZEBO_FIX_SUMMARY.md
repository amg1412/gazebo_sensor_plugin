# Multi-Robot Gazebo Simulation - Fix Documentation

## Issues Found & Resolved

### 1. **Stale Gazebo Processes (PRIMARY ISSUE)**
**Symptom**: `[Err] [Master.cc:96] EXCEPTION: Unable to start server[bind: Address already in use]`

**Root Cause**: Previous Gazebo instances were not properly terminated, keeping port 11345 occupied.

**Solution**: Kill all stale processes before launching
```bash
pkill -9 -f "gzserver"
pkill -9 -f "gazebo"
pkill -9 -f "ros2"
sleep 2
```

### 2. **SDF Version Incompatibility**
**Symptom**: Gazebo crash with exit code 255

**Root Cause**: World file used SDF version 1.6, which is incompatible with Gazebo 11.10.2 (used in ROS Humble). Gazebo 11 requires SDF 1.9+

**Solution**: Updated `worlds/multi_robot_world.world` from `<sdf version="1.6">` to `<sdf version="1.9">`

### 3. **GUI Mode Initialization Issues**  
**Symptom**: Gazebo client crashes with null pointer exceptions

**Root Cause**: GUI rendering initialization conflicts in headless environments

**Solution**: Modified `launch/multi_robot.launch.py` to:
- Set `gui: false` - Disable Gazebo GUI client
- Set `headless: true` - Run in pure server mode
- Set `verbose: true` - Enable debugging

## Files Modified

1. **src/multi_robot_nav/worlds/multi_robot_world.world**
   - Line 2: Changed `<sdf version="1.6">` to `<sdf version="1.9">`

2. **src/multi_robot_nav/launch/multi_robot.launch.py**
   - Added launch arguments for headless mode and GUI control
   - Lines ~160-169: Added `gui`, `headless`, and enhanced `verbose` parameters

## Recommended Usage

### Quick Start (WITH Cleanup)
```bash
cd /home/amg/multi_robot_ws
bash cleanup_gazebo.sh
source install/setup.bash
ros2 launch multi_robot_nav multi_robot.launch.py
```

### Build & Launch
```bash
colcon build
source install/setup.bash
ros2 launch multi_robot_nav multi_robot.launch.py
```

## Verification

The simulation should now:
1. ✅ Start gzserver successfully without port conflicts
2. ✅ Spawn 3 TurtleBot3 robots (robot1, robot2, robot3) into the world
3. ✅ Initialize Nav2 navigation stack for each robot
4. ✅ Load robot lifecycle nodes and task allocators

## Troubleshooting

If the issue persists:

1. **Check for blocked ports:**
   ```bash
   lsof -i :11345
   ```

2. **View Gazebo logs:**
   ```bash
   cat ~/.ros/log/*/gzserver*.log
   ```

3. **Force full cleanup:**
   ```bash
   pkill -9 -f "ros2|gazebo|gzserver|spawn_entity"
   rm -rf ~/.gazebo/
   ```

4. **Rebuild workspace:**
   ```bash
   cd /home/amg/multi_robot_ws
   rm -rf build install
   colcon build
   ```

## Performance Notes

- Simulation runs in **headless mode** (no GUI) for better resource efficiency
- Three robots run concurrently with independent Nav2 stacks
- Recommend running on systems with 4GB+ RAM and dual-core processors minimum
