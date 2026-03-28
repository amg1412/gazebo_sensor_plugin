#!/bin/bash
# Cleanup script for multi-robot Gazebo simulation
# Ensures clean startup by removing stale processes

echo "Cleaning up stale Gazebo and ROS processes..."
pkill -9 -f "gzserver" 2>/dev/null
pkill -9 -f "gazebo" 2>/dev/null
pkill -9 -f "ros2" 2>/dev/null
pkill -9 -f "spawn_entity" 2>/dev/null
sleep 2
echo "Cleanup complete. Ready to launch simulation."
