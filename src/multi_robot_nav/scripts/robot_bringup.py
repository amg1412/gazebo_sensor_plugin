#!/usr/bin/env python3
"""
robot_bringup.py
────────────────
Drives the ROS 2 lifecycle transitions for all three robot lifecycle nodes.

  Unconfigured  ──configure──►  Inactive  ──activate──►  Active
                                                          (ready for goals)

Call this script AFTER Gazebo is running and all three robot_lifecycle_nodes
have been launched.  It calls the /robotN/robot_lifecycle_node/change_state
service for each robot in sequence.

Usage:
  ros2 run multi_robot_nav robot_bringup.py
"""

import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


ROBOT_NAMESPACES = ["robot1", "robot2", "robot3"]
NODE_NAME        = "robot_lifecycle_node"


class RobotBringup(Node):
    def __init__(self):
        super().__init__("robot_bringup")

    def transition(self, ns: str, transition_id: int) -> bool:
        srv_name = f"/{ns}/{NODE_NAME}/change_state"
        client   = self.create_client(ChangeState, srv_name)

        self.get_logger().info(f"Waiting for {srv_name} …")
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"Service {srv_name} not available!")
            return False

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() and future.result().success:
            self.get_logger().info(
                f"[{ns}] Transition {transition_id} SUCCESS")
            return True
        else:
            self.get_logger().error(
                f"[{ns}] Transition {transition_id} FAILED")
            return False


def main():
    rclpy.init()
    node = RobotBringup()

    # ── Step 1: Configure all robots ──────────────────────────────────────
    node.get_logger().info("─── Phase 1: Configuring all robots ───")
    for ns in ROBOT_NAMESPACES:
        ok = node.transition(ns, Transition.TRANSITION_CONFIGURE)
        if not ok:
            node.get_logger().error(f"Failed to configure {ns}. Aborting.")
            rclpy.shutdown()
            return
        time.sleep(0.5)

    # ── Step 2: Activate all robots ───────────────────────────────────────
    node.get_logger().info("─── Phase 2: Activating all robots ───")
    for ns in ROBOT_NAMESPACES:
        ok = node.transition(ns, Transition.TRANSITION_ACTIVATE)
        if not ok:
            node.get_logger().error(f"Failed to activate {ns}. Aborting.")
            rclpy.shutdown()
            return
        time.sleep(0.5)

    node.get_logger().info(
        "All robots ACTIVE — run task_allocator.py to start navigation.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
