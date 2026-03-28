#!/usr/bin/env python3
"""
task_allocator.py
─────────────────
Assigns navigation waypoints to 3 TurtleBot3 robots and monitors
their completion via Nav2 action clients.

Strategy: round-robin assignment.
  – Maintains a queue of waypoints.
  – Polls each robot's /robotN/nav_status topic.
  – As soon as a robot finishes (SUCCEEDED / FAILED / CANCELLED),
    it is given the next waypoint in the queue.

Run after all three robot_lifecycle_nodes are active.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ── Waypoint list (x, y, yaw_deg) ─────────────────────────────────────────
# Adjust these to fit your Gazebo world.  The default TurtleBot3 world
# (turtlebot3_world.world) is roughly 4 m × 4 m centred at origin.
WAYPOINTS = [
    ( 1.5,  0.0,   0.0),
    ( 1.5,  1.5,  90.0),
    ( 0.0,  1.5, 180.0),
    (-1.5,  1.5, 180.0),
    (-1.5,  0.0, 270.0),
    (-1.5, -1.5, 270.0),
    ( 0.0, -1.5,   0.0),
    ( 1.5, -1.5,   0.0),
]

ROBOT_NAMESPACES = ["robot1", "robot2", "robot3"]


# ── Helper ─────────────────────────────────────────────────────────────────
def make_pose_stamped(node: Node, x: float, y: float, yaw_deg: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp    = node.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    yaw = math.radians(yaw_deg)
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


# ── Per-robot state ────────────────────────────────────────────────────────
class RobotAgent:
    """
    Holds the action client and current goal state for one robot.
    Subscribes to /robotN/nav_status (published by the C++ lifecycle node)
    to track high-level state, and uses the Nav2 action client directly
    to send new goals.
    """

    def __init__(self, node: Node, namespace: str, cb_group):
        self.ns     = namespace
        self.node   = node
        self.status = "IDLE"          # mirrors NavState enum in C++ node
        self.busy   = False
        self.goal_handle = None

        # Action client to Nav2 running under this robot's namespace
        self._ac = ActionClient(
            node,
            NavigateToPose,
            f"/{namespace}/navigate_to_pose",
            callback_group=cb_group,
        )

        # Subscribe to the C++ node's status topic
        node.create_subscription(
            String,
            f"/{namespace}/nav_status",
            self._status_cb,
            10,
            callback_group=cb_group,
        )

        node.get_logger().info(f"[TaskAllocator] Agent created for /{namespace}")

    # ── Callbacks ───────────────────────────────────────────────────────────
    def _status_cb(self, msg: String):
        prev = self.status
        self.status = msg.data
        if self.status in ("SUCCEEDED", "FAILED", "CANCELLED"):
            self.busy = False
        if prev != self.status:
            self.node.get_logger().info(
                f"[{self.ns}] status: {prev} → {self.status}")

    def _goal_response_cb(self, future):
        gh = future.result()
        if not gh:
            self.node.get_logger().error(f"[{self.ns}] Goal REJECTED")
            self.busy = False
            return
        self.goal_handle = gh
        self.node.get_logger().info(f"[{self.ns}] Goal ACCEPTED")
        # Request result
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"[{self.ns}] Waypoint REACHED ✓")
        else:
            self.node.get_logger().warn(
                f"[{self.ns}] Goal ended with status {result.status}")
        self.busy = False
        self.goal_handle = None

    # ── Public ──────────────────────────────────────────────────────────────
    def is_ready(self) -> bool:
        """True if the robot is idle and the action server is reachable."""
        return not self.busy and self._ac.server_is_ready()

    def send_goal(self, x: float, y: float, yaw_deg: float):
        if self.busy:
            return
        self.busy = True
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(self.node, x, y, yaw_deg)

        self.node.get_logger().info(
            f"[{self.ns}] Sending waypoint → ({x:.2f}, {y:.2f}, {yaw_deg:.0f}°)")

        send_future = self._ac.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def cancel(self):
        if self.goal_handle and self.busy:
            self.node.get_logger().warn(f"[{self.ns}] Cancelling current goal.")
            self.goal_handle.cancel_goal_async()


# ── Main allocator node ────────────────────────────────────────────────────
class TaskAllocator(Node):
    """
    Round-robin task allocator.
    Ticks every second; assigns the next waypoint to any idle robot.
    Stops when the waypoint queue is exhausted and all robots are idle.
    """

    def __init__(self):
        super().__init__("task_allocator")
        self._cb_group = ReentrantCallbackGroup()

        self._waypoints = list(WAYPOINTS)          # mutable queue
        self._wp_index  = 0
        self._assigned  = 0
        self._completed = 0

        self._agents = {
            ns: RobotAgent(self, ns, self._cb_group)
            for ns in ROBOT_NAMESPACES
        }

        # Wait for all Nav2 action servers before starting
        self.get_logger().info("[TaskAllocator] Waiting for Nav2 action servers …")
        self._wait_for_servers()

        self._timer = self.create_timer(
            1.0, self._tick, callback_group=self._cb_group)

        self.get_logger().info(
            f"[TaskAllocator] Started — {len(self._waypoints)} waypoints, "
            f"{len(self._agents)} robots.")

    # ── Server wait ──────────────────────────────────────────────────────────
    def _wait_for_servers(self):
        for agent in self._agents.values():
            while not agent._ac.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    f"[TaskAllocator] Still waiting for /{agent.ns}/navigate_to_pose …")
        self.get_logger().info("[TaskAllocator] All action servers ready ✓")

    # ── Tick ─────────────────────────────────────────────────────────────────
    def _tick(self):
        # Check if we're done
        all_idle = all(not a.busy for a in self._agents.values())
        if self._wp_index >= len(self._waypoints) and all_idle:
            self.get_logger().info(
                "[TaskAllocator] All waypoints assigned and completed. Shutting down.")
            self._timer.cancel()
            rclpy.shutdown()
            return

        # Round-robin: try each robot in order
        for ns, agent in self._agents.items():
            if self._wp_index >= len(self._waypoints):
                break
            if agent.is_ready():
                wp = self._waypoints[self._wp_index]
                self._wp_index += 1
                agent.send_goal(*wp)
                self._assigned += 1
                self.get_logger().info(
                    f"[TaskAllocator] Waypoint {self._wp_index}/{len(self._waypoints)} "
                    f"assigned to /{ns}")


# ── Entry point ────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn("[TaskAllocator] Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
