#!/usr/bin/env python3

import os
import signal
import subprocess

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class ROSInterface(Node):

    def __init__(self):

        super().__init__('robot_gui_interface')

        # ================= MODE CONTROL (FOR MUX) =================
        self.mode_pub = self.create_publisher(String, '/cmd_vel_mode', 10)

        # ================= STATE FEEDBACK =================
        self.state_callback_fn = None

        self.create_subscription(
            String,
            '/robot_state',
            self._state_cb,
            10
        )

        # ================= NAV2 CLIENT =================
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.current_goal_handle = None

        # ================= WOA PROCESS =================
        self.woa_process = None

        # ================= LOCATIONS (YOUR NEW WORLD) =================
        self.locations = {
            "startpose": {
                "position": (-0.0015, -0.0037, 0.0),
                "orientation": (0.0, 0.0, -0.007, 0.999)
            },
            "checkout": {
                "position": (1.97, 3.24, 0.0),
                "orientation": (0.0, 0.0, -0.70, 0.71)
            },
            "entrance": {
                "position": (-3.43, 6.83, 0.0),
                "orientation": (0.0, 0.0, -0.71, 0.70)
            },
            "charging_station": {
                "position": (-4.03, -6.34, 0.0),
                "orientation": (0.0, 0.0, 0.69, 0.71)
            },
            "storage": {
                "position": (-4.62, -1.99, 0.0),
                "orientation": (0.0, 0.0, 0.00, 0.999)
            },
            "aisle1": {
                "position": (3.02, -4.00, 0.0),
                "orientation": (0.0, 0.0, -0.007, 0.999)
            },
            "aisle2": {
                "position": (4.24, -0.09, 0.0),
                "orientation": (0.0, 0.0, 0.999, 0.001)
            }
        }

    # =========================================================
    # STATE FEEDBACK
    # =========================================================
    def _state_cb(self, msg):
        if self.state_callback_fn:
            self.state_callback_fn(msg.data)

    # =========================================================
    # MODE CONTROL (THIS IS THE CORE OF YOUR NEW SYSTEM)
    # =========================================================
    def set_mode(self, mode: str):

        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

        self.get_logger().info(f"[MODE] => {mode}")

    # =========================================================
    # WOA CONTROL
    # =========================================================
    def start_woa_following(self):

        if self.woa_process:
            self.get_logger().warn("WOA already running.")
            return

        self._cancel_nav()

        self.set_mode("WOA")

        self.woa_process = subprocess.Popen(
            ["ros2", "run", "follow_target_cpp", "woa_dwb_tfc"],
            preexec_fn=os.setsid
        )

        self.get_logger().info("WOA started")

    def stop_woa_following(self):

        if self.woa_process:
            try:
                os.killpg(os.getpgid(self.woa_process.pid), signal.SIGKILL)
            except Exception as e:
                self.get_logger().error(f"WOA stop error: {e}")

        self.woa_process = None

        self.set_mode("STOP")

        self.get_logger().info("WOA stopped")

    # =========================================================
    # NAVIGATION (NAV2)
    # =========================================================
    def send_goal(self, name: str):

        if name not in self.locations:
            self.get_logger().error(f"Unknown location: {name}")
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 not ready")
            return

        self._cancel_nav()

        self.set_mode("NAV")

        x, y, z = self.locations[name]["position"]
        qx, qy, qz, qw = self.locations[name]["orientation"]

        goal = NavigateToPose.Goal()
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal.pose = pose

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_cb)

    def _goal_cb(self, future):

        self.current_goal_handle = future.result()

        if not self.current_goal_handle.accepted:
            self.get_logger().warn("Nav goal rejected")
            self.set_mode("STOP")
            return

        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):

        self.current_goal_handle = None
        self.set_mode("STOP")

        self.get_logger().info("Nav goal finished")

    # =========================================================
    # NAV CANCEL
    # =========================================================
    def _cancel_nav(self):

        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except:
                pass

        self.current_goal_handle = None

    # =========================================================
    # EMERGENCY
    # =========================================================
    def emergency_stop(self):

        self.stop_woa_following()
        self._cancel_nav()

        self.set_mode("STOP")

        self.get_logger().warn("EMERGENCY STOP")

    def stop_robot(self):
        self.emergency_stop()