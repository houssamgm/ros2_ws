#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401 (needed for Buffer.transform)


def yaw_from_quat(q):
    # quaternion -> yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FollowLeader(Node):
    def __init__(self):
        super().__init__('follow_leader')

        # --- Params ---
        self.declare_parameter('leader_odom_topic', '/leader/odom')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('follower_base_frame', 'base_link')
        self.declare_parameter('follow_distance', 0.8)      # meters behind leader
        self.declare_parameter('send_rate_hz', 2.0)         # how often we *try* to send
        self.declare_parameter('min_goal_move', 0.25)       # only resend if goal moved this much
        self.declare_parameter('goal_timeout_sec', 0.0)     # 0 = let Nav2 handle

        self.leader_odom_topic = self.get_parameter('leader_odom_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.follower_base_frame = self.get_parameter('follower_base_frame').value
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.min_goal_move = float(self.get_parameter('min_goal_move').value)

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Nav2 action ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- Leader state ---
        self.last_leader_odom = None

        # --- Goal bookkeeping ---
        self.last_sent_goal_xy = None

        # Sub
        self.create_subscription(Odometry, self.leader_odom_topic, self.on_leader_odom, 10)

        # Timer
        period = 1.0 / max(self.send_rate_hz, 0.1)
        self.create_timer(period, self.tick)

        self.get_logger().info(f"Listening leader odom: {self.leader_odom_topic}")
        self.get_logger().info(f"Using frames: global={self.global_frame}, follower_base={self.follower_base_frame}")
        self.get_logger().info("Waiting for Nav2 action server: /navigate_to_pose ...")

    def on_leader_odom(self, msg: Odometry):
        self.last_leader_odom = msg

    def tick(self):
        if self.last_leader_odom is None:
            return

        if not self.nav_client.server_is_ready():
            # don’t spam logs too hard
            self.get_logger().warn("Nav2 action server not ready yet (/navigate_to_pose).")
            return

        # 1) Build leader pose in leader/odom frame from Odometry
        leader_ps = PoseStamped()
        leader_ps.header = self.last_leader_odom.header  # frame_id should be leader/odom
        leader_ps.pose = self.last_leader_odom.pose.pose

        # 2) Transform leader pose -> global_frame (map)
        try:
            leader_in_map = self.tf_buffer.transform(leader_ps, self.global_frame, timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f"TF transform failed ({leader_ps.header.frame_id} -> {self.global_frame}): {e}")
            return

        # 3) Compute follow target behind leader in map frame
        q = leader_in_map.pose.orientation
        yaw = yaw_from_quat(q)

        goal_x = leader_in_map.pose.position.x - self.follow_distance * math.cos(yaw)
        goal_y = leader_in_map.pose.position.y - self.follow_distance * math.sin(yaw)

        # 4) Only resend if goal moved enough
        if self.last_sent_goal_xy is not None:
            dx = goal_x - self.last_sent_goal_xy[0]
            dy = goal_y - self.last_sent_goal_xy[1]
            if math.hypot(dx, dy) < self.min_goal_move:
                return

        # 5) Send Nav2 goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = leader_in_map.pose.orientation  # face same direction as leader

        self.get_logger().info(f"Sending goal: ({goal_x:.2f}, {goal_y:.2f}) in {self.global_frame}")

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

        self.last_sent_goal_xy = (goal_x, goal_y)

    def on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Send goal failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            return

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        try:
            res = future.result().result
            status = future.result().status
            self.get_logger().info(f"Nav2 result status: {status}")
        except Exception as e:
            self.get_logger().error(f"Result failed: {e}")


def main():
    rclpy.init()
    node = FollowLeader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()