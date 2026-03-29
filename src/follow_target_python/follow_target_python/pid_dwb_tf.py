#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# https://github.com/ashtalekar36/dwa_custom_planner
class PIDFollow(Node):

    def __init__(self):
        super().__init__('pid_follow')

        # ---------- PID parameters ----------
        self.d_ref = 0.7
        self.Kp = 1.2
        self.Ki = 0.0
        self.Kd = 0.2
        self.Kp_angle = 3.0

        self.v_max = 0.6
        self.w_max = 1.5

        # ---------- DWA safety parameters ----------
        self.max_accel = 0.6
        self.max_delta_w = 1.5
        self.predict_time = 0.8
        self.dt_sim = 0.1
        self.v_reso = 0.07
        self.w_reso = 0.25
        self.robot_radius = 0.22

        # PID memory
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # ROS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("PID follower with DWA obstacle layer started")

    # --------------------------------------------------
    def scan_callback(self, msg):
        self.scan = msg

    # --------------------------------------------------
    # ---------------- DWA FUNCTIONS -------------------
    # --------------------------------------------------

    def predict_trajectory(self, v, w):
        traj = []
        x, y, yaw = 0.0, 0.0, 0.0

        t = 0.0
        while t <= self.predict_time:
            x += v * math.cos(yaw) * self.dt_sim
            y += v * math.sin(yaw) * self.dt_sim
            yaw += w * self.dt_sim
            traj.append((x, y))
            t += self.dt_sim

        return traj

    def calc_obstacle_cost(self, traj):

        if self.scan is None:
            return 0.0

        min_dist = float('inf')

        for (x, y) in traj:
            for i, r in enumerate(self.scan.ranges):

                if math.isinf(r) or math.isnan(r):
                    continue

                angle = self.scan.angle_min + i * self.scan.angle_increment

                obs_x = r * math.cos(angle)
                obs_y = r * math.sin(angle)

                dist = math.hypot(x - obs_x, y - obs_y)

                if dist < min_dist:
                    min_dist = dist

        if min_dist == float('inf'):
            return 0.0

        # Hard collision only if extremely close
        if min_dist < self.robot_radius:
            return float('inf')

        # Smooth penalty
        return 1.0 / (min_dist + 0.01)

    def dwa_safety_filter(self, v_pid, w_pid):

        if self.scan is None:
            return v_pid, w_pid

        min_v = max(-self.v_max, v_pid - self.max_accel * self.dt_sim)
        max_v = min(self.v_max, v_pid + self.max_accel * self.dt_sim)

        # widen angular search around forward motion
        angle_expand = 0.6 if abs(v_pid) > 0.05 else 0.3

        min_w = max(-self.w_max, w_pid - self.max_delta_w * self.dt_sim - angle_expand)
        max_w = min(self.w_max, w_pid + self.max_delta_w * self.dt_sim + angle_expand)
        best_v = None
        best_w = None
        min_cost = float('inf')

        for v in np.arange(min_v, max_v + self.v_reso, self.v_reso):
            for w in np.arange(min_w, max_w + self.w_reso, self.w_reso):

                traj = self.predict_trajectory(v, w)
                obstacle_cost = self.calc_obstacle_cost(traj)

                if obstacle_cost == float('inf'):
                    continue

                tracking_cost = abs(v - v_pid) + abs(w - w_pid)

                final_cost = 3.5 * obstacle_cost + 4 * tracking_cost

                if final_cost < min_cost:
                    min_cost = final_cost
                    best_v = v
                    best_w = w

        # If everything unsafe → rotate to search free space
        if best_v is None:
            return 0.0, 0.8

        return best_v, best_w

    # --------------------------------------------------
    # ---------------- CONTROL LOOP --------------------
    # --------------------------------------------------

    def control_loop(self):

        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'leader/base_link',
                rclpy.time.Time())
        except:
            return

        dx = trans.transform.translation.x
        dy = trans.transform.translation.y

        distance = math.sqrt(dx * dx + dy * dy)

        # ---------- Distance PID ----------
        error = distance - self.d_ref

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt == 0:
            return

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        v = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        v = max(min(v, self.v_max), -self.v_max)

        # ---------- Heading ----------
        angle_error = math.atan2(dy, dx)
        w = self.Kp_angle * angle_error
        w = max(min(w, self.w_max), -self.w_max)

        # ---------- DWA Obstacle Layer ----------
        v, w = self.dwa_safety_filter(v, w)

        # ---------- Publish ----------
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        self.prev_error = error
        self.prev_time = now

        self.get_logger().info(f"dist={distance:.2f}  v={v:.2f}  w={w:.2f}")


def main():
    rclpy.init()
    node = PIDFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()