#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# lidar follow pid dwb but follow obtacles
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

        # ---------- Detection / tracking fix ----------
        self.track_gate = 0.50          # max distance from previous target
        self.max_lost_cycles = 6        # hold target briefly before giving up
        self.lost_cycles = 0

        # PID memory
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # Tracking memory
        self.prev_target = None

        # ROS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("PID + LiDAR + DWA follower started")

    # --------------------------------------------------
    def scan_callback(self, msg):
        self.scan = msg

    # --------------------------------------------------
    # ----------- LIDAR PROCESSING ---------------------
    # --------------------------------------------------

    def scan_to_points(self):
        points = []
        for i, r in enumerate(self.scan.ranges):

            if math.isinf(r) or math.isnan(r):
                continue

            angle = self.scan.angle_min + i * self.scan.angle_increment

            # focus front
            if abs(angle) > 1.2:
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            points.append((x, y))

        return points

    def cluster_points(self, points, threshold=0.15):
        clusters = []
        used = [False]*len(points)

        for i in range(len(points)):
            if used[i]:
                continue

            cluster = [points[i]]
            used[i] = True
            queue = [points[i]]

            while queue:
                px, py = queue.pop()

                for j in range(len(points)):
                    if used[j]:
                        continue

                    qx, qy = points[j]
                    if math.hypot(px - qx, py - qy) < threshold:
                        used[j] = True
                        cluster.append((qx, qy))
                        queue.append((qx, qy))

            clusters.append(cluster)

        return clusters

    def compute_cluster_features(self, cluster):
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]

        cx = np.mean(xs)
        cy = np.mean(ys)

        width = max(ys) - min(ys)
        length = max(xs) - min(xs)

        return cx, cy, width, length

    def select_target(self, clusters):

        candidates = []

        for cluster in clusters:
            if len(cluster) < 5:
                continue

            cx, cy, w, l = self.compute_cluster_features(cluster)

            size = math.hypot(w, l)

            if size < 0.08:
                continue

            if size > 0.6:
                continue

            if not (0.08 < w < 0.35 or 0.08 < l < 0.35):
                continue

            candidates.append((cx, cy))

        if not candidates:
            # keep previous target for a few cycles instead of losing it instantly
            if self.prev_target is not None and self.lost_cycles < self.max_lost_cycles:
                self.lost_cycles += 1
                return self.prev_target
            return None

        # first detection: choose nearest
        if self.prev_target is None:
            self.lost_cycles = 0
            target = min(
                candidates,
                key=lambda p: math.hypot(p[0], p[1])
            )
            return target

        # --------------------------------------------------
        # FIX:
        # Strongly prefer target continuity.
        # Do NOT switch easily to another similar obstacle.
        # --------------------------------------------------

        px, py = self.prev_target

        # only consider candidates close to previous target
        gated_candidates = [
            p for p in candidates
            if math.hypot(p[0] - px, p[1] - py) < self.track_gate
        ]

        if gated_candidates:
            self.lost_cycles = 0
            return min(
                gated_candidates,
                key=lambda p: math.hypot(p[0] - px, p[1] - py)
            )

        # if no close candidate found, keep old target briefly
        if self.lost_cycles < self.max_lost_cycles:
            self.lost_cycles += 1
            return self.prev_target

        # only after target is really lost, allow reacquisition
        self.lost_cycles = 0
        return min(
            candidates,
            key=lambda p: math.hypot(p[0], p[1])
        )

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

        if min_dist < self.robot_radius:
            return float('inf')

        return 1.0 / (min_dist + 0.01)

    def dwa_safety_filter(self, v_pid, w_pid):

        if self.scan is None:
            return v_pid, w_pid

        min_v = max(-self.v_max, v_pid - self.max_accel * self.dt_sim)
        max_v = min(self.v_max, v_pid + self.max_accel * self.dt_sim)

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

        if best_v is None:
            return 0.0, 0.8

        return best_v, best_w

    # --------------------------------------------------
    # ---------------- CONTROL LOOP --------------------
    # --------------------------------------------------

    def control_loop(self):

        if self.scan is None:
            return

        points = self.scan_to_points()
        clusters = self.cluster_points(points)
        target = self.select_target(clusters)

        if target is None:
            self.get_logger().warn("Leader not detected")
            return

        self.prev_target = target

        dx, dy = target
        distance = math.sqrt(dx * dx + dy * dy)

        # ---------- PID ----------
        error = distance - self.d_ref

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt == 0:
            return

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        v = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        v = max(min(v, self.v_max), -self.v_max)

        angle_error = math.atan2(dy, dx)
        w = self.Kp_angle * angle_error
        w = max(min(w, self.w_max), -self.w_max)

        # ---------- DWA (UNCHANGED) ----------
        v, w = self.dwa_safety_filter(v, w)

        # ---------- Publish ----------
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        self.prev_error = error
        self.prev_time = now

        self.get_logger().info(f"dist={distance:.2f} v={v:.2f} w={w:.2f}")

def main():
    rclpy.init()
    node = PIDFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
