#!/usr/bin/env python3

import math
import numpy as np
import time 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WOA_Follow(Node):

    def __init__(self):
        super().__init__('woa_follow')

        # ---------------- Parameters ----------------
        self.d_ref = 0.7
        self.stop_threshold = 0.05

        self.v_max = 0.6
        self.w_max = 3.0

        self.alpha = 2.0
        self.beta = 1.5

        self.n_whales = 25
        self.max_iter = 10

        self.dt = 0.6
        self.b = 1.0

        # 🔥 ROBOT SIZE
        self.robot_radius = 0.22
        self.safety_margin = 0.05

        # ---------- Detection / tracking ----------
        self.track_gate = 0.50
        self.max_lost_cycles = 6
        self.lost_cycles = 0
        self.prev_target = None

        # Smoothing
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.smooth_gain = 0.2

        # ---------------- ROS ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("WOA follower with HARD safety + LiDAR tracking")

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
            if self.prev_target is not None and self.lost_cycles < self.max_lost_cycles:
                self.lost_cycles += 1
                return self.prev_target
            return None

        if self.prev_target is None:
            self.lost_cycles = 0
            return min(candidates, key=lambda p: math.hypot(p[0], p[1]))

        px, py = self.prev_target

        gated_candidates = [
            p for p in candidates
            if math.hypot(p[0] - px, p[1] - py) < self.track_gate
        ]

        if gated_candidates:
            self.lost_cycles = 0
            return min(gated_candidates, key=lambda p: math.hypot(p[0] - px, p[1] - py))

        if self.lost_cycles < self.max_lost_cycles:
            self.lost_cycles += 1
            return self.prev_target

        self.lost_cycles = 0
        return min(candidates, key=lambda p: math.hypot(p[0], p[1]))

    # --------------------------------------------------
    # 🔥 HARD COLLISION CHECK
    # --------------------------------------------------

    def is_safe(self, v, w):

        if self.scan is None:
            return True

        theta = w * self.dt
        new_x = v * math.cos(theta)
        new_y = v * math.sin(theta)

        for i, r in enumerate(self.scan.ranges):

            if math.isinf(r) or math.isnan(r):
                continue

            angle = self.scan.angle_min + i * self.scan.angle_increment

            if abs(angle) > math.pi / 3:
                continue

            obs_x = r * math.cos(angle)
            obs_y = r * math.sin(angle)

            dist = math.hypot(new_x - obs_x, new_y - obs_y)

            if dist < (self.robot_radius + self.safety_margin):
                return False

        return True

    # --------------------------------------------------
    def predict_relative_state(self, dx, dy, v, w):

        x_rel = dx - v * self.dt
        y_rel = dy

        th = -w * self.dt
        c = math.cos(th)
        s = math.sin(th)

        dx_next = c * x_rel - s * y_rel
        dy_next = s * x_rel + c * y_rel

        return dx_next, dy_next

    # --------------------------------------------------
    def fitness(self, candidate, dx, dy):

        v = float(candidate[0])
        w = float(candidate[1])

        if not self.is_safe(v, w):
            return float('inf')

        dx_p, dy_p = self.predict_relative_state(dx, dy, v, w)

        d = math.hypot(dx_p, dy_p)
        theta = math.atan2(dy_p, dx_p)

        e_d = abs(d - self.d_ref)
        e_theta = abs(theta)

        effort = 0.05 * abs(v) + 0.02 * abs(w)

        return self.alpha * e_d + self.beta * e_theta + effort

    # --------------------------------------------------
    def woa_optimize(self, dx, dy):

        lb = np.array([-self.v_max, -self.w_max])
        ub = np.array([ self.v_max,  self.w_max])

        whales = np.random.uniform(lb, ub, size=(self.n_whales, 2))

        fitness_vals = np.array([self.fitness(w, dx, dy) for w in whales])

        if np.all(np.isinf(fitness_vals)):
            return 0.0, 0.8

        best_idx = np.argmin(fitness_vals)
        X_best = whales[best_idx].copy()
        F_best = fitness_vals[best_idx]

        for t in range(self.max_iter):

            a = 2.0 - 2.0 * (t / float(self.max_iter))

            for i in range(self.n_whales):

                r1 = np.random.rand(2)
                r2 = np.random.rand(2)

                A = 2.0 * a * r1 - a
                C = 2.0 * r2

                p = np.random.rand()
                l = np.random.uniform(-1.0, 1.0)

                Xi = whales[i].copy()

                if p < 0.5:
                    if np.linalg.norm(A) < 1.0:
                        D = np.abs(C * X_best - Xi)
                        X_new = X_best - A * D
                    else:
                        rand_idx = np.random.randint(self.n_whales)
                        X_rand = whales[rand_idx].copy()
                        D = np.abs(C * X_rand - Xi)
                        X_new = X_rand - A * D
                else:
                    D_prime = np.abs(X_best - Xi)
                    X_new = D_prime * np.exp(self.b * l) * np.cos(2.0 * math.pi * l) + X_best

                whales[i] = np.clip(X_new, lb, ub)

            fitness_vals = np.array([self.fitness(w, dx, dy) for w in whales])

            if np.all(np.isinf(fitness_vals)):
                return 0.0, 0.8

            current_best_idx = np.argmin(fitness_vals)

            if fitness_vals[current_best_idx] < F_best:
                F_best = fitness_vals[current_best_idx]
                X_best = whales[current_best_idx].copy()

        return float(X_best[0]), float(X_best[1])

    # --------------------------------------------------
    def control_loop(self):
        start_time = time.perf_counter()   


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

        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)

        if abs(distance - self.d_ref) < self.stop_threshold and abs(heading) < 0.08:
            v_cmd = 0.0
            w_cmd = 0.0
        else:
            v_cmd, w_cmd = self.woa_optimize(dx, dy)

        v = (1.0 - self.smooth_gain) * v_cmd + self.smooth_gain * self.prev_v
        w = (1.0 - self.smooth_gain) * w_cmd + self.smooth_gain * self.prev_w

        self.prev_v = v
        self.prev_w = w

        cmd = Twist()
        cmd.linear.x = float(np.clip(v, -self.v_max, self.v_max))
        cmd.angular.z = float(np.clip(w, -self.w_max, self.w_max))
        self.cmd_pub.publish(cmd)
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0

        self.get_logger().info(
            f"FULL LOOP = {elapsed_ms:.2f} ms | "
            f"dist={distance:.2f} v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}"
        )
        self.get_logger().info(
            f"dist={distance:.2f} v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}"
        )
        # Python WOA + LiDAR timing:

        # - Min: ~2 ms (steady state / no movement)
        # - Max: ~335 ms (rare spike)
        # - Avg: ~58–60 ms
        
        # → Usually real-time safe for 10 Hz (100 ms)
        # → BUT NOT deterministic (spikes exist)
        # → Occasional deadline miss (very important)

        # → CPU usage fluctuates depending on WOA convergence


def main(args=None):
    rclpy.init(args=args)
    node = WOA_Follow()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()