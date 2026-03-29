#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


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

        # 🔥 ROBOT SIZE (IMPORTANT FIX)
        self.robot_radius = 0.22
        self.safety_margin = 0.05

        # Smoothing
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.smooth_gain = 0.2

        # ---------------- ROS ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("WOA follower with HARD safety + robot radius")

    # --------------------------------------------------
    def scan_callback(self, msg):
        self.scan = msg

    # --------------------------------------------------
    # 🔥 HARD COLLISION CHECK WITH ROBOT SIZE
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

            # only check front
            if abs(angle) > math.pi / 3:
                continue

            obs_x = r * math.cos(angle)
            obs_y = r * math.sin(angle)

            dist = math.hypot(new_x - obs_x, new_y - obs_y)

            # 🔥 USE ROBOT SIZE HERE
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

        # 🔥 reject unsafe solutions
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

        # 🔥 all unsafe → escape
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

            # 🔥 check again
            if np.all(np.isinf(fitness_vals)):
                return 0.0, 0.8

            current_best_idx = np.argmin(fitness_vals)

            if fitness_vals[current_best_idx] < F_best:
                F_best = fitness_vals[current_best_idx]
                X_best = whales[current_best_idx].copy()

        return float(X_best[0]), float(X_best[1])

    # --------------------------------------------------
    def control_loop(self):

        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'leader/base_link',
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        dx = trans.transform.translation.x
        dy = trans.transform.translation.y

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

        self.get_logger().info(
            f"dist={distance:.2f} v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}"
        )


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