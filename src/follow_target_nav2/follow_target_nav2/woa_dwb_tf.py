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

        self.alpha = 2.0   # distance error weight
        self.beta = 1.5    # heading error weight

        self.n_whales = 25
        self.max_iter = 10

        self.dt = 0.6
        self.b = 1.0       # spiral constant from WOA paper

        # ---------- DWA safety parameters ----------
        self.max_accel = 0.6
        self.max_delta_w = 1.5
        self.predict_time = 0.8
        self.dt_sim = 0.1
        self.v_reso = 0.07
        self.w_reso = 0.25
        self.robot_radius = 0.22

        # Optional smoothing
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

        self.get_logger().info("WOA follower with DWA obstacle layer started")

    # --------------------------------------------------
    def scan_callback(self, msg):
        self.scan = msg

    # --------------------------------------------------
    # ----------- ROBOT MOTION MODEL -------------------
    # --------------------------------------------------

    def predict_relative_state(self, dx, dy, v, w):
        """
        One-step relative motion prediction in robot frame.
        Candidate solution is X = [v, w].
        This is not part of WOA itself; it is only used to evaluate
        the fitness of a candidate control in a robotics setting.
        """

        # Relative translation due to robot forward motion
        x_rel = dx - v * self.dt
        y_rel = dy

        # Relative rotation due to robot angular motion
        th = -w * self.dt
        c = math.cos(th)
        s = math.sin(th)

        dx_next = c * x_rel - s * y_rel
        dy_next = s * x_rel + c * y_rel

        return dx_next, dy_next

    # --------------------------------------------------
    # ---------------- FITNESS -------------------------
    # --------------------------------------------------

    def fitness(self, candidate, dx, dy):
        """
        Evaluate a whale position X = [v, w].
        """
        v = float(candidate[0])
        w = float(candidate[1])

        dx_p, dy_p = self.predict_relative_state(dx, dy, v, w)

        d = math.hypot(dx_p, dy_p)
        theta = math.atan2(dy_p, dx_p)

        e_d = abs(d - self.d_ref)
        e_theta = abs(theta)

        # Small regularization to avoid aggressive motion
        effort = 0.05 * abs(v) + 0.02 * abs(w)

        return self.alpha * e_d + self.beta * e_theta + effort

    # --------------------------------------------------
    # ---------------- WOA -----------------------------
    # --------------------------------------------------

    def woa_optimize(self, dx, dy):
        """
        Original-style WOA over X = [v, w], adapted for control.
        Uses:
          - Eq. (2.1), (2.2): encircling best
          - Eq. (2.5): spiral update
          - Eq. (2.7), (2.8): exploration using random whale
        """

        lb = np.array([-self.v_max, -self.w_max], dtype=float)
        ub = np.array([ self.v_max,  self.w_max], dtype=float)
        dim = 2

        # Initialize whales population
        whales = np.random.uniform(lb, ub, size=(self.n_whales, dim))

        # Evaluate initial population
        fitness_vals = np.array([self.fitness(whales[i], dx, dy) for i in range(self.n_whales)])
        best_idx = np.argmin(fitness_vals)
        X_best = whales[best_idx].copy()
        F_best = fitness_vals[best_idx]

        # Main WOA loop
        for t in range(self.max_iter):
            # a decreases linearly from 2 to 0 (paper)
            a = 2.0 - 2.0 * (t / float(self.max_iter))

            for i in range(self.n_whales):
                r1 = np.random.rand(dim)
                r2 = np.random.rand(dim)

                A = 2.0 * a * r1 - a
                C = 2.0 * r2

                p = np.random.rand()
                l = np.random.uniform(-1.0, 1.0)

                Xi = whales[i].copy()

                if p < 0.5:
                    if np.linalg.norm(A) < 1.0:
                        # Exploitation: encircling prey (best whale)
                        # D = |C * X_best - X_i|
                        D = np.abs(C * X_best - Xi)
                        # X(t+1) = X_best - A * D
                        X_new = X_best - A * D
                    else:
                        # Exploration: search around a random whale
                        rand_idx = np.random.randint(self.n_whales)
                        X_rand = whales[rand_idx].copy()

                        D = np.abs(C * X_rand - Xi)
                        X_new = X_rand - A * D
                else:
                    # Spiral updating around the best whale
                    D_prime = np.abs(X_best - Xi)
                    X_new = D_prime * np.exp(self.b * l) * np.cos(2.0 * math.pi * l) + X_best

                # Boundary handling
                X_new = np.clip(X_new, lb, ub)
                whales[i] = X_new

            # Re-evaluate population after all updates
            fitness_vals = np.array([self.fitness(whales[i], dx, dy) for i in range(self.n_whales)])
            current_best_idx = np.argmin(fitness_vals)

            if fitness_vals[current_best_idx] < F_best:
                F_best = fitness_vals[current_best_idx]
                X_best = whales[current_best_idx].copy()

        return float(X_best[0]), float(X_best[1])

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

    def dwa_safety_filter(self, v_cmd, w_cmd):

        if self.scan is None:
            return v_cmd, w_cmd

        min_v = max(-self.v_max, v_cmd - self.max_accel * self.dt_sim)
        max_v = min(self.v_max, v_cmd + self.max_accel * self.dt_sim)

        # widen angular search around forward motion
        angle_expand = 0.6 if abs(v_cmd) > 0.05 else 0.3

        min_w = max(-self.w_max, w_cmd - self.max_delta_w * self.dt_sim - angle_expand)
        max_w = min(self.w_max, w_cmd + self.max_delta_w * self.dt_sim + angle_expand)

        best_v = None
        best_w = None
        min_cost = float('inf')

        for v in np.arange(min_v, max_v + self.v_reso, self.v_reso):
            for w in np.arange(min_w, max_w + self.w_reso, self.w_reso):

                traj = self.predict_trajectory(v, w)
                obstacle_cost = self.calc_obstacle_cost(traj)

                if obstacle_cost == float('inf'):
                    continue

                tracking_cost = abs(v - v_cmd) + abs(w - w_cmd)
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
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        dx = trans.transform.translation.x
        dy = trans.transform.translation.y

        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)

        # Stop if already close enough and aligned enough
        if abs(distance - self.d_ref) < self.stop_threshold and abs(heading) < 0.08:
            v_cmd = 0.0
            w_cmd = 0.0
        else:
            v_cmd, w_cmd = self.woa_optimize(dx, dy)

        # ---------- DWA Obstacle Layer ----------
        v_cmd, w_cmd = self.dwa_safety_filter(v_cmd, w_cmd)

        # Small smoothing to reduce jitter
        v = (1.0 - self.smooth_gain) * v_cmd + self.smooth_gain * self.prev_v
        w = (1.0 - self.smooth_gain) * w_cmd + self.smooth_gain * self.prev_w

        self.prev_v = v
        self.prev_w = w

        cmd = Twist()
        cmd.linear.x = float(np.clip(v, -self.v_max, self.v_max))
        cmd.angular.z = float(np.clip(w, -self.w_max, self.w_max))
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"v={cmd.linear.x:.2f} w={cmd.angular.z:.2f} dist={distance:.2f} ang={heading:.2f}"
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