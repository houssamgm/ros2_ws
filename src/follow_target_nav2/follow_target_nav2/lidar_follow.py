#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PIDFollow(Node):

    def __init__(self):
        super().__init__('pid_follow_lidar')

        # --- Paramètres PID ---
        self.d_ref = 0.7
        self.Kp = 1.2
        self.Ki = 0.0
        self.Kd = 0.2
        self.Kp_angle = 3.0

        self.v_max = 0.6
        self.w_max = 1.5

        # --- Paramètres de filtrage ---
        self.min_range_cutoff = 0.15  # Ignore tout ce qui est à moins de 15cm (le robot lui-même)
        self.max_range_cutoff = 5.0   # Ignore ce qui est trop loin pour être le leader
        self.angle_limit = math.radians(30) # Filtre frontal strict ±30°

        # PID memory
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan = None

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("PID Follower optimisé : Filtre ±30° activé.")

    def scan_callback(self, msg):
        self.scan = msg

    def detect_leader(self):
        if self.scan is None:
            return None, None

        ranges = self.scan.ranges
        min_dist = float('inf')
        min_angle = 0.0
        found = False

        for i, r in enumerate(ranges):
            # 1. Filtrage des valeurs invalides
            if math.isinf(r) or math.isnan(r):
                continue

            # 2. Filtrage de la distance (bruit de proximité et trop loin)
            if r < self.min_range_cutoff or r > self.max_range_cutoff:
                continue

            # 3. Filtrage angulaire (Zone frontale ±30°)
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if abs(angle) > self.angle_limit:
                continue

            # 4. Recherche du point le plus proche
            if r < min_dist:
                min_dist = r
                min_angle = angle
                found = True

        return (min_dist, min_angle) if found else (None, None)

    def control_loop(self):
        distance, angle = self.detect_leader()
        cmd = Twist()

        if distance is None:
            self.cmd_pub.publish(cmd) # Stop
            return

        # --- Calculs PID ---
        error = distance - self.d_ref
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9

        if dt <= 0: return

        # Terme Intégral avec anti-windup simple
        self.integral += error * dt
        
        # Terme Dérivé
        derivative = (error - self.prev_error) / dt

        # Commande Linéaire (v)
        v = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        v = max(min(v, self.v_max), -self.v_max)

        # Commande Angulaire (w)
        # On utilise directement l'angle du point le plus proche
        w = self.Kp_angle * angle
        w = max(min(w, self.w_max), -self.w_max)

        # Publication
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # Mise à jour mémoire
        self.prev_error = error
        self.prev_time = now

def main():
    rclpy.init()
    node = PIDFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
