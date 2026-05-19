#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelMux(Node):

    def __init__(self):

        super().__init__('cmd_vel_mux')

        # ================= MODE =================
        self.mode = "STOP"

        # ================= INPUTS =================
        self.nav_cmd = Twist()
        self.woa_cmd = Twist()

        self.create_subscription(Twist, '/nav_cmd_vel', self.nav_cb, 10)
        self.create_subscription(Twist, '/woa_cmd_vel', self.woa_cb, 10)

        # MODE CONTROL FROM GUI
        self.create_subscription(String, '/cmd_vel_mode', self.mode_cb, 10)

        # ================= OUTPUT =================
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("CMD_VEL MUX READY")

    # ================= CALLBACKS =================
    def nav_cb(self, msg):
        self.nav_cmd = msg

    def woa_cb(self, msg):
        self.woa_cmd = msg

    def mode_cb(self, msg):
        self.mode = msg.data
        self.get_logger().info(f"MODE → {self.mode}")

    # ================= LOOP =================
    def loop(self):

        out = Twist()

        if self.mode == "NAV":
            out = self.nav_cmd

        elif self.mode == "WOA":
            out = self.woa_cmd

        elif self.mode == "STOP":
            out = Twist()

        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()