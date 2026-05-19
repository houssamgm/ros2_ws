import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class LeaderCurve(Node):

    def __init__(self):
        super().__init__('leader_curve')

        self.pub = self.create_publisher(Twist, '/leader/cmd_vel', 10)

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.timer = self.create_timer(0.1, self.loop)

    def loop(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        t = now - self.start_time

        cmd = Twist()

        # speed
        cmd.linear.x = 0.65

        # aggressive curved motion
        cmd.angular.z = 1.2 * math.sin(0.8 * t)
        cmd.angular.z = max(min(cmd.angular.z, 1.1), -1.1)
        if int(t) % 4 == 0:
            cmd.angular.z += 1.5 * math.sin(3.0 * t)
            
        cmd.angular.z = max(min(cmd.angular.z, 1.1), -1.1)
        # stop after 30 sec
        if t > 30:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # publish
        self.pub.publish(cmd)

        # logs
        self.get_logger().info(
            f"t={t:.2f} | v={cmd.linear.x:.2f} | w={cmd.angular.z:.2f}"
        )


def main():
    rclpy.init()
    node = LeaderCurve()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()