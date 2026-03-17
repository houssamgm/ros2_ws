#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class UWBDistance(Node):

    def __init__(self):
        super().__init__('uwb_distance')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.2, self.compute_distance)

        self.get_logger().info("UWB distance simulator started")

    def compute_distance(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',               # follower frame
                'leader/base_link',        # leader frame
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn("TF not ready")
            return

        dx = trans.transform.translation.x
        dy = trans.transform.translation.y

        distance = math.sqrt(dx*dx + dy*dy)

        self.get_logger().info(f"Distance: {distance:.3f} m")

def main():
    rclpy.init()
    node = UWBDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()