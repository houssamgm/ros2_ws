#!/usr/bin/env python3

import os
import yaml
import rclpy
from threading import Thread
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class GuideNode(Node):

    def __init__(self):
        super().__init__('guide_node')
        self.get_logger().info("Guide Node Started")

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.locations = self.load_locations()

        # Fire background user execution immediately so main can reach spinning loop
        self.input_thread = Thread(target=self.user_loop, daemon=True)
        self.input_thread.start()

    def load_locations(self):
        package_share = get_package_share_directory('diffbot_sim')
        yaml_path = os.path.join(package_share, 'config', 'supermarket.yaml')

        with open(yaml_path, 'r') as file:
            locations = yaml.safe_load(file)

        self.get_logger().info("Supermarket locations loaded successfully.")
        return locations

    def print_menu(self):
        print("\n===== SUPERMARKET LOCATIONS =====")
        for name in self.locations.keys():
            print(f"- {name}")
        print("=================================\n")

    def user_loop(self):
        self.get_logger().info("Waiting for Nav2 action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Nav2 is ready.")

        import time
        time.sleep(1.0)
        
        while rclpy.ok():
            self.print_menu()
            destination = input("Enter destination (or 'exit'): ").strip()

            if destination == "exit":
                self.get_logger().info("Exiting Guide Node.")
                rclpy.shutdown()
                return

            if destination not in self.locations:
                print(f"[{destination}] is an invalid location. Please try again.")
                continue

            self.send_goal(destination)
            time.sleep(2.0)

    def send_goal(self, destination):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        loc = self.locations[destination]
        pose.pose.position.x = float(loc['position']['x'])
        pose.pose.position.y = float(loc['position']['y'])
        pose.pose.position.z = float(loc['position']['z'])

        pose.pose.orientation.x = float(loc['orientation']['x'])
        pose.pose.orientation.y = float(loc['orientation']['y'])
        pose.pose.orientation.z = float(loc['orientation']['z'])
        pose.pose.orientation.w = float(loc['orientation']['w'])

        goal_msg.pose = pose
        self.get_logger().info(f"Sending robot to target: {destination}")

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal was rejected by Nav2.")
            return

        self.get_logger().info("Goal accepted by Nav2.")
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")

    def get_result_callback(self, future):
        self.get_logger().info("Target location reached successfully!")


def main(args=None):
    rclpy.init(args=args)
    node = GuideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()