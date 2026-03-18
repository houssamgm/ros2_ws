#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


class GuideNode(Node):

    def __init__(self):
        super().__init__('guide_node')

        self.get_logger().info("Guide Node Started")

        # Create Action Client
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Load locations
        self.locations = self.load_locations()

        # Wait for Nav2 server
        self.get_logger().info("Waiting for Nav2 action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Nav2 is ready.")

        # Start interaction loop
        self.run()

    def load_locations(self):
        package_share = get_package_share_directory('diffbot_sim')
        yaml_path = os.path.join(package_share, 'config', 'location.yaml')

        with open(yaml_path, 'r') as file:
            locations = yaml.safe_load(file)

        self.get_logger().info("Locations loaded successfully.")
        return locations

    def run(self):
        while rclpy.ok():
            print("\nAvailable locations:")
            for name in self.locations.keys():
                print(f"- {name}")

            destination = input("\nEnter destination (or 'exit'): ")

            if destination == "exit":
                break

            if destination not in self.locations:
                print("Invalid location. Try again.")
                continue

            self.send_goal(destination)

    def send_goal(self, destination):

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        # Retrieve position
        pose.pose.position.x = self.locations[destination]['position']['x']
        pose.pose.position.y = self.locations[destination]['position']['y']
        pose.pose.position.z = self.locations[destination]['position']['z']

        # Retrieve orientation (already quaternion)
        pose.pose.orientation.x = self.locations[destination]['orientation']['x']
        pose.pose.orientation.y = self.locations[destination]['orientation']['y']
        pose.pose.orientation.z = self.locations[destination]['orientation']['z']
        pose.pose.orientation.w = self.locations[destination]['orientation']['w']

        goal_msg.pose = pose

        self.get_logger().info(f"Navigating to {destination}...")

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {feedback.distance_remaining:.2f} m"
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Goal reached successfully!")


def main(args=None):
    rclpy.init(args=args)
    node = GuideNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()