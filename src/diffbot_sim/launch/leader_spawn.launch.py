from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('diffbot_sim')
    leader_urdf = os.path.join(pkg, 'urdf', 'leader.urdf.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='leader',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', leader_urdf]),
                'use_sim_time': True
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', 'leader',
                '-topic', '/leader/robot_description',
                '-x', '1.0',
                '-y', '0.0',
                '-z', '0.1'
            ]
        )
    ])