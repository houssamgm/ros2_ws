import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_name = 'diffbot_sim'
    use_nav2 = LaunchConfiguration('use_nav2')

    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Launch Nav2 stack'
    )

    # =========================================================
    # WORLD PATH (AWS BOOKSTORE)
    # =========================================================
    world_file_path = os.path.expanduser(
        '~/ros2_ws/src/diffbot_sim/aws_worlds/bookstore/worlds/bookstore.world'
    )

    # ✅ FIXED: direct Gazebo launch (IMPORTANT)
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            world_file_path
        ],
        output='screen'
    )

    # =========================================================
    # ROBOT DESCRIPTION
    # =========================================================
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'robot.xacro.urdf'
    )

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'diffbot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3'
        ],
        output='screen'
    )

    # =========================================================
    # SLAM
    # =========================================================
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_sim'),
                'launch',
                'slam.launch.py'
            )
        ),
        launch_arguments={
            'mode': 'mapping',
            'use_sim_time': 'true'
        }.items()
    )

    # =========================================================
    # NAVIGATION
    # =========================================================
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'bringup_navigation.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'log_level': 'ERROR',
            'params_file': os.path.join(
                get_package_share_directory(package_name),
                'config',
                'nav2_params.yaml'
            )
        }.items(),
        condition=IfCondition(use_nav2)
    )

    return LaunchDescription([
        declare_use_nav2,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        slam,
        navigation,
    ])