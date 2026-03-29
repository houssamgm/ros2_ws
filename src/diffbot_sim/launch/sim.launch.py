import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler,
    ExecuteProcess
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    package_name = 'diffbot_sim'
    use_nav2 = LaunchConfiguration('use_nav2')

    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Launch Nav2 stack'
    )
    # Specify the path to your world file
    world_file_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'round_obstacle.world' 
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('diffbot_sim'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf',
                              'robot.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'diffbot'],
                        output='screen')
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

       
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('diffbot_sim'), 'launch', 'bringup_navigation.py'
        )]),
        launch_arguments={
            'use_sim_time': 'true',
            'log_level': 'ERROR',
            'params_file': os.path.join(
                get_package_share_directory('diffbot_sim'),
                'config',
                'nav2_params.yaml')
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
