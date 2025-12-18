import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('vk_ros2_driver'),
                'config', 'example.yaml'),
            description='Full path to the parameters file to use for all launched nodes'),

        Node(
            package='vk_ros2_driver',
            executable='vk_ros2_driver_node',
            name='vk_ros2_driver_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]

        )
      
    ])
