'''
Author: gaoyuan
Date: 2023-04-10 16:30:46
LastEditors: gaoyuan
LastEditTime: 2023-04-11 18:04:50
'''

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('ros2_galaxy_camera'), 'config', 'camera_params.yaml')

    camera_info_url = 'package://ros2_galaxy_camera/config/camera_info.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                            default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                            default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                            default_value='false'),

        Node(
            package='ros2_galaxy_camera',
            executable='ros2_galaxy_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }],
        )
    ])


