'''
Author: gaoyuan
Date: 2023-05-30 00:18:44
LastEditors: 2476240435 2476240435@qq.com
LastEditTime: 2023-11-25 20:25:37
'''
import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription
    
    with open(node_params, 'r') as f:
        if (launch_params['camera'] == 'galaxy'):
            camera_node_params = yaml.safe_load(f)['/galaxy_camera']['ros__parameters']
        elif (launch_params['camera'] == 'hik'):  
            camera_node_params = yaml.safe_load(f)['/hik_camera']['ros__parameters']

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[camera_node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')
    galaxy_camera_node = get_camera_node('ros2_galaxy_camera', 'ros2_galaxy_camera::galaxy_camera_node')

    if (launch_params['camera'] == 'hik'):
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif (launch_params['camera'] == 'mv'):
        cam_detector = get_camera_detector_container(mv_camera_node)
    elif (launch_params['camera'] == 'galaxy'):
        cam_detector = get_camera_detector_container(galaxy_camera_node)

    rm_port_node = Node(
        package='rm_port',
        executable='rm_port_node',
        name='rm_port',
        output='both',
        parameters=[node_params],
        emulate_tty=True,
        on_exit=Shutdown(),
    
    )
    rm_serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[node_params],
    )

    rm_angle_solver_node = Node(
        package='rm_angle_solver',
        executable='rm_angle_solver_node',
        name='rm_angle_solver',
        output='screen',
        parameters=[node_params],
        emulate_tty=True,
        respawn ="true",
    )

    delay_serial_node = TimerAction(
        period=1.5,
        actions=[rm_port_node],
        # actions=[rm_serial_driver_node],
    )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    delay_rm_angle_solver_node = TimerAction(
        period=2.0,
        actions=[rm_angle_solver_node],
    )

    return LaunchDescription([
    delay_serial_node,
    robot_state_publisher,
    cam_detector,
    delay_tracker_node,
    delay_rm_angle_solver_node,
    ])
