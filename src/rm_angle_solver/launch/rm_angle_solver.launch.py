'''
Author: gaoyuan
Date: 2023-06-12 08:52:02
LastEditors: gaoyuan
LastEditTime: 2023-06-12 09:10:32
'''
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    params_file = os.path.join(
        # get_package_share_directory('rm_port'), 'config', 'guard_port.yaml')
        get_package_share_directory('rm_angle_solver'), 'config', 'rm_angle_solver.yaml')

    
    with open(params_file, 'r') as f:
        rm_angle_solver_params = yaml.safe_load(f)['/rm_angle_solver']['ros__parameters']

    # rm_port_container = ComposableNodeContainer(
    #         name='rm_port_container',
    #         namespace='',
    #         package='rclcpp_components',
    #         executable='component_container',
    #         emulate_tty=True,
    #         # prefix=[  # Sudo command cause need to be sudoer when we do this node cause it real time
    #         #     "sudo -E env PATH=",
    #         #     EnvironmentVariable("PATH", default_value="${PATH}"),
    #         #     " LD_LIBRARY_PATH=",
    #         #     EnvironmentVariable("LD_LIBRARY_PATH", default_value="${LD_LIBRARY_PATH}"),
    #         #     " PYTHONPATH=",
    #         #     EnvironmentVariable("PYTHONPATH", default_value="${PYTHONPATH}"),
    #         #     " HOME=/tmp ",
    #         # ],
    #         composable_node_descriptions=[
    #             ComposableNode(
    #                 package='rm_port',
    #                 plugin='guard_port::rm_port',
    #                 name='guard_port_node',
    #                 parameters=[rm_port_3a_params],
    #                 # remappings=[('/joint_states', '/zed2/joint_states'),],
    #                 )
    #         ],
    #         output='screen',
    # )

    # return LaunchDescription([rm_port_container])


    rm_angle_solver_node = Node(
        package='rm_angle_solver',
        executable='rm_angle_solver',
        name='rm_angle_solver',
        output='screen',
        parameters=[rm_angle_solver_params],
        emulate_tty=True,
        # prefix=[  # Sudo command cause need to be sudoer when we do this node cause it real time
        #     "sudo -E env PATH=",
        #     EnvironmentVariable("PATH", default_value="${PATH}"),
        #     " LD_LIBRARY_PATH=",
        #     EnvironmentVariable("LD_LIBRARY_PATH", default_value="${LD_LIBRARY_PATH}"),
        #     " PYTHONPATH=",
        #     EnvironmentVariable("PYTHONPATH", default_value="${PYTHONPATH}"),
        #     " HOME=/tmp ",
        # ],
        respawn ="true",
    )
    return LaunchDescription([
        rm_angle_solver_node
        ])