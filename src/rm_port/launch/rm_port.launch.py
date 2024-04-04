'''
Author: chengtianle
Date: 2023-04-02 11:48:56
LastEditors: gaoyuan
LastEditTime: 2023-06-21 04:14:50
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
        get_package_share_directory('rm_port'), 'config', 'rm_port.yaml')

    
    with open(params_file, 'r') as f:
        rm_port_params = yaml.safe_load(f)['/rm_port']['ros__parameters']


        rm_port_node = Node(
        package='rm_port',
        executable='rm_port_node',  #指定节点要执行的可执行文件
        name='rm_port',
        output='both',  #设置节点的输出方式
        parameters=[rm_port_params],
        emulate_tty=True,    #启用终端模拟，使得节点的输出在控制台中更易于查看
        # respawn ="true",    #如果节点退出，则自动重新启动节点
    )
        

    return LaunchDescription([
        rm_port_node
        ])