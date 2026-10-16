"""Visualize the SCARA model alone (sliders instead of the real robot).

    ros2 launch scara_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('scara_description')
    xacro_file = os.path.join(share, 'urdf', 'scara.urdf.xacro')
    rviz_file = os.path.join(share, 'rviz', 'scara.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
