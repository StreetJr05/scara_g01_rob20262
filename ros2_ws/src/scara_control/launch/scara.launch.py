"""Full high-level stack for the real robot.

    ros2 launch scara_control scara.launch.py                 # agent + bridge + RViz
    ros2 launch scara_control scara.launch.py dev:=/dev/ttyACM0 pendant:=true
    ros2 launch scara_control scara.launch.py agent:=false     # agent started by hand
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
    desc = get_package_share_directory('scara_description')
    ctrl = get_package_share_directory('scara_control')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc, 'urdf', 'scara.urdf.xacro')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('dev', default_value='/dev/ttyUSB0', description='ESP32 serial port'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('agent', default_value='true', description='start micro_ros_agent'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('pendant', default_value='false', description='open the teach pendant'),
        DeclareLaunchArgument('params', default_value=os.path.join(ctrl, 'config', 'scara_params.yaml')),

        Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
             arguments=['serial', '--dev', LaunchConfiguration('dev'), '-b', LaunchConfiguration('baud')],
             output='screen', condition=IfCondition(LaunchConfiguration('agent'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='scara_control', executable='bridge', name='scara_bridge',
             parameters=[LaunchConfiguration('params')], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(desc, 'rviz', 'scara.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='scara_control', executable='teach_pendant',
             condition=IfCondition(LaunchConfiguration('pendant'))),
    ])
