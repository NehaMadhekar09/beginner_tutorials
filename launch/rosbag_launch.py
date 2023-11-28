"""
File: rosbag_launch.py
Author: Neha Nitin Madhekar
License: Apache License 2.0

This launch file launches talker and listener nodes and records the rosbag

"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

"""
    Generate a LaunchDescription for the ROS2 package.

    Returns:
        LaunchDescription: The generated launch description.
"""
def generate_launch_description():
    args_frequency = DeclareLaunchArgument('frequency', default_value='10.0')  
    args_record_bag = DeclareLaunchArgument('record_rosbag', default_value='True', choices=['True', 'False'])  
    rosbag_recorder = ExecuteProcess(
        condition = IfCondition(LaunchConfiguration('record_rosbag')),
        cmd = ['ros2', 'bag', 'record', '-o', 'bag_list', '-a'],
        shell=True
    )
    return LaunchDescription([
        args_frequency,
        args_record_bag,
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker',
            parameters = [
                {"frequency": LaunchConfiguration('frequency')}
            ],
        ),
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='listener'
        ),
        rosbag_recorder
    ])