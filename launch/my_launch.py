"""
File: my_launch.py
Author: Neha Nitin Madhekar
License: Apache License 2.0

This is a sample launch file for a ROS2 package.

.. note::
   This launch file starts a talker and a listener node.

"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Generate a LaunchDescription for the ROS2 package.

    Returns:
        LaunchDescription: The generated launch description.
    """

    # Declare launch argument for node frequency
    args_Frequency = DeclareLaunchArgument('frequency', default_value=TextSubstitution(text="120"))

    return LaunchDescription([
        args_Frequency,
        Node(
            package='beginner_tutorials',
            executable='talker',
            parameters=[
                {"frequency": LaunchConfiguration('frequency')}
            ],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='beginner_tutorials',
            executable='listener'
        )
    ])