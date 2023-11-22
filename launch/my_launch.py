from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    args_Frequency = DeclareLaunchArgument('frequency', default_value=TextSubstitution(text="150"))

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