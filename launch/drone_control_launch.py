from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_project',
            executable='drone_control',
            name='drone_control_node',
            output='screen',
            shell=True,
        ),
    ])


