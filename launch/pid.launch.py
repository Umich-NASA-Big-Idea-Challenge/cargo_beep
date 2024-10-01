from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cargo_beep',
            executable='pid_lean',
            name='lean'
        ),
        Node(
            package='cargo_beep',
            executable='pid_turning',
            name="turn",
        ),
        Node(
            package='cargo_beep',
            executable='pid_velocity',
            name='velocity'
        ),
        Node(
            package='cargo_beep',
            executable='pid_governor',
            name='governor'
        )
    ])


