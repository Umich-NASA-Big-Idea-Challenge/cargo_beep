from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cargo_beep',
            executable='imu_publisher',
            name="imu",
        ),
        Node(
            package='cargo_beep',
            executable='motor0_drive',
            name='dev0'
        ),
        Node(
            package='cargo_beep',
            executable='motor1_drive',
            name='dev1'
        )
    ])