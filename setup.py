import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cargo_beep'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # LAUNCH FILES
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='jaybrow@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor0_drive = cargo_beep.motor0_drive:main',
            'motor1_drive = cargo_beep.motor1_drive:main',
            'imu_publisher = cargo_beep.imu_pub:main',
            "keyboard_control = cargo_beep.keyboard_controller:main",
            "joystick_control = cargo_beep.joystick_controller:main",
            "get_setpoint = cargo_beep.get_setpoint:main",
            "traction_test = cargo_beep.traction_testing:main",
            'pid_lean = cargo_beep.pid_lean:main',
            "pid_turning =  cargo_beep.pid_turning:main",
            "pid_velocity = cargo_beep.pid_velocity:main",
            "pid_governor = cargo_beep.pid_governor:main",
            "state_space = cargo_beep.state_space:main"
        ],
    },
)