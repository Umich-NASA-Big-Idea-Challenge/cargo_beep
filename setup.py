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
            'pid_control = cargo_beep.pid_controller:main',
            'test_motor_control = cargo_beep.test_motor_control:main',
        ],
    },
)
