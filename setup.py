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
            'test_motor0_drive = cargo_beep.motor0_drive:main',
            'test_motor1_drive = cargo_beep.motor1_drive:main',
            'test_imu_publisher = cargo_beep.imu_pub:main',
            'motor_control = cargo_beep.motor_controller:main'
        ],
    },
)
