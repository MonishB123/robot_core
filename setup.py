from setuptools import setup

package_name = 'robot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ufset',
    maintainer_email='mannyjainlakes@gmail.com',
    description='Core robot nodes (lidar, imu, motors, tf broadcaster)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Each node must have a main() function
            'lidar_node = robot_core.lidar_node:main',
            'imu_node = robot_core.imu_node:main',
            'motor_node = robot_core.motor_node:main',
            'tf_broadcaster = robot_core.tf_broadcaster:main',
        ],
    },
)
