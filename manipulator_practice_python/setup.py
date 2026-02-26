import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'manipulator_practice_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bkyb',
    maintainer_email='bkyb@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realtime_trajectory_test = manipulator_practice_python.realtime_trajectory_test:main',
            'pose_hip = manipulator_practice_python.hip_pose_detector_node:main',
            'dynamic_obstacle_node = manipulator_practice_python.dynamic_obstacle_node:main',
            'map_publisher = manipulator_practice_python.map_publisher:main',
            'gateway = manipulator_practice_python.robot_action_gateway:main',
            'data_player = manipulator_practice_python.data_player:main',
        ],
    },
)