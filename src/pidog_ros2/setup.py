from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pidog_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bernard Chan',
    maintainer_email='chanlhock@gmail.com',
    description='ROS 2 package for autonomous Pi Dog robot control',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'ros2_autonomous_pidog = pidog_ros2.ros2_autonomous_pidog:main',
            'pidog_movement_node = pidog_ros2.pidog_movement_node:main',
            'pidog_distance_node = pidog_ros2.pidog_distance_node:main',
            'pidog_camera_node = pidog_ros2.pidog_camera_node:main',
            'pidog_imu_node = pidog_ros2.pidog_imu_node:main',
            'pidog_dual_touch_node = pidog_ros2.pidog_dual_touch_node:main',
            'pidog_direction_sensor_node = pidog_ros2.pidog_direction_sensor_node:main',
            'pidog_tts_speaks_node = pidog_ros2.pidog_tts_speaks_node:main',
            'pidog_stt_voice_command_node = pidog_ros2.pidog_stt_voice_command_node:main',
            'voice_bridge = pidog_ros2.voice_bridge:main',
        ],
    },
)
