#!/usr/bin/env python3
"""
Launch file for PiDog Autonomous ROS 2 System
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='pidog',
        description='Namespace for all nodes'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get configuration
    debug = LaunchConfiguration('debug')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Package path
    pkg_share = FindPackageShare('pidog_ros2')
    
    # Parameters file
    params_file = PathJoinSubstitution([pkg_share, 'config', 'pidog_params.yaml'])
    
    # Log level
    log_level = 'debug' if debug == 'true' else 'info'
    
    # Set Python path
    set_python_path = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=os.environ.get('PYTHONPATH', '') + ':/ros2_ws/install/pidog_ros2/lib/python3.10/site-packages'
    )
    
    # Common node configuration
    node_config = {
        'namespace': namespace,
        'parameters': [params_file, {'use_sim_time': use_sim_time}],
        'arguments': ['--ros-args', '--log-level', log_level],
        'output': 'screen',
        'emulate_tty': True,
    }
    
    # Define all nodes
    nodes = [
        Node(package='pidog_ros2', executable='pidog_movement_node', name='pidog_movement_node', **node_config),
        Node(package='pidog_ros2', executable='pidog_distance_node', name='pidog_distance_node', **node_config),
        Node(package='pidog_ros2', executable='pidog_camera_node', name='pidog_camera_node', **node_config),
        Node(package='pidog_ros2', executable='pidog_imu_node', name='pidog_imu_node', **node_config),
        Node(package='pidog_ros2', executable='pidog_dual_touch_node', name='pidog_dual_touch_node', **node_config),
        Node(package='pidog_ros2', executable='pidog_direction_sensor_node', name='pidog_direction_sensor_node', **node_config),
        Node(package='pidog_ros2', executable='pidog_tts_speaks_node', name='pidog_tts_speaks_node', **node_config),
        Node(package='pidog_ros2', executable='pidog_stt_voice_command_node', name='pidog_stt_voice_command_node', **node_config),
        Node(package='pidog_ros2', executable='ros2_autonomous_pidog', name='ros2_autonomous_pidog', **node_config),
    ]
    
    return LaunchDescription([
        set_python_path,
        debug_arg,
        namespace_arg,
        use_sim_time_arg,
        LogInfo(msg=['Starting PiDog Autonomous System']),
        LogInfo(msg=['  Namespace: ', namespace]),
        LogInfo(msg=['  Debug mode: ', debug]),
        LogInfo(msg=['  Log level: ', log_level]),
        LogInfo(msg=['  Parameters: ', params_file]),
    ] + nodes)


if __name__ == '__main__':
    generate_launch_description()
