from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{name}]: {message}'),
        # Suppress Python warnings and broken pipe errors
        SetEnvironmentVariable('PYTHONWARNINGS', 'ignore'),
        Node(
            package='pidog_ros2',
            executable='ros2_autonomous_pidog',
            name='ros2_autonomous_pidog',
            namespace='pidog',
            output='screen'
        ),
        Node(
            package='pidog_ros2',
            executable='pidog_movement_node',
            name='pidog_movement_node',
            namespace='pidog',
            output='screen'
        ),
        Node(
            package='pidog_ros2',
            executable='pidog_distance_node',
            name='pidog_distance_node',
            namespace='pidog',
            output='screen'
        ),
        Node(
            package='pidog_ros2',
            executable='pidog_camera_node',
            name='pidog_camera_node',
            namespace='pidog',
            output='screen'
        ),
        Node(
            package='pidog_ros2',
            executable='pidog_imu_node',
            name='pidog_imu_node',
            namespace='pidog',
            output='screen'
        ),
        Node(
            package='pidog_ros2',
            executable='pidog_dual_touch_node',
            name='pidog_dual_touch_node',
            namespace='pidog',
            output='screen'
        ),
        Node(
            package='pidog_ros2',
            executable='pidog_direction_sensor_node',
            name='pidog_direction_sensor_node',
            namespace='pidog',
            output='screen'
        ),
        Node(
            package='pidog_ros2',
            executable='pidog_tts_speaks_node',
            name='pidog_tts_speaks_node',
            namespace='pidog',
            output='screen'
        ),
        Node(
            package='pidog_ros2',
            executable='pidog_stt_voice_command_node',
            name='pidog_stt_voice_command_node',
            namespace='pidog',
            output='screen'
        ),
        Node(
            package='pidog_ros2',
            executable='voice_bridge',
            name='voice_bridge',
            namespace='pidog',
            output='screen'
        ),
    ])
