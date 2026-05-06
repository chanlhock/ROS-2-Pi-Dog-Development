from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Only start the autonomous node - it handles all movements directly
        Node(
            package='pidog_ros2',
            executable='ros2_autonomous_pidog',
            namespace='pidog',
            name='ros2_autonomous_pidog',
            parameters=[{
                'enable_head_scanning': False,
                'enable_sound_turning': False,  # Disable until microphone works
                'enable_face_interaction': True,
                'personality_actions': True,
            }],
            output='screen',
            prefix=['']  # No prefix, run normally
        ),
        
        # Camera node (doesn't use GPIO)
        Node(
            package='pidog_ros2',
            executable='pidog_camera_node',
            namespace='pidog',
            name='pidog_camera_node',
            output='screen',
        ),
        
        # TTS node (doesn't use GPIO)
        Node(
            package='pidog_ros2',
            executable='pidog_tts_speaks_node',
            namespace='pidog',
            name='pidog_tts_speaks_node',
            output='screen',
        ),
        
        # IMU node (simulation - doesn't use hardware)
        Node(
            package='pidog_ros2',
            executable='pidog_imu_node',
            namespace='pidog',
            name='pidog_imu_node',
            parameters=[{
                'simulate_imu': True,
                'use_hardware_imu': False,
            }],
            output='screen',
        ),
        
        # Touch simulation node
        Node(
            package='pidog_ros2',
            executable='pidog_dual_touch_node',
            namespace='pidog',
            name='pidog_dual_touch_node',
            parameters=[{
                'simulate_touch': True,
            }],
            output='screen',
        ),
    ])
