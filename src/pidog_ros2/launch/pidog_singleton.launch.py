from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Autonomous node (initializes PiDog once)
        Node(
            package='pidog_ros2',
            executable='ros2_autonomous_pidog',
            namespace='pidog',
            name='ros2_autonomous_pidog',
            parameters=[{
                'enable_head_scanning': False,
                'enable_sound_turning': True,
                'enable_face_interaction': True,
                'personality_actions': True,
            }],
            output='screen',
        ),
        
        # Movement node (uses shared PiDog)
        Node(
            package='pidog_ros2',
            executable='pidog_movement_node',
            namespace='pidog',
            name='pidog_movement_node',
            parameters=[{
                'enable_hardware': True,
                'use_simulation': False,
            }],
            output='screen',
        ),
        
        # Distance node (simulation - no hardware conflict)
        Node(
            package='pidog_ros2',
            executable='pidog_distance_node',
            namespace='pidog',
            name='pidog_distance_node',
            parameters=[{
                'use_simulation': True,
                'publish_frequency': 10.0,
            }],
            output='screen',
        ),
        
        # Direction sensor (uses shared PiDog)
        Node(
            package='pidog_ros2',
            executable='pidog_direction_sensor_node',
            namespace='pidog',
            name='pidog_direction_sensor_node',
            parameters=[{'enabled': True, 'update_rate': 10.0}],
            output='screen',
        ),
        
        # Touch sensor (simulation)
        Node(
            package='pidog_ros2',
            executable='pidog_dual_touch_node',
            namespace='pidog',
            name='pidog_dual_touch_node',
            parameters=[{'simulate_touch': True}],
            output='screen',
        ),
        
        # Camera node
        Node(
            package='pidog_ros2',
            executable='pidog_camera_node',
            namespace='pidog',
            name='pidog_camera_node',
            output='screen',
        ),
        
        # IMU node (simulation)
        Node(
            package='pidog_ros2',
            executable='pidog_imu_node',
            namespace='pidog',
            name='pidog_imu_node',
            parameters=[{'simulate_imu': True, 'use_hardware_imu': False}],
            output='screen',
        ),
        
        # TTS node
        Node(
            package='pidog_ros2',
            executable='pidog_tts_speaks_node',
            namespace='pidog',
            name='pidog_tts_speaks_node',
            output='screen',
        ),
    ])
