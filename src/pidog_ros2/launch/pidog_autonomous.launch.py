from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments for log levels only
    declared_args = [
        DeclareLaunchArgument('log_level', default_value='INFO', 
                              description='Default log level for all nodes'),
        DeclareLaunchArgument('autonomous_log_level', default_value='INFO',
                              description='Log level for autonomous node'),
        DeclareLaunchArgument('movement_log_level', default_value='INFO',
                              description='Log level for movement node'),
        DeclareLaunchArgument('distance_log_level', default_value='INFO',
                              description='Log level for distance node'),
        DeclareLaunchArgument('camera_log_level', default_value='INFO',
                              description='Log level for camera node'),
        DeclareLaunchArgument('imu_log_level', default_value='INFO',
                              description='Log level for IMU node'),
        DeclareLaunchArgument('touch_log_level', default_value='INFO',
                              description='Log level for touch node'),
        DeclareLaunchArgument('direction_log_level', default_value='INFO',
                              description='Log level for direction sensor node'),
        DeclareLaunchArgument('tts_log_level', default_value='INFO',
                              description='Log level for TTS node'),
        DeclareLaunchArgument('stt_log_level', default_value='INFO',
                              description='Log level for STT node'),
        DeclareLaunchArgument('battery_log_level', default_value='INFO',
                              description='Log level for battery monitor node'),

        # Node-specific startup arguments
        DeclareLaunchArgument('enable_wandering', default_value='True',
                              description='Enable wandering behavior'),
        DeclareLaunchArgument('enable_obstacle_avoidance', default_value='True',
                              description='Enable obstacle avoidance'),
    ]
    
    # Helper function to create node with log level
    def make_node(name, executable, log_level_arg, extra_args=None, extra_params=None):
        node_args = ['--ros-args', '--log-level', log_level_arg]
        if extra_args:
            node_args.extend(extra_args)
        
        params = extra_params if extra_params else []
        
        return Node(
            package='pidog_ros2',
            executable=executable,
            name=name,
            namespace='pidog',
            output='screen',
            arguments=node_args,
            parameters=params,
            emulate_tty=True,  # Ensures log output is properly displayed
        )
    
    # Create nodes with their specific log levels
    nodes = [
        make_node('ros2_autonomous_pidog', 'ros2_autonomous_pidog',
                 LaunchConfiguration('autonomous_log_level'),
                 extra_params=[
                     {'enable_wandering': LaunchConfiguration('enable_wandering')},
                     {'enable_obstacle_avoidance': LaunchConfiguration('enable_obstacle_avoidance')}
                 ]),
        
        make_node('pidog_movement_node', 'pidog_movement_node',
                 LaunchConfiguration('movement_log_level')),
        
        make_node('pidog_distance_node', 'pidog_distance_node',
                 LaunchConfiguration('distance_log_level')),
        
        make_node('pidog_camera_node', 'pidog_camera_node',
                 LaunchConfiguration('camera_log_level')),
        
        make_node('pidog_imu_node', 'pidog_imu_node',
                 LaunchConfiguration('imu_log_level')),
        
        make_node('pidog_dual_touch_node', 'pidog_dual_touch_node',
                 LaunchConfiguration('touch_log_level')),
        
        make_node('pidog_direction_sensor_node', 'pidog_direction_sensor_node',
                 LaunchConfiguration('direction_log_level')),
        
        make_node('pidog_tts_speaks_node', 'pidog_tts_speaks_node',
                 LaunchConfiguration('tts_log_level')),
        
        make_node('pidog_stt_voice_command_node', 'pidog_stt_voice_command_node',
                 LaunchConfiguration('stt_log_level')),
        
        make_node('pidog_battery_monitor_node', 'pidog_battery_monitor_node',
                 LaunchConfiguration('battery_log_level')),
    ]
    
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'),
        SetEnvironmentVariable('PYTHONWARNINGS', 'ignore'),
        *declared_args,
        *nodes,
    ])