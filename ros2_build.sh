cd /ros2_ws

# Complete cleanup
rm -rf build/ install/ log/
rm -rf src/pidog_ros2/__pycache__/
rm -rf src/pidog_ros2/pidog_ros2/__pycache__/

# Rebuild with verbose output
colcon build --packages-select pidog_ros2 --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

cd /ros2_ws/install/pidog_ros2
mkdir -p lib/pidog_ros2
ln -sf $(realpath bin/pidog_movement_node) lib/pidog_ros2/
ln -sf $(realpath bin/pidog_distance_node) lib/pidog_ros2/
ln -sf $(realpath bin/pidog_camera_node) lib/pidog_ros2/
ln -sf $(realpath bin/pidog_imu_node) lib/pidog_ros2/
ln -sf $(realpath bin/pidog_dual_touch_node) lib/pidog_ros2/
ln -sf $(realpath bin/pidog_direction_sensor_node) lib/pidog_ros2/
ln -sf $(realpath bin/pidog_tts_speaks_node) lib/pidog_ros2/
ln -sf $(realpath bin/pidog_stt_voice_command_node) lib/pidog_ros2/
ln -sf $(realpath bin/ros2_autonomous_pidog) lib/pidog_ros2/
ln -sf $(realpath bin/pidog_battery_monitor_node) lib/pidog_ros2/

# Now the launch file should work
cd /ros2_ws
source install/setup.bash
rm log.txt
rm core*
clear
#ros2 launch pidog_ros2 pidog_autonomous.launch.py | tee log.txt
#ros2 launch pidog_ros2 pidog_autonomous.launch.py \
#    autonomous_log_level:=INFO \
#    distance_log_level:=INFO \
#    enable_wandering:=False \
#    enable_obstacle_avoidance:=False | tee log.txt

#!/bin/bash

if [ "$1" = "-off" ]; then
    # Off mode - with parameters
    ros2 launch pidog_ros2 pidog_autonomous.launch.py \
        autonomous_log_level:=INFO \
        distance_log_level:=INFO \
        enable_wandering:=False \
        enable_obstacle_avoidance:=False | tee log.txt
else
    # Normal mode - without parameters
    ros2 launch pidog_ros2 pidog_autonomous.launch.py | tee log.txt
fi


sed -i.bak '/ALSA lib pcm/d' log.txt

