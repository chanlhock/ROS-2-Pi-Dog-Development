cd /ros2_ws
source install/setup.bash
rm log.txt
rm core*
clear

ros2 run rviz2 rviz2 -d /ros2_ws/urdf/urdf_config.rviz
