cd /ros2_ws
source install/setup.bash
rm log.txt
rm core*
clear

ros2 launch urdf_tutorial display.launch.py model:=/ros2_ws/urdf/pidog.urdf.xacro
