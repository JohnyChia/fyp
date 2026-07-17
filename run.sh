#!/bin/bash

echo "============================"
echo "   ROS2 ROBOT STARTUP"
echo "============================"

echo "--- Killing robot nodes ---"
pkill -9 -f vision_node || true
pkill -9 -f depth_node || true
pkill -9 -f bridge_node || true
pkill -9 -f mapping_launch || true
pkill -9 -f nav2 || true

sleep 1

echo "--- Cleaning DDS shared memory ---"
rm -rf /dev/shm/fastdds* /dev/shm/fastrtps* || true

echo "--- Building workspace ---"
cd ~/durian_ws || { echo "Failed to enter workspace"; exit 1; }

rm -rf build/ install/ log/

echo "--- Building all packages in workspace ---"
colcon build --symlink-install || { echo "Build failed"; exit 1; }

source install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GAZEBO_MODEL_PATH=$HOME/.gazebo/models
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=0

export FASTRTPS_DEFAULT_PROFILES_FILE=""
export RCUTILS_LOGGING_BUFFERED_STREAM=1

echo "--- DDS sanity check ---"
timeout 3 ros2 run demo_nodes_cpp talker &
sleep 2
pkill -f talker || true

echo "--- Launch system ---"
# ros2 launch durian_inspection_pkg mapping_launch.py
ros2 launch durian_inspection_pkg navigation_launch.py