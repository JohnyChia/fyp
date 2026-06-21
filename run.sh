#!/bin/bash

echo "--- Killing ROS ---"
pkill -9 -f ros2 || true
pkill -9 -f gz || true
pkill -9 -f vision_node || true

sleep 1

echo "--- Cleaning DDS shared memory ---"
rm -rf /dev/shm/fastdds* /dev/shm/fastrtps*

cd ~/durian_ws

echo "--- Sourcing ROS ---"
source /opt/ros/humble/setup.bash

echo "--- Build ---"
colcon build --symlink-install

if [ $? -ne 0 ]; then
  echo "❌ Build failed"
  exit 1
fi

source install/setup.bash

echo "--- Check package ---"
ros2 pkg list | grep durian_inspection_pkg

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/johny/cyclonedds.xml
export ROS_DOMAIN_ID=0
export CUDA_VISIBLE_DEVICES=0

echo "Waiting system stabilize..."
sleep 2

echo "--- Launch Nav2 ---"
ros2 launch durian_inspection_pkg navigation_launch.py
# ros2 launch durian_inspection_pkg mapping_launch.py