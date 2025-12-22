#!/usr/bin/env zsh
export CC=/usr/bin/gcc-13 
export CXX=/usr/bin/g++-13
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source install/setup.zsh
ros2 run camera_imu_syncer camera_imu_syncer_node
