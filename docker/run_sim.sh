#!/usr/bin/env bash
set -euo pipefail

/workspace/docker/start_vnc.sh

source /opt/ros/noetic/setup.bash

export ROS_PACKAGE_PATH="/workspace/ros_ws/src:${ROS_PACKAGE_PATH:-}"
export GAZEBO_MODEL_PATH="/workspace/ros_ws/src/dual_ur5_gazebo/models:${GAZEBO_MODEL_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export SVGA_VGPU10="${SVGA_VGPU10:-0}"

cd /workspace/ros_ws
if [[ ! -f /workspace/ros_ws/src/CMakeLists.txt ]]; then
  catkin_init_workspace /workspace/ros_ws/src
fi
catkin_make
source /workspace/ros_ws/devel/setup.bash

echo "VNC URL: http://localhost:6080/vnc.html"
echo "Using software GL: LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"

roslaunch dual_ur5_gazebo sim.launch scene_config:="${SCENE_CONFIG:-/workspace/ros_ws/src/dual_ur5_gazebo/config/scene.yaml}"
