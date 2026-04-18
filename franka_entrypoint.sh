#!/bin/bash

# Clone Franka dependencies into the workspace
vcs import /ros2_ws/src < /ros2_ws/src/dependency.repos --recursive --skip-existing

# Apply manage_overruns patch to hardware_interface (not yet upstream)
if [ -d /ros2_ws/src/ros2_control/hardware_interface ]; then
  git -C /ros2_ws/src/ros2_control apply /ros2_ws/src/patches/manage_overruns.patch --verbose 2>&1 || \
    echo "Warning: manage_overruns patch may already be applied or failed to apply"
fi

exec "$@"
