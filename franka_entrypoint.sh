#!/bin/bash

# Clone Franka dependencies into the workspace
vcs import /ros2_ws/src < /ros2_ws/src/dependency.repos --recursive --skip-existing

exec "$@"
