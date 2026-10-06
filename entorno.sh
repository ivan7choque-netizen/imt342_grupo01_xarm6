#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "$HOME/grupo_01_xarm6_ws/install/setup.bash" ]; then
  source "$HOME/grupo_01_xarm6_ws/install/setup.bash"
fi
