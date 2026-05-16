#!/bin/bash
# ──────────────────────────────────────────────────────────────────
#  BlueROV2 Mission Control — Quick Launcher
#  Run from anywhere:  ~/bluerov2_control_project/launch.sh
# ──────────────────────────────────────────────────────────────────
set -e

# Use CycloneDDS to fix "sequence size exceeds remaining buffer"
# Install once:  sudo apt install ros-humble-rmw-cyclonedds-cpp
if python3 -c "import importlib; importlib.import_module('cyclonedds')" 2>/dev/null || \
   dpkg -l ros-humble-rmw-cyclonedds-cpp 2>/dev/null | grep -q "^ii"; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    echo "[DDS] Using CycloneDDS"
else
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/bluerov2_control_project/fastdds_profile.xml
    echo "[DDS] Using FastRTPS (install ros-humble-rmw-cyclonedds-cpp for best results)"
fi

source /opt/ros/humble/setup.bash
source ~/bluerov2_control_project/ros2_ws/install/setup.bash
python3 -m bluerov2_control.mission_control "$@"
