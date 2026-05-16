#!/bin/bash
# ──────────────────────────────────────────────────────────────────
#  BlueROV2 Workspace Setup Script
#  Automates dependency checks, ROS 2 build, and environment setup.
# ──────────────────────────────────────────────────────────────────
set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}🚀 Starting BlueROV2 Workspace Setup...${NC}"

# 1. Check ROS 2 Environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS 2 not detected! Please source /opt/ros/humble/setup.bash first.${NC}"
    exit 1
fi
echo -e "${GREEN}✅ ROS 2 $ROS_DISTRO detected.${NC}"

# 2. Install System Dependencies
echo -e "${BLUE}📦 Installing system dependencies...${NC}"
sudo apt update
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-pip

# 3. Install Python Dependencies
echo -e "${BLUE}🐍 Installing Python packages...${NC}"
pip3 install -r requirements.txt

# 4. Build Workspace
echo -e "${BLUE}🛠️ Building ROS 2 workspace...${NC}"
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. Finalize
echo -e "${GREEN}✨ Setup Complete!${NC}"
echo -e "${BLUE}To start simulation, run:${NC}"
echo -e "  source ros2_ws/install/setup.bash"
echo -e "  ./launch.sh"
