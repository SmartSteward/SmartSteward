#!/bin/bash
# Setup script for SmartSteward ROS environment

echo "======================================"
echo "SmartSteward ROS Environment Setup"
echo "======================================"

# Detect ROS 2 installation
if [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_DISTRO="humble"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    ROS_DISTRO="iron"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_DISTRO="jazzy"
else
    echo "Error: No ROS 2 installation found!"
    echo "Please install ROS 2 first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "Found ROS 2 $ROS_DISTRO"

# Source ROS 2
echo "Sourcing ROS 2 $ROS_DISTRO..."
source /opt/ros/$ROS_DISTRO/setup.bash

# Check if workspace is built
if [ -f "install/setup.bash" ]; then
    echo "Sourcing workspace..."
    source install/setup.bash
    echo ""
    echo "Environment setup complete!"
    echo "You can now run: ros2 launch smartsteward_bringup smartsteward.launch.py"
else
    echo ""
    echo "Workspace not built yet. Please run: ./build.sh"
fi

echo "======================================"
