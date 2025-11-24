#!/bin/bash
# Build script for SmartSteward ROS workspace

set -e

echo "======================================"
echo "Building SmartSteward ROS Workspace"
echo "======================================"

# Check if colcon is installed
if ! command -v colcon &> /dev/null; then
    echo "Error: colcon not found. Please install ROS 2 first."
    echo "Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Build the workspace
echo "Building packages..."
colcon build --symlink-install

# Check build status
if [ $? -eq 0 ]; then
    echo ""
    echo "======================================"
    echo "Build completed successfully!"
    echo "======================================"
    echo ""
    echo "To use the workspace, run:"
    echo "  source install/setup.bash"
    echo ""
    echo "To launch the system, run:"
    echo "  ros2 launch smartsteward_bringup smartsteward.launch.py"
else
    echo ""
    echo "Build failed! Please check the error messages above."
    exit 1
fi
