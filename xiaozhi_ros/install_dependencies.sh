#!/bin/bash
# Install dependencies for SmartSteward ROS workspace

set -e

echo "======================================"
echo "Installing SmartSteward Dependencies"
echo "======================================"

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install ROS 2 build tools if not present
echo "Installing ROS 2 build tools..."
sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Install Python dependencies
echo "Installing Python dependencies..."
sudo apt-get install -y \
    python3-pip \
    python3-serial

pip3 install --user pyserial

# Initialize rosdep if not already done
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo "Initializing rosdep..."
    sudo rosdep init
fi

echo "Updating rosdep..."
rosdep update

# Install package dependencies
echo "Installing ROS package dependencies..."
cd "$(dirname "$0")"
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "======================================"
echo "Dependencies installed successfully!"
echo "======================================"
echo ""
echo "Next steps:"
echo "1. Build the workspace: ./build.sh"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Launch the system: ros2 launch smartsteward_bringup smartsteward.launch.py"
