#!/bin/bash

# ROS2 Jazzy Installation Script for Ubuntu 24.04 LTS
# This script sets up a complete ROS2 test environment

set -e  # Exit on any error

echo "Starting ROS2 Jazzy installation..."

# Step 1: Install prerequisites and add ROS2 repository
echo "Installing prerequisites..."
sudo apt update
sudo apt install -y software-properties-common curl

echo "Adding ROS2 GPG key..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "Adding ROS2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Step 2: Install ROS2 Jazzy
echo "Installing ROS2 Jazzy desktop..."
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-rosdep python3-colcon-common-extensions

# Step 3: Initialize rosdep
echo "Initializing rosdep..."
sudo rosdep init
rosdep update

# Step 4: Set up environment
echo "Setting up ROS2 environment..."
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "Added ROS2 source to ~/.bashrc"
fi

# Source for current session
source /opt/ros/jazzy/setup.bash

# Step 5: Create workspace
echo "Creating ROS2 workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash

echo ""
echo "✅ ROS2 Jazzy installation complete!"
echo ""
echo "To test your installation, run these commands in separate terminals:"
echo "Terminal 1: ros2 run demo_nodes_cpp talker"
echo "Terminal 2: ros2 run demo_nodes_py listener"
echo ""
echo "Remember to source your workspace in new terminals:"
echo "source ~/ros2_ws/install/setup.bash"