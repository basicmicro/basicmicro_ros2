#!/bin/bash

# ROS2 Uninstaller Script
# This script completely removes ROS2 from the system

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

echo -e "${RED}================================="
echo -e "  ROS2 Uninstaller"
echo -e "=================================${NC}"
echo ""

# Warning message
print_warning "This will completely remove ROS2 from your system!"
echo ""
echo -e "${YELLOW}This will:${NC}"
echo -e "  - Remove all ROS2 packages and dependencies"
echo -e "  - Remove ROS2 repository and GPG keys"
echo -e "  - Remove rosdep configuration"
echo -e "  - Remove ROS2 sourcing from ~/.bashrc"
echo -e "  - Clean up all ROS2 directories"
echo ""
read -p "Are you sure you want to proceed? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    print_error "Uninstall cancelled by user"
    exit 1
fi

echo ""
print_status "Starting ROS2 uninstallation..."

# Step 1: Remove ROS2 packages
print_status "Removing ROS2 packages..."
sudo apt remove --purge -y ros-jazzy-* python3-rosdep python3-colcon-common-extensions 2>/dev/null || true
print_success "ROS2 packages removed"

# Step 2: Remove ROS2 repository and keys
print_status "Removing ROS2 repository and GPG keys..."
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
print_success "ROS2 repository and keys removed"

# Step 3: Remove rosdep configuration
print_status "Removing rosdep configuration..."
sudo rm -rf /etc/ros/ 2>/dev/null || true
rm -rf ~/.ros/rosdep/ 2>/dev/null || true
print_success "rosdep configuration removed"

# Step 4: Remove ROS2 sourcing from .bashrc
print_status "Removing ROS2 sourcing from ~/.bashrc..."
if grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    sed -i '/source \/opt\/ros\/jazzy\/setup.bash/d' ~/.bashrc
    print_success "ROS2 sourcing removed from ~/.bashrc"
else
    print_status "No ROS2 sourcing found in ~/.bashrc"
fi

# Step 5: Clean up apt cache
print_status "Cleaning up apt cache..."
sudo apt update
sudo apt autoremove -y
sudo apt autoclean
print_success "APT cache cleaned"

# Step 6: Remove any remaining ROS2 directories
print_status "Removing remaining ROS2 directories..."
sudo rm -rf /opt/ros/jazzy 2>/dev/null || true
sudo rm -rf /opt/ros/humble 2>/dev/null || true
sudo rm -rf /opt/ros/iron 2>/dev/null || true
print_success "ROS2 directories removed"

# Step 7: Clean up user workspace
print_status "Cleaning up user workspace..."
rm -rf ~/ros2_ws 2>/dev/null || true
print_success "User workspace cleaned"

# Step 8: Verify uninstallation
print_status "Verifying uninstallation..."
if [ -d "/opt/ros/jazzy" ]; then
    print_warning "Some ROS2 files may still exist in /opt/ros/jazzy"
else
    print_success "ROS2 installation directory removed"
fi

if command -v ros2 >/dev/null 2>&1; then
    print_warning "ros2 command still available - may need to restart terminal"
else
    print_success "ros2 command no longer available"
fi

echo ""
echo -e "${GREEN}================================="
echo -e "  UNINSTALL COMPLETE!"
echo -e "=================================${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo -e "1. Close and reopen your terminal to ensure environment is clean"
echo -e "2. Run 'ros2 --version' to verify ROS2 is completely removed"
echo -e "3. You can now test fresh ROS2 installation with:"
echo -e "   ${BLUE}./install.sh --system-wide${NC}"
echo ""