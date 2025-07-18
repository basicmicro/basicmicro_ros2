# Virtual Environment Quick Start Guide

Get your Basicmicro ROS2 driver running for development work - workspace-based installation with virtual environment for iterative development and testing.

> 📦 **For production use**: See [System-Wide Quick Start Guide](system_wide_quick_start.md) for system-wide installation.

## Prerequisites Checklist

Before starting, ensure you have:

- [ ] **Ubuntu 22.04** (recommended) or compatible Linux distribution
- [ ] **Python 3.10+** with pip
- [ ] **Basicmicro motor controller** (RoboClaw or MCP series)
- [ ] **USB or Async Serial connection** to your motor controller
- [ ] **Administrative access** for serial port permissions

### Hardware Requirements
- Basicmicro controller (tested with USB Roboclaw 2x15a v4.4.2)
- Motors connected to M1/M2 channels
- Encoders connected (recommended for feedback)
- Power supply appropriate for your motors

## Step 1: Get the Project

You have **two flexible options** for workspace setup:

### Option A: Clone Directly to Workspace (Recommended)
```bash
# Create a ROS2 workspace (default or custom location)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the ROS2 Basicmicro driver repository
git clone https://github.com/basicmicro/basicmicro_ros2.git
cd basicmicro_ros2
```

### Option B: Clone Anywhere, Let Script Copy
```bash
# Clone to any location you prefer
git clone https://github.com/basicmicro/basicmicro_ros2.git ~/Downloads/basicmicro_ros2
cd ~/Downloads/basicmicro_ros2

# Script will copy to workspace during installation
```

### Custom Workspace Locations
```bash
# Use a custom workspace location
mkdir -p ~/my_robotics_projects/workspace/src
cd ~/my_robotics_projects/workspace/src
git clone https://github.com/basicmicro/basicmicro_ros2.git
cd basicmicro_ros2

# Or clone anywhere and specify workspace during install:
# ./install.sh --dev --workspace ~/my_robotics_projects/workspace
```

> 💡 **Performance Tip**: Option A (clone directly to workspace) is more efficient as it avoids copying. Option B gives you flexibility to test from any location.

## Step 2: Install the ROS2 Driver for Development

Run the automated installation script - it will handle everything, including ROS2 installation if needed:

### Development Installation

```bash
# Default installation (uses ~/ros2_ws)
./install.sh --dev

# Custom workspace installation
./install.sh --dev --workspace ~/my_robotics_projects/workspace

# Without virtual environment (uses system Python)
./install.sh --dev --no-venv
```

**What this does:**
- ✅ Checks for ROS2 installation (prompts to install if not found)
- ✅ Creates Python virtual environment with system packages access
- ✅ Installs all Python dependencies (PyYAML, numpy, pyserial, empy, lark-parser, catkin_pkg)
- ✅ Installs Basicmicro Python library via pip (pip install basicmicro)
- ✅ Installs ROS2 dependencies using rosdep
- ✅ Builds the ROS2 package in workspace
- ✅ Sets up serial port permissions
- ✅ Tests hardware connection (optional)
- ✅ Creates launch script for easy startup

**Expected output:**
```
=================================
  Basicmicro ROS2 Driver Installer
=================================

[INFO] Installation mode: development
[SUCCESS] ROS2 jazzy already sourced
[SUCCESS] Workspace already exists: /home/user/ros2_ws
[SUCCESS] Virtual environment created with system packages access
[INFO] Installing Python dependencies...
[INFO] Installing Basicmicro Python library...
[SUCCESS] Python dependencies and Basicmicro library installed
[SUCCESS] ROS2 dependencies installed
[SUCCESS] Package built successfully
[SUCCESS] Launch script created: /home/user/ros2_ws/launch_basicmicro.sh

=================================
  INSTALLATION COMPLETE!
=================================
```

## Step 3: Launch the Driver

### Using the Launch Script
```bash
# Check available serial ports first
ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* 2>/dev/null

# Use the generated launch script
~/ros2_ws/launch_basicmicro.sh /dev/ttyACM0
```

### Manual Launch
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source venv/bin/activate
ros2 run basicmicro_driver basicmicro_node.py --ros-args -p port:=/dev/ttyACM0
```

**Note**: Replace `/dev/ttyACM0` with your actual serial port (e.g., `/dev/ttyACM2`).

**Expected output:**
```
[INFO] [basicmicro_driver]: Controller connected: USB Roboclaw 2x15a v4.4.2
[INFO] [basicmicro_driver]: Publishing diagnostics on /diagnostics
[INFO] [basicmicro_driver]: Subscribing to /cmd_vel
[INFO] [basicmicro_driver]: Publishing odometry on /odom
```

## Step 4: Test Motor Movement

In a new terminal, set up the environment and test:

```bash
# Source the development environment
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source venv/bin/activate

# Send a basic movement command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Stop the motors
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Development Workflow

### Building After Code Changes
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
colcon build --packages-select basicmicro_driver
source install/setup.bash
```

### Running Tests
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source venv/bin/activate
colcon test --packages-select basicmicro_driver
```

### Debugging and Development
```bash
# Launch with debugging output
ros2 run basicmicro_driver basicmicro_node.py --ros-args -p port:=/dev/ttyACM0 --ros-args --log-level debug

# Monitor topics
ros2 topic echo /diagnostics
ros2 topic echo /odom

# Check node info
ros2 node info /basicmicro_node
```

## Installation Script Options

### Workspace Location Options

| Command | Workspace Location | Virtual Environment | Use Case |
|---------|-------------------|---------------------|----------|
| `./install.sh --dev` | `~/ros2_ws` | Yes | Standard development |
| `./install.sh --dev --workspace ~/custom_ws` | `~/custom_ws` | Yes | Custom location |
| `./install.sh --dev --no-venv` | `~/ros2_ws` | No | System Python only |

### Advanced Options

**Custom Workspace:**
```bash
./install.sh --dev --workspace /path/to/your/workspace
```
Install to any directory you prefer. Creates `src/`, `venv/`, `build/`, `install/` in specified location.

**Different ROS2 Distribution:**
```bash
./install.sh --dev --ros-distro humble
```
Use ROS2 Humble instead of Jazzy.

**No Virtual Environment:**
```bash
./install.sh --dev --no-venv
```
Uses system Python instead of virtual environment (not recommended for development).

## You Know It's Working When...

✅ **Controller Communication**: Version string displays correctly  
✅ **ROS2 Integration**: Node starts without errors  
✅ **Motor Response**: Motors move when sending cmd_vel commands  
✅ **Encoder Feedback**: Odometry data publishes on /odom topic  
✅ **Safety**: Emergency stop works (Ctrl+C stops motors immediately)  
✅ **Development**: Can rebuild and test code changes easily

## Post-Installation Testing

For comprehensive testing and verification, follow the complete testing workflow:

📋 **[Post-Installation Testing Guide](post_install_testing.md)**

This systematic testing guide covers:
- ✅ Basic connectivity verification
- ✅ Motor response testing  
- ✅ Safety system verification (CRITICAL)
- ✅ Sensor data validation
- ✅ Performance testing
- ✅ Advanced features
- ✅ Troubleshooting guide

**Important**: The post-installation testing workflow is designed to catch issues early and ensure reliable operation for development work.

## Quick Troubleshooting

### "Virtual Environment Issues"
```bash
# Remove and recreate virtual environment
cd ~/ros2_ws
rm -rf venv
./install.sh --dev
```

### "Build Failures"
```bash
# Clean build and rebuild
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select basicmicro_driver
```

### "Import Errors"
```bash
# Ensure virtual environment is activated
cd ~/ros2_ws
source venv/bin/activate
python -c "import rclpy; print('ROS2 Python OK')"
```

## Workspace Location Summary

| Setup Approach | Clone Location | Install Command | Final Package Location |
|----------------|----------------|-----------------|----------------------|
| **Standard** | `~/ros2_ws/src/` | `./install.sh --dev` | `~/ros2_ws/src/basicmicro_ros2/` |
| **Custom Direct** | `~/custom_ws/src/` | `./install.sh --dev --workspace ~/custom_ws` | `~/custom_ws/src/basicmicro_ros2/` |
| **Clone Anywhere** | Any directory | `./install.sh --dev --workspace ~/target_ws` | `~/target_ws/src/basicmicro_driver/` (copied) |

## Development vs Production

| Feature | Virtual Environment Mode | System-Wide Mode |
|---------|---------------------------|------------------|
| **Installation Location** | Custom or `~/ros2_ws/` | `/opt/ros/jazzy/` |
| **Clone Location** | Flexible | Any directory |
| **Environment Setup** | Required each session | Automatic |
| **Code Changes** | Easy rebuild | Requires reinstall |
| **Testing** | Isolated workspace | System-wide impact |
| **Debugging** | Full access | Limited access |

## Next Steps

Once development setup is confirmed:

1. **Code Changes**: Edit files in `~/ros2_ws/src/ROS2 Drivers/basicmicro_driver/`
2. **Build & Test**: Use `colcon build` and `colcon test`
3. **Hardware Testing**: Use real hardware or simulation
4. **Production Deploy**: Use `./install.sh --system-wide` when ready

## Time Comparison

| Method | Setup Time | Complexity | Flexibility |
|--------|------------|------------|-------------|
| **Development Install** | 5-10 minutes | Medium | High |
| **System Install** | 2-5 minutes | Low | Low |
| Manual Setup | 15-45 minutes | High | High |

**Estimated completion time: 5-10 minutes for development setup**

---
*The virtual environment installation provides full flexibility for code changes and testing. For production deployment, switch to system-wide installation using the [System-Wide Quick Start Guide](system_wide_quick_start.md).*