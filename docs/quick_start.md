# Quick Start Guide

Get your Basicmicro ROS2 driver running in under 5 minutes with system-wide installation - from initial setup to moving motors.

> 🔧 **For development work**: See [Development Quick Start](dev_quick_start.md) for workspace-based installation.

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

First, clone the repository:

```bash
# Clone the ROS2 Drivers repository
git clone https://github.com/basicmicro/ROS2_Drivers.git
cd ROS2_Drivers/basicmicro_driver
```

## Step 2: Install the ROS2 Driver System-Wide

Run the automated installation script - it will handle everything, including ROS2 installation if needed:

### System-Wide Installation
```bash
# Run the automated installation script for system-wide installation
./install.sh --system-wide
```

**What this does:**
- ✅ Checks for ROS2 installation (prompts to install if not found)
- ✅ Installs all Python dependencies system-wide (PyYAML, numpy, pyserial, empy, lark-parser, catkin_pkg)
- ✅ Installs Basicmicro Python library
- ✅ Installs ROS2 dependencies using rosdep
- ✅ Builds and installs the ROS2 package system-wide
- ✅ Sets up serial port permissions
- ✅ Tests hardware connection (optional)
- ✅ Makes driver available globally

**Expected output (if ROS2 already installed):**
```
=================================
  Basicmicro ROS2 Driver Installer
=================================

[INFO] Installation mode: system-wide
[INFO] Checking ROS2 installation...
[SUCCESS] ROS2 jazzy found and sourced
[INFO] Installing Python dependencies...
[SUCCESS] System Python dependencies installed
[INFO] Installing Basicmicro Python library...
[SUCCESS] Basicmicro Python library installed
[INFO] Installing ROS2 dependencies...
[SUCCESS] ROS2 dependencies installed
[INFO] Building ROS2 package...
[SUCCESS] Package built and installed system-wide
[INFO] Setting up serial port permissions...
[SUCCESS] User already in dialout group

=================================
  INSTALLATION COMPLETE!
=================================
```

**If ROS2 is not installed, you'll see:**
```
[WARNING] ROS2 not found on this system.

Would you like to install ROS2 jazzy now?
This will:
  - Install ROS2 jazzy Desktop
  - Set up development tools (colcon, rosdep)
  - Configure environment variables
  - Add ROS2 sourcing to ~/.bashrc

Install ROS2 jazzy? (y/n): y
[INFO] Installing ROS2 jazzy...
[SUCCESS] ROS2 jazzy installation complete!
```

## Step 3: Launch the Driver
```bash
# Check available serial ports first
ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* 2>/dev/null

# Launch the driver (works from any directory)
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
In a new terminal:
```bash
# Send a basic movement command (works from any directory)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Stop the motors
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Installation Script Options

### System-Wide Installation (Recommended)
```bash
./install.sh --system-wide
```
Installs driver system-wide, accessible from any directory.

### Different ROS2 Distribution
```bash
./install.sh --system-wide --ros-distro humble
```
Use ROS2 Humble instead of Jazzy.

### Development Installation
```bash
./install.sh --dev
```
For development work - see [Development Quick Start](dev_quick_start.md) for details.

## You Know It's Working When...

✅ **Controller Communication**: Version string displays correctly  
✅ **ROS2 Integration**: Node starts without errors  
✅ **Motor Response**: Motors move when sending cmd_vel commands  
✅ **Encoder Feedback**: Odometry data publishes on /odom topic  
✅ **Safety**: Emergency stop works (Ctrl+C stops motors immediately)

## Quick Troubleshooting

### Installation Script Failed
```bash
# Check the error message and run with verbose output
./install.sh
```
The script provides detailed error messages and suggests solutions.

### "No serial ports found"
```bash
# Check available ports
ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* 2>/dev/null

# For WSL users, ensure serial port is mapped
# See: docs/wsl_comport_setup.md
```

### "Permission denied: '/dev/ttyACM0'"
The installation script automatically adds you to the dialout group, but you may need to:
```bash
# Log out and log back in, or run:
sudo usermod -a -G dialout $USER
# Then restart your terminal
```

### "Controller connection failed"
- Check cable connection (USB or serial)
- Verify controller power
- Try different serial port (`/dev/ttyACM1`, `/dev/ttyAMA0`, `/dev/ttyUSB0`)
- Check controller address (default: 0x80)

### "No movement detected"
- Verify motor connections to M1/M2 terminals
- Check motor power supply
- Ensure controller is configured in Motion Studio
- Try higher cmd_vel values (up to 1.0 m/s)

## Advanced Usage

### Manual Environment Setup
With system-wide installation, no special environment setup is needed:
```bash
# Driver is available system-wide, just run:
ros2 run basicmicro_driver basicmicro_node.py --ros-args -p port:=/dev/ttyACM0
```

### Post-Installation Testing
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

### Quick Hardware Test
For a quick connection test:
```bash
# The installation script includes this test, but you can run it manually
python3 -c "
from basicmicro import Basicmicro
controller = Basicmicro('/dev/ttyACM0', 38400)  # Adjust port as needed
success = controller.Open()
if success:
    version = controller.ReadVersion(0x80)
    print(f'Controller connected: {version[1].strip()}')
    controller.close()
else:
    print('Connection failed - check port and connections')
"
```

## Success Validation Commands

Verify everything is working:
```bash
# Check all ROS2 topics are publishing
ros2 topic list

# Monitor odometry data
ros2 topic echo /odom --once

# Test emergency stop
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

## Next Steps

Once basic functionality is confirmed:

1. **Complete Setup**: Follow [Installation Guide](installation.md) for full configuration
2. **Controller Configuration**: See [Motor Controller Setup](motor_controller_setup.md) for Motion Studio configuration
3. **Try Examples**: Run `python3 rover_demo.py` or `python3 arm_demo.py`
4. **Advanced Features**: Explore [Service API](service_api.md) for advanced control

## Time Comparison

| Method | Setup Time | Complexity | Error Rate |
|--------|------------|------------|------------|
| **Installation Script** | 2-5 minutes | Low | Very Low |
| Manual Setup | 15-45 minutes | High | High |

**Estimated completion time: 2-5 minutes for system-wide installation**

---
*The installation script handles all dependency management, environment setup, and hardware testing automatically. For manual installation or advanced configuration, see the [Installation Guide](installation.md).*