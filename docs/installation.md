# Installation and Setup Guide

Complete installation instructions for the Basicmicro ROS2 driver package.

## System Requirements

### Operating System
- **Ubuntu 22.04 LTS** (recommended)
- **Ubuntu 20.04 LTS** (with ROS2 Humble)
- **Other Linux distributions** with ROS2 support

### ROS2 Distributions
- **ROS2 Jazzy** (recommended, Ubuntu 22.04)
- **ROS2 Humble** (Ubuntu 22.04/20.04)
- **ROS2 Iron** (experimental support)

### Python Requirements
- **Python 3.10+** (3.12 recommended)
- **pip** package manager
- **venv** (recommended for isolation)

### Hardware Requirements
- Basicmicro motor controller (RoboClaw or MCP series)
- USB serial or Async Serial connection (TTL/RS-232)
- Compatible motors and encoders

## Dependency Installation

### 1. ROS2 Installation

If ROS2 is not already installed:

#### Ubuntu 22.04 - ROS2 Jazzy
```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop python3-argcomplete -y

# Install additional tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y
```

#### Ubuntu 20.04/22.04 - ROS2 Humble
```bash
# Similar process but install ros-humble-desktop
sudo apt install ros-humble-desktop python3-argcomplete -y
sudo apt install python3-colcon-common-extensions python3-rosdep -y
```

### 2. Python Dependencies

#### Core Python Packages
```bash
# Install essential Python packages
pip3 install --user pyserial numpy scipy

# For development and testing
pip3 install --user pytest pytest-mock pytest-asyncio
```

#### Basicmicro Python Library
```bash
# Navigate to the Basicmicro Python library
cd /path/to/basicmicro_ros2/Basicmicro_python

# Install in development mode
pip3 install -e .

# Verify installation
python3 -c "import basicmicro; print('Basicmicro library installed successfully')"
```

### 3. ROS2 Package Dependencies

```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash  # or humble

# Install ROS2 dependencies
sudo apt install ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-nav-msgs -y
sudo apt install ros-jazzy-diagnostic-msgs ros-jazzy-std-srvs -y
sudo apt install ros-jazzy-hardware-interface ros-jazzy-controller-manager -y
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers -y

# For visualization and debugging
sudo apt install ros-jazzy-rviz2 ros-jazzy-rqt -y
```

## Package Build Process

### 1. Workspace Setup

```bash
# Create or navigate to your ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone <repository-url> basicmicro_ros2
cd ~/ros2_ws
```

### 2. Environment Configuration

```bash
# Source ROS2 distribution
source /opt/ros/jazzy/setup.bash

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update

# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Package

```bash
# Build only the basicmicro_driver package
colcon build --packages-select basicmicro_driver

# Or build entire workspace
colcon build

# Source the workspace
source install/setup.bash
```

**Expected build output:**
```
Starting >>> basicmicro_driver
Finished <<< basicmicro_driver [9.55s]

Summary: 1 package finished [9.55s]
```

### 4. Package Verification

#### Service Generation Check
```bash
# Verify service interfaces were generated
ros2 interface list | grep basicmicro_driver
```

**Expected output:** List of 15 service interfaces:
```
basicmicro_driver/srv/ExecutePositionSequence
basicmicro_driver/srv/ExecuteTrajectory
basicmicro_driver/srv/GetAvailableHomingMethods
[... 12 more services ...]
```

#### Import Test
```bash
# Test Python imports
python3 -c "
try:
    from basicmicro_driver.srv import MoveDistance
    from basicmicro_driver.hardware_interface import BasicmicroHardwareInterface
    print('✅ All imports successful')
except ImportError as e:
    print(f'❌ Import failed: {e}')
"
```

#### Package Structure Verification
```bash
# Check installed files
ls install/basicmicro_driver/lib/basicmicro_driver/
ls install/basicmicro_driver/share/basicmicro_driver/srv/
```

## Environment Configuration

### 1. Shell Environment Setup

Add to your `~/.bashrc` for automatic sourcing:

```bash
# ROS2 Environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Optional: Set default serial port (USB or Async Serial)
export BASICMICRO_DEFAULT_PORT=/dev/ttyACM0  # or /dev/ttyAMA0 for Async Serial
export BASICMICRO_DEFAULT_BAUD=38400
```

### 2. Serial Port Permissions

#### Add User to dialout Group
```bash
# Add current user to dialout group
sudo usermod -a -G dialout $USER

# Verify group membership
groups $USER
```

#### udev Rules (Optional)
Create persistent device names:

```bash
# Create udev rule file
sudo tee /etc/udev/rules.d/99-basicmicro.rules << 'EOF'
# RoboClaw controllers
SUBSYSTEM=="tty", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2404", SYMLINK+="roboclaw"
SUBSYSTEM=="tty", ATTRS{product}=="Roboclaw*", SYMLINK+="roboclaw"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. Python Path Configuration

If using virtual environments:

```bash
# Create virtual environment
python3 -m venv ~/basicmicro_venv
source ~/basicmicro_venv/bin/activate

# Install dependencies in venv
pip install pyserial numpy scipy pytest

# Install Basicmicro library
cd /path/to/Basicmicro_python
pip install -e .
```

## Validation and Testing

### 1. Connection Test

```bash
# Test serial connection
python3 -c "
import serial
try:
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    print('✅ Serial port accessible')
    ser.close()
except serial.SerialException as e:
    print(f'❌ Serial error: {e}')
except PermissionError:
    print('❌ Permission denied - check dialout group membership')
"
```

### 2. Controller Communication Test

```bash
# Test Basicmicro controller communication
python3 -c "
from basicmicro import Basicmicro
controller = Basicmicro('/dev/ttyACM0', 38400)
success = controller.Open()
if success:
    version = controller.ReadVersion(0x80)
    if version[0]:
        print(f'✅ Controller: {version[1].strip()}')
    else:
        print('❌ Version read failed')
    controller.close()
else:
    print('❌ Controller connection failed')
"
```

### 3. ROS2 Package Test

```bash
# Test package functionality
cd ~/ros2_ws
source install/setup.bash

# Run unit tests
python3 -m pytest src/basicmicro_ros2/basicmicro_driver/test/unit/ -v

# Quick node startup test
timeout 5s ros2 run basicmicro_driver basicmicro_node.py --ros-args -p port:=/dev/ttyACM0 || echo "Node startup test complete"
```

## Troubleshooting Installation Issues

### Build Failures

#### Missing Dependencies
```bash
# Update package lists
sudo apt update

# Install missing ROS2 packages
sudo apt install ros-jazzy-* --fix-missing

# Reinstall Python dependencies
pip3 install --user --upgrade pyserial numpy scipy
```

#### Colcon Build Errors
```bash
# Clean build directory
rm -rf build install log

# Build with verbose output
colcon build --packages-select basicmicro_driver --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Check for specific error messages in log files
cat log/latest_build/basicmicro_driver/stderr.log
```

### Import Errors

#### Service Import Issues
```bash
# Ensure workspace is sourced
source install/setup.bash

# Check interface generation
ls install/basicmicro_driver/share/basicmicro_driver/srv/

# Regenerate interfaces if needed
colcon build --packages-select basicmicro_driver --cmake-clean-cache
```

#### Python Module Issues
```bash
# Check PYTHONPATH
echo $PYTHONPATH

# Manual path addition (temporary)
export PYTHONPATH=$PYTHONPATH:$(pwd)/install/basicmicro_driver/lib/python3.12/site-packages
```

### Hardware Connection Issues

#### Device Not Found
```bash
# List all USB devices
lsusb

# List serial devices with details
ls -la /dev/tty*

# Check dmesg for USB events
dmesg | grep -i usb | tail -10
```

#### Permission Problems
```bash
# Check current permissions
ls -la /dev/ttyACM0

# Temporary permission fix
sudo chmod 666 /dev/ttyACM0

# Permanent fix: add to dialout group (requires logout/login)
sudo usermod -a -G dialout $USER
```

## Next Steps

After successful installation:

1. **Controller Configuration**: See [Motor Controller Setup Guide](motor_controller_setup.md)
2. **WSL Users**: Follow [WSL Comport Setup Guide](wsl_comport_setup.md)
3. **Quick Start**: Try the [System-Wide Quick Start Guide](system_wide_quick_start.md) for immediate functionality
4. **Examples**: Run demonstration scripts (`rover_demo.py`, `arm_demo.py`)

## Installation Verification Checklist

- [ ] ROS2 distribution installed and sourced
- [ ] Python dependencies installed (pyserial, numpy, scipy)
- [ ] Basicmicro Python library installed
- [ ] ROS2 workspace built successfully
- [ ] Service interfaces generated (15 services visible)
- [ ] Serial port permissions configured
- [ ] Controller communication test passed
- [ ] Package imports working correctly

**Installation typically takes 15-30 minutes on a clean system.**

---
*For Windows WSL users, see the [WSL Comport Setup Guide](wsl_comport_setup.md) for additional configuration steps.*