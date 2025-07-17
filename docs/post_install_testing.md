# Post-Installation Testing Workflow

Complete testing guide to verify your Basicmicro ROS2 driver installation and ensure everything is working correctly.

## Overview

This guide provides a systematic approach to testing your installation, from basic connectivity to advanced features. Follow these steps in order to verify proper operation.

## Prerequisites

- ✅ Installation completed successfully (no errors)
- ✅ Motor controller connected via USB/serial
- ✅ Motors connected to M1/M2 channels
- ✅ Power supply connected and turned on
- ✅ User added to dialout group (Linux) or proper permissions set

## Testing Phases

### Phase 1: Basic Connectivity Test

#### Step 1.1: Check Serial Port
```bash
# Linux - Check for available serial ports
ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* 2>/dev/null

# Should show something like: /dev/ttyACM0 or /dev/ttyUSB0
```

#### Step 1.2: Test Driver Launch
```bash
# For development installation
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source venv/bin/activate

# For system-wide installation
# (No environment setup needed)

# Launch driver (replace /dev/ttyACM0 with your actual port)
ros2 run basicmicro_driver basicmicro_node.py --ros-args -p port:=/dev/ttyACM0
```

**Expected Output:**
```
[INFO] [basicmicro_node]: Connected to controller: USB Roboclaw 2x15a v4.4.2
[INFO] [basicmicro_node]: Encoders reset to zero
[INFO] [basicmicro_node]: Basicmicro node started
[INFO] [basicmicro_node]: Port: /dev/ttyACM0, Baud: 38400, Address: 128
```

**If you see connection errors:**
- Check serial port path
- Verify controller is powered on
- Check USB cable connection
- Ensure proper permissions (dialout group on Linux)

#### Step 1.3: Verify Topics
In a new terminal:
```bash
# Check that ROS2 topics are published
ros2 topic list

# Should show:
# /cmd_vel
# /diagnostics
# /joint_states
# /basicmicro/status
```

### Phase 2: Motor Response Test

#### Step 2.1: Basic Movement Test
```bash
# Test forward movement (1-2 second delay is normal)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Stop motors
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Test backward movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.1}, angular: {z: 0.0}}" --once

# Stop motors
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

**Expected Behavior:**
- Motors should respond to commands
- 1-2 second delay is normal for `--once` commands
- Motors should stop when commanded

#### Step 2.2: Rotation Test
```bash
# Test left turn
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}" --once

# Stop motors
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Test right turn
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.3}}" --once

# Stop motors
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

#### Step 2.3: Safety Test (CRITICAL)
```bash
# Start continuous movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10

# Press Ctrl+C in the driver terminal - motors should stop immediately
```

**Expected Behavior:**
- Motors must stop immediately when driver is terminated
- Should see message: "Shutting down... Stopping motors for safety."

### Phase 3: Sensor Data Verification

#### Step 3.1: Check Encoder Data
```bash
# Monitor encoder/joint data
ros2 topic echo /joint_states

# Should show changing position values if motors are moving
```

#### Step 3.2: Check Diagnostics
```bash
# Monitor diagnostic information
ros2 topic echo /diagnostics

# Should show battery voltage, temperature, connection status
```

#### Step 3.3: Monitor Connection Health
```bash
# Monitor connection status
ros2 topic echo /basicmicro/status

# Should show connection state updates
```

### Phase 4: Performance Testing

#### Step 4.1: Continuous Control Test
```bash
# Test continuous movement (no delays after startup)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10

# Let run for 10 seconds, then Ctrl+C
```

**Expected Behavior:**
- Initial 1-2 second delay is normal
- After startup, commands should be immediate
- No "Waiting for subscription" messages during operation

#### Step 4.2: Programmatic Control Test
```bash
# Test programmatic control example
cd /path/to/basicmicro_driver
python3 examples/programmatic_control.py

# Choose option 1 (demonstration sequence)
```

**Expected Behavior:**
- No command delays
- Smooth motor movements
- Clear status messages

### Phase 5: Advanced Feature Testing

#### Step 5.1: Launch File Test
```bash
# Test launch file
ros2 launch basicmicro_driver basicmicro_driver.launch.py port:=/dev/ttyACM0

# Should start driver with proper configuration
```

#### Step 5.2: Parameter Configuration Test
```bash
# Test custom parameters
ros2 run basicmicro_driver basicmicro_node.py --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud:=38400 \
  -p address:=128 \
  -p wheel_radius:=0.1 \
  -p wheel_separation:=0.3
```

## Troubleshooting Guide

### Common Issues and Solutions

#### "Permission denied" on serial port
```bash
# Linux - Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in
```

#### "No module named 'basicmicro'"
```bash
# Install Basicmicro Python library
cd /path/to/Basicmicro_python
pip install -e .
```

#### "Failed to open serial connection"
- Check port path: `ls /dev/ttyACM*`
- Verify controller is powered on
- Try different USB port/cable
- Check for other programs using the port

#### Motors don't respond to commands
- Verify power supply is connected and on
- Check motor connections to M1/M2 channels
- Ensure controller address matches (default: 128)
- Check for error messages in driver output

#### "Waiting for at least 1 matching subscription(s)..."
- This is normal for `--once` commands (1-2 seconds)
- For real-time control, use continuous or programmatic methods
- Check that driver is running in another terminal

#### Connection degraded/failed messages
- Check USB cable and connections
- Monitor `/diagnostics` topic for detailed error info
- Connection will auto-recover for intermittent issues
- If error count reaches 100, restart driver

## Success Criteria

### ✅ Installation Successful When:
- Driver starts without errors
- Controller version detected correctly
- All ROS2 topics are published
- Motors respond to velocity commands
- Motors stop immediately on Ctrl+C (CRITICAL SAFETY)
- Encoder data updates correctly
- Diagnostics show healthy values
- Connection monitoring reports "good" state

### ✅ Performance Criteria:
- `--once` commands: 1-2 second delay (normal)
- Continuous control: Immediate response after startup
- Programmatic control: No command delays
- Error recovery: Automatic healing for intermittent issues
- Safety shutdown: Immediate motor stop on exit

## Post-Testing Recommendations

### For Development Use:
1. Use programmatic control for applications
2. Monitor connection health during operation
3. Implement proper error handling in your code
4. Test emergency stop procedures regularly

### For Production Use:
1. Set up proper logging and monitoring
2. Implement watchdog timers for safety
3. Test all failure scenarios
4. Document your specific hardware configuration

## Next Steps

After successful testing:
1. **Configure parameters** for your specific hardware
2. **Integrate with your application** using programmatic control
3. **Set up monitoring** for connection health
4. **Document your setup** for future reference

---

**Remember**: The `--once` command delays are normal ROS2 behavior. For real-time applications, use continuous or programmatic control methods as documented in [control_methods.md](control_methods.md).

## Getting Help

If you encounter issues not covered in this guide:
1. Check the [troubleshooting](troubleshooting.md) documentation
2. Review the [control methods](control_methods.md) guide
3. Examine the connection diagnostics: `ros2 topic echo /diagnostics`
4. Check the driver logs for detailed error messages

The testing workflow is designed to catch issues early and ensure reliable operation. Take time to verify each phase before moving to the next.