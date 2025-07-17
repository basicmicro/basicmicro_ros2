# Rover Example - Complete Walkthrough

This guide provides a complete walkthrough of using the Basicmicro ROS2 driver with a differential drive rover. You'll learn to set up hardware, configure the system, and control a rover through both command-line interfaces and the provided demonstration script.

## Prerequisites

Before starting this example, ensure you have completed:

✅ **Hardware Setup**: Motion Studio configured for differential drive ([motor_controller_setup.md](motor_controller_setup.md))
✅ **ROS2 Installation**: Package built and environment sourced ([installation.md](installation.md))
✅ **Hardware Connection**: Controller connected and recognized by system ([quick_start.md](quick_start.md))

### Required Hardware Configuration

**Motion Studio Configuration Checklist:**
- [ ] Motor 1: Left wheel (forward = positive encoder counts)
- [ ] Motor 2: Right wheel (forward = positive encoder counts)
- [ ] PID tuning completed for both motors
- [ ] Encoder directions verified (forward movement = positive counts)
- [ ] Speed limits set to safe values (typically 3000-5000 counts/sec)

### Verify Hardware Connection

```bash
# Test basic communication
python3 -c "
from basicmicro import Basicmicro
controller = Basicmicro('/dev/ttyACM0', 38400)  # Adjust port as needed
print('Connected:', controller.Open())
if controller.Open():
    version = controller.ReadVersion(0x80)
    print(f'Controller: {version[1].strip()}' if version[0] else 'Read failed')
    controller.close()
"
```

**Expected Output:**
```
Connected: True
Controller: USB Roboclaw 2x15a v4.4.2
```

## Step-by-Step Rover Setup

### Step 1: Build and Source Workspace

```bash
# Navigate to workspace
cd "/mnt/c/Users/acidtech/Documents/Claude-Code/ROS2 Drivers"

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash  # Or your ROS2 distribution

# Build the package
colcon build --packages-select basicmicro_driver

# Source the workspace
source install/setup.bash
```

**You know it worked when:**
- Build completes without errors
- `ros2 pkg list | grep basicmicro` shows the package

### Step 2: Configure Rover Parameters

Create a rover configuration file:

```bash
# Create config directory if it doesn't exist
mkdir -p config

# Create rover-specific configuration
cat > config/rover_config.yaml << 'EOF'
rover_node:
  ros__parameters:
    # Hardware connection
    port: "/dev/ttyACM0"
    baud_rate: 38400
    address: 128  # 0x80
    
    # Differential drive parameters
    wheel_separation: 0.335  # Distance between wheels (meters)
    wheel_radius: 0.0508     # Wheel radius (meters) - 4 inch wheels
    encoder_counts_per_rev: 3200  # Encoder resolution
    
    # Motion limits
    max_linear_velocity: 1.0   # m/s
    max_angular_velocity: 2.0  # rad/s
    
    # Control parameters
    controller_frequency: 20.0  # Hz
    use_sim_time: false
    
    # Hardware interface
    motion_strategy: "SPEED_ACCEL"
    max_acceleration: 1000  # counts/sec²
EOF
```

### Step 3: Launch the Rover Node

```bash
# Launch the main rover node with configuration
ros2 run basicmicro_driver basicmicro_node.py --ros-args --params-file config/rover_config.yaml
```

**Expected Output:**
```
[INFO] [rover_node]: BasicMicro ROS2 Driver starting...
[INFO] [rover_node]: Connected to controller at /dev/ttyACM0
[INFO] [rover_node]: Controller: USB Roboclaw 2x15a v4.4.2
[INFO] [rover_node]: Motion strategy: SPEED_ACCEL
[INFO] [rover_node]: Differential drive configured: wheel_sep=0.335m, wheel_radius=0.0508m
[INFO] [rover_node]: Rover node ready - listening for /cmd_vel commands
```

### Step 4: Test Basic Movement

Open a new terminal and test rover movement:

```bash
# Source the environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Test forward movement (0.2 m/s for 2 seconds)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Stop the rover
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

**Validation Steps:**
1. **Forward Movement**: Rover should move forward smoothly
2. **Motor Response**: Both wheels should turn at same speed
3. **Stop Command**: Rover should stop immediately when zero velocity sent

### Step 5: Test Turning

```bash
# Test rotation (0.5 rad/s clockwise)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'

# Stop rotation
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

**Expected Behavior:**
- **Clockwise Turn**: Right wheel forward, left wheel backward
- **Smooth Rotation**: No jerky movements or wheel slip
- **Immediate Stop**: Motors stop when commanded

### Step 6: Monitor Rover Status

In another terminal, monitor rover feedback:

```bash
# Monitor odometry data
ros2 topic echo /odom --field data

# Monitor joint states
ros2 topic echo /joint_states

# Check diagnostic information
ros2 topic echo /diagnostics
```

**Key Information to Observe:**
- **Odometry**: Position (x, y) and orientation (theta) updates
- **Joint States**: Left and right wheel positions and velocities
- **Diagnostics**: Motor current, voltage, temperature status

## Complete Rover Demonstration

### Using the Rover Demo Script

The package includes a complete demonstration script:

```bash
# Run the comprehensive rover demonstration
python3 rover_demo.py
```

**Demo Sequence:**
1. **Hardware Validation**: Tests controller connection and communication
2. **Forward Movement**: 1-meter forward at 0.3 m/s
3. **Backward Movement**: 1-meter backward at 0.3 m/s  
4. **Rotation Test**: 360-degree rotation at 0.5 rad/s
5. **Figure-8 Pattern**: Complex trajectory combining linear and angular motion
6. **Emergency Stop Test**: Immediate stop from full speed

**Expected Demo Output:**
```
=== Basicmicro Rover Demonstration ===
✓ Hardware connection established
✓ Controller: USB Roboclaw 2x15a v4.4.2
✓ Starting movement sequence...

Phase 1: Forward movement (1m at 0.3 m/s)
- Current position: x=0.95m, y=0.02m (95% accuracy)
✓ Forward movement completed

Phase 2: Backward movement (1m at 0.3 m/s)  
- Returned to start: x=0.03m, y=0.01m
✓ Return movement completed

Phase 3: Rotation test (360° at 0.5 rad/s)
- Final heading: 6.25° (98% accuracy)
✓ Rotation test completed

Phase 4: Figure-8 pattern
✓ Complex trajectory completed

Phase 5: Emergency stop test
✓ Emergency stop: 0.8m → 0.0m in 0.2s
✓ All tests completed successfully!
```

### Manual Control with Keyboard

For interactive control, use the keyboard teleop package:

```bash
# Install teleop package if not available
sudo apt install ros-jazzy-teleop-twist-keyboard

# Start keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

**Control Keys:**
- **i**: Forward
- **,**: Backward  
- **j**: Turn left
- **l**: Turn right
- **k**: Stop
- **q/z**: Increase/decrease max speeds

## Performance Validation

### Measuring Rover Performance

```bash
# Monitor command response time
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' &
  
# Record timing data
ros2 topic echo /odom --field twist.twist.linear.x | head -10
```

**Performance Expectations:**
- **Command Latency**: < 50ms from cmd_vel to motor response
- **Speed Accuracy**: ± 5% of commanded velocity
- **Position Accuracy**: ± 2cm for 1-meter movements
- **Smooth Operation**: No jerky movements or oscillations

### Diagnostic Monitoring

```bash
# Check motor health during operation
ros2 topic echo /diagnostics --field status
```

**Healthy Status Indicators:**
- **Motor Current**: < 80% of rated current
- **Battery Voltage**: > 11.0V for 12V systems
- **Temperature**: < 60°C internal temperature
- **Communication**: > 95% success rate

## Troubleshooting Common Issues

### Rover Not Moving

**Possible Causes & Solutions:**

1. **No cmd_vel messages received**
   ```bash
   # Check if messages are being published
   ros2 topic hz /cmd_vel
   ```

2. **Hardware connection lost**
   ```bash
   # Test direct hardware communication
   python3 -c "
   from basicmicro import Basicmicro
   controller = Basicmicro('/dev/ttyACM0', 38400)
   print('Connected:', controller.Open())
   "
   ```

3. **Motion strategy not compatible**
   - Check Motion Studio configuration
   - Verify PID tuning is complete
   - Ensure SPEED_ACCEL mode is supported

### Incorrect Movement Direction

**Direction Troubleshooting:**

1. **Encoder directions wrong**
   - Check Motion Studio encoder setup
   - Verify positive movement = positive encoder counts
   - Swap encoder wires if necessary

2. **Motor wiring issues**  
   - Verify Motor 1 = left wheel, Motor 2 = right wheel
   - Check that forward command moves rover forward
   - Reverse motor connections in Motion Studio if needed

3. **Wheel separation parameter**
   - Verify `wheel_separation` matches physical robot
   - Test: angular velocity should produce expected turning radius

### Poor Odometry Accuracy

**Odometry Troubleshooting:**

1. **Wheel parameter verification**
   ```bash
   # Measure actual wheel diameter and update config
   # wheel_radius = (wheel_diameter_meters) / 2
   ```

2. **Encoder resolution check**
   ```bash
   # Verify encoder_counts_per_rev matches encoder specs
   # Common values: 3200 (with 4x encoding), 1600, 4096
   ```

3. **Calibration procedure**
   - Drive rover exactly 1 meter forward
   - Compare odometry reading to actual distance
   - Adjust wheel_radius proportionally

### Performance Issues

**Optimization Steps:**

1. **Reduce controller frequency if lag occurs**
   ```yaml
   controller_frequency: 10.0  # Reduce from 20.0 Hz
   ```

2. **Adjust motion limits for smoother operation**
   ```yaml
   max_acceleration: 500  # Reduce from 1000
   max_linear_velocity: 0.5  # Reduce from 1.0
   ```

3. **Check system resources**
   ```bash
   # Monitor CPU usage during operation
   top -p $(pgrep -f basicmicro_node)
   ```

## Integration with Navigation Stack

### Basic Navigation Setup

The rover is ready for integration with ROS2 navigation:

```bash
# Install navigation packages
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# The rover provides standard interfaces:
# - /cmd_vel subscription (geometry_msgs/Twist)
# - /odom publication (nav_msgs/Odometry)  
# - /tf transforms (map → odom → base_link)
```

**Next Steps for Autonomous Navigation:**
1. **SLAM Setup**: Configure SLAM Toolbox or Cartographer
2. **Sensor Integration**: Add LIDAR, cameras, or other sensors
3. **Map Creation**: Build environment maps for navigation
4. **Path Planning**: Configure nav2 for autonomous navigation

## Advanced Rover Configuration

### Multi-Robot Setup

```yaml
# Robot 1 namespace
robot_1:
  ros__parameters:
    port: "/dev/ttyACM0"
    # ... other parameters

# Robot 2 namespace  
robot_2:
  ros__parameters:
    port: "/dev/ttyACM1"
    # ... other parameters
```

### Custom Motion Strategies

```yaml
# For different rover types
motion_strategy: "DUTY"        # Open-loop control
motion_strategy: "SPEED"       # Velocity control only
motion_strategy: "SPEED_ACCEL" # Velocity with acceleration limits
motion_strategy: "POSITION"    # Position control (requires homing)
```

## Success Criteria

**✅ You know the rover example is working when:**

1. **Basic Movement**: Forward/backward/rotation commands work smoothly
2. **Accurate Odometry**: Position tracking within 2% error for short distances
3. **Responsive Control**: < 50ms latency from cmd_vel to motor response
4. **Stable Operation**: No oscillations, smooth acceleration/deceleration
5. **Integration Ready**: Standard ROS2 interfaces working for navigation stack

**📊 Performance Benchmarks:**
- **Movement Accuracy**: 95%+ for 1-meter straight-line movements
- **Rotation Accuracy**: 98%+ for 360-degree rotations
- **Speed Control**: ±5% of commanded velocities
- **Communication Reliability**: >99% success rate with controller

The rover is now ready for advanced applications including autonomous navigation, multi-robot coordination, and complex trajectory execution.