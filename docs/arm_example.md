# Arm Example - Two-Axis Servo Positioning

This guide demonstrates using the Basicmicro ROS2 driver for precise servo positioning with a two-axis robotic arm. You'll learn servo configuration, homing procedures, position control, and safety systems for accurate positioning applications.

## Prerequisites

Before starting this example, ensure you have completed:

✅ **Hardware Setup**: Motion Studio configured for servo positioning ([motor_controller_setup.md](motor_controller_setup.md))
✅ **ROS2 Installation**: Package built and environment sourced ([installation.md](installation.md))
✅ **Hardware Connection**: Controller connected and communication verified ([system_wide_quick_start.md](system_wide_quick_start.md))

### Required Hardware Configuration

**Motion Studio Servo Configuration Checklist:**
- [ ] Motor 1: Base rotation servo (shoulder joint)
- [ ] Motor 2: Elbow servo (elbow joint)
- [ ] Position PID tuning completed for both servos
- [ ] Homing switches configured and tested
- [ ] Position limits set for safe operation range
- [ ] Encoder resolution configured (typically 3200 counts/rev)

### Servo Hardware Requirements

**Recommended Servo Setup:**
- **Base Servo**: High-torque servo for shoulder rotation (±180°)
- **Elbow Servo**: Medium-torque servo for elbow movement (±90°)
- **Encoders**: Quadrature encoders for position feedback
- **Limit Switches**: Home position sensors for calibration
- **Emergency Stop**: Hardware e-stop button for safety

### Verify Servo Communication

```bash
# Test servo communication and positioning
python3 -c "
from basicmicro import Basicmicro
controller = Basicmicro('/dev/ttyACM0', 38400)  # Adjust port
if controller.Open():
    print('Controller connected')
    # Read current positions
    enc1 = controller.ReadEncM1(0x80)
    enc2 = controller.ReadEncM2(0x80)
    print(f'Base position: {enc1[1]} counts' if enc1[0] else 'Base read failed')
    print(f'Elbow position: {enc2[1]} counts' if enc2[0] else 'Elbow read failed')
    controller.close()
else:
    print('Connection failed')
"
```

**Expected Output:**
```
Controller connected
Base position: 1205 counts
Elbow position: -847 counts
```

## Step-by-Step Arm Setup

### Step 1: Build and Source Workspace

```bash
# Navigate to workspace
cd ~/ros2_ws

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build and source
colcon build --packages-select basicmicro_driver
source install/setup.bash
```

### Step 2: Create Arm Configuration

```bash
# Create arm-specific configuration
cat > config/arm_config.yaml << 'EOF'
arm_node:
  ros__parameters:
    # Hardware connection
    port: "/dev/ttyACM0"
    baud_rate: 38400
    address: 128  # 0x80
    
    # Servo configuration
    motion_strategy: "POSITION"
    max_position_error: 50  # counts
    position_hold_threshold: 10  # counts
    
    # Joint parameters - Base (Motor 1)
    base_joint:
      encoder_counts_per_rev: 3200
      gear_ratio: 50.0           # 50:1 gearbox
      position_limits:
        min: -9000               # ~-180° with gearing
        max: 9000                # ~+180° with gearing
      max_velocity: 1000         # counts/sec
      max_acceleration: 500      # counts/sec²
      home_offset: 0             # Encoder counts at home position
      
    # Joint parameters - Elbow (Motor 2) 
    elbow_joint:
      encoder_counts_per_rev: 3200
      gear_ratio: 30.0           # 30:1 gearbox
      position_limits:
        min: -4500               # ~-90° with gearing
        max: 4500                # ~+90° with gearing  
      max_velocity: 800          # counts/sec
      max_acceleration: 400      # counts/sec²
      home_offset: 0             # Encoder counts at home position
    
    # Safety parameters
    emergency_stop_enabled: true
    position_tolerance: 20       # counts
    velocity_timeout: 2.0        # seconds
    
    # Control parameters
    controller_frequency: 50.0   # Hz for precise positioning
    use_sim_time: false
EOF
```

### Step 3: Launch Arm Control Node

```bash
# Launch the arm positioning node
ros2 run basicmicro_driver basicmicro_node.py --ros-args --params-file config/arm_config.yaml
```

**Expected Output:**
```
[INFO] [arm_node]: BasicMicro ROS2 Driver starting...
[INFO] [arm_node]: Connected to controller at /dev/ttyACM0
[INFO] [arm_node]: Controller: USB Roboclaw 2x15a v4.4.2
[INFO] [arm_node]: Motion strategy: POSITION
[INFO] [arm_node]: Base joint configured: ±180° range, 50:1 gearing
[INFO] [arm_node]: Elbow joint configured: ±90° range, 30:1 gearing
[INFO] [arm_node]: Arm control ready - position control active
[WARN] [arm_node]: Arm not homed - perform homing before operation
```

## Homing Procedures

### Step 4: Perform System Homing

**⚠️ Safety First**: Ensure clear workspace and emergency stop accessible.

```bash
# Open new terminal for service calls
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Home the base joint (Motor 1)
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 1, method: 'home_forward', speed: 200, acceleration: 100}"
```

**Expected Homing Sequence:**
1. **Slow Movement**: Servo moves slowly toward home switch
2. **Switch Contact**: Movement stops when home switch activates
3. **Precise Return**: Small reverse movement for precise positioning
4. **Position Zero**: Encoder position set to home_offset value
5. **Status Update**: Homing complete, position control enabled

```bash
# Home the elbow joint (Motor 2)
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 2, method: 'home_reverse', speed: 150, acceleration: 100}"
```

**Homing Validation:**
```bash
# Check homing status
ros2 service call /motion_config basicmicro_driver/srv/MotionConfig \
  "{action: 'get_status'}"

# Expected response:
# success: true
# message: "Both joints homed and ready"
# base_homed: true
# elbow_homed: true
```

### Step 5: Test Basic Positioning

**Move to Safe Test Position:**

```bash
# Move base to center position (0°)
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 1, method: 'absolute_position', position: 0, speed: 500, acceleration: 300}"

# Move elbow to neutral position (0°)
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 2, method: 'absolute_position', position: 0, speed: 400, acceleration: 200}"
```

**Position Validation:**
```bash
# Monitor joint positions
ros2 topic echo /joint_states --field position

# Expected output (approximately):
# [0.0, 0.0]  # Both joints at center positions
```

## Arm Movement Demonstrations

### Basic Position Control

```bash
# Demonstration sequence - basic positions
echo "=== Basic Arm Positioning Demo ==="

# Position 1: Base 45°, Elbow 30°
echo "Moving to Position 1..."
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 1, method: 'absolute_position', position: 2250, speed: 600, acceleration: 400}"
sleep 2
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 2, method: 'absolute_position', position: 1350, speed: 500, acceleration: 300}"
sleep 3

# Position 2: Base -60°, Elbow -45°  
echo "Moving to Position 2..."
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 1, method: 'absolute_position', position: -3000, speed: 600, acceleration: 400}"
sleep 2
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 2, method: 'absolute_position', position: -2025, speed: 500, acceleration: 300}"
sleep 3

# Return to center
echo "Returning to center..."
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 1, method: 'absolute_position', position: 0, speed: 800, acceleration: 500}"
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 2, method: 'absolute_position', position: 0, speed: 600, acceleration: 400}"
```

### Coordinated Multi-Joint Movement

```bash
# Trajectory example - coordinated movement
ros2 service call /trajectory_execution basicmicro_driver/srv/TrajectoryExecution \
  "{points: [
    {time: 2.0, motor1_position: 1500, motor2_position: 900},
    {time: 4.0, motor1_position: 3000, motor2_position: 1800},
    {time: 6.0, motor1_position: 1500, motor2_position: 2700},
    {time: 8.0, motor1_position: 0, motor2_position: 0}
  ], speed: 800, acceleration: 400}"
```

**Expected Trajectory Behavior:**
- **Smooth Interpolation**: Both joints move simultaneously
- **Coordinated Timing**: Joints reach waypoints synchronously
- **Acceleration Limiting**: No jerky movements or overshooting
- **Position Accuracy**: ±20 counts final position accuracy

### Using the Arm Demo Script

```bash
# Run comprehensive arm demonstration
python3 arm_demo.py
```

**Demo Sequence:**
1. **Connection Test**: Verify controller communication
2. **Homing Sequence**: Automatic homing of both joints
3. **Range of Motion**: Test full workspace limits
4. **Precision Test**: Small incremental movements
5. **Speed Variation**: Fast and slow movement demonstration
6. **Trajectory Execution**: Coordinated multi-joint paths
7. **Emergency Stop**: Safety system validation

**Expected Demo Output:**
```
=== Basicmicro Arm Demonstration ===
✓ Hardware connection established
✓ Controller: USB Roboclaw 2x15a v4.4.2

Phase 1: Homing sequence
✓ Base joint homed at position: 0
✓ Elbow joint homed at position: 0

Phase 2: Range of motion test
✓ Base range: -9000 to +9000 counts (±180°)
✓ Elbow range: -4500 to +4500 counts (±90°)

Phase 3: Precision positioning test
✓ Target: 1000 counts, Actual: 998 counts (±2 counts accuracy)
✓ Target: 2500 counts, Actual: 2503 counts (±3 counts accuracy)

Phase 4: Speed control test
✓ Slow movement: 200 counts/sec - smooth operation
✓ Fast movement: 1000 counts/sec - stable control

Phase 5: Trajectory execution
✓ 4-point coordinated trajectory completed
✓ Maximum position error: 8 counts

Phase 6: Emergency stop test
✓ Immediate stop from 800 counts/sec: 0.15s response
✓ All tests completed successfully!
```

## Advanced Arm Control

### Inverse Kinematics Service

```bash
# Move end-effector to Cartesian position
ros2 service call /arm_kinematics basicmicro_driver/srv/ArmKinematics \
  "{target_x: 0.3, target_y: 0.2, target_z: 0.15, movement_speed: 600}"
```

**Kinematics Calculation:**
- **Input**: End-effector position (x, y, z) in meters
- **Output**: Joint angles (base, elbow) in encoder counts
- **Validation**: Workspace limits and reachability checking
- **Safety**: Collision detection and joint limit enforcement

### Position Hold and Release

```bash
# Enable position hold (servo actively maintains position)
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 1, method: 'position_hold_enable'}"

# Release position hold (servo can be moved manually)
ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
  "{motor: 1, method: 'position_hold_release'}"
```

**Position Hold Features:**
- **Active Correction**: Servo resists external forces
- **Power Efficiency**: Reduced current when on-target
- **Manual Override**: Emergency manual movement capability
- **Status Monitoring**: Position hold state reporting

## Safety Systems

### Emergency Stop Procedures

```bash
# Immediate emergency stop
ros2 service call /emergency_stop std_srvs/srv/Trigger "{}"

# Reset after emergency stop
ros2 service call /reset_emergency_stop std_srvs/srv/Trigger "{}"
```

**Emergency Stop Behavior:**
1. **Immediate Deceleration**: Maximum safe deceleration applied
2. **Position Hold**: Servos maintain current position
3. **System Lock**: No new commands accepted until reset
4. **Status Indication**: Emergency state reported in diagnostics

### Position Limit Enforcement

The arm automatically enforces configured position limits:

```yaml
# Limits are enforced in software and hardware
position_limits:
  base_min: -9000    # Software limit
  base_max: 9000     # Software limit
  elbow_min: -4500   # Software limit  
  elbow_max: 4500    # Software limit
```

**Limit Behavior:**
- **Soft Limits**: Gradual deceleration when approaching limits
- **Hard Limits**: Immediate stop if limits exceeded
- **Status Reporting**: Limit violations logged and reported
- **Recovery**: Automatic return to safe position

## Performance Monitoring

### Position Accuracy Validation

```bash
# Test positioning accuracy across workspace
for position in -4000 -2000 0 2000 4000; do
  echo "Testing position: $position"
  ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
    "{motor: 1, method: 'absolute_position', position: $position, speed: 600, acceleration: 400}"
  sleep 3
  ros2 topic echo /joint_states --field position -1 | head -1
done
```

**Accuracy Benchmarks:**
- **Position Accuracy**: ±10 counts (±0.1°) for most positions
- **Repeatability**: ±5 counts for repeated moves to same position
- **Settling Time**: < 1 second for 90° movements
- **Overshoot**: < 2% of movement distance

### Real-Time Monitoring

```bash
# Monitor arm performance continuously
ros2 topic echo /diagnostics --field status &
ros2 topic echo /joint_states --field effort &
ros2 topic echo /arm_status
```

**Key Performance Indicators:**
- **Position Error**: Current vs target position difference
- **Motor Current**: Servo loading and torque requirements
- **Velocity Profile**: Acceleration/deceleration smoothness  
- **Temperature**: Motor and controller thermal status

## Troubleshooting

### Positioning Issues

**Servo Not Reaching Target Position:**

1. **Check Position Limits**
   ```bash
   # Verify target is within configured limits
   ros2 param get /arm_node base_joint.position_limits
   ```

2. **Verify Homing Status**
   ```bash
   # Confirm both joints are properly homed
   ros2 service call /motion_config basicmicro_driver/srv/MotionConfig \
     "{action: 'get_status'}"
   ```

3. **PID Tuning Check**
   - Verify Motion Studio PID parameters
   - Test with slower speeds if overshooting
   - Increase position tolerance if small errors acceptable

### Homing Problems

**Homing Switch Not Detected:**

1. **Hardware Verification**
   ```bash
   # Test switch directly in Motion Studio
   # Verify switch wiring and polarity
   ```

2. **Homing Direction**
   ```bash
   # Try opposite homing direction
   ros2 service call /servo_position_control basicmicro_driver/srv/ServoPosition \
     "{motor: 1, method: 'home_reverse', speed: 150, acceleration: 100}"
   ```

3. **Switch Configuration**
   - Check Motion Studio limit switch settings
   - Verify normally-open vs normally-closed configuration
   - Test switch activation manually

### Communication Errors

**Service Call Failures:**

1. **Node Status Check**
   ```bash
   ros2 node list | grep arm
   ros2 service list | grep servo
   ```

2. **Parameter Verification**
   ```bash
   ros2 param list /arm_node
   ros2 param get /arm_node port
   ```

3. **Hardware Connection**
   ```bash
   # Test direct hardware communication
   python3 -c "
   from basicmicro import Basicmicro
   controller = Basicmicro('/dev/ttyACM0', 38400)
   print('Connected:', controller.Open())
   "
   ```

## Integration Examples

### MoveIt! Integration

The arm is ready for MoveIt! motion planning integration:

```bash
# Install MoveIt! packages
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-planners

# The arm provides standard interfaces:
# - joint_states (sensor_msgs/JointState)
# - position commands via services
# - URDF description for planning
```

**MoveIt! Configuration Steps:**
1. **URDF Description**: Robot model with joint limits and geometry
2. **SRDF Configuration**: Planning groups and collision geometry
3. **Kinematics Plugin**: KDL or TRAC-IK solver integration
4. **Planning Interface**: RViz integration for interactive planning

### Multi-Arm Coordination

```yaml
# Dual-arm configuration example
arm_left:
  ros__parameters:
    port: "/dev/ttyACM0"
    # ... arm configuration

arm_right:
  ros__parameters:
    port: "/dev/ttyACM1"  
    # ... arm configuration
```

## Success Criteria

**✅ You know the arm example is working when:**

1. **Homing Complete**: Both joints home successfully and report ready status
2. **Precise Positioning**: ±10 count accuracy for position commands
3. **Smooth Movement**: No jerky motion or oscillations during moves
4. **Safety Systems**: Emergency stop responds within 200ms
5. **Coordinated Motion**: Multi-joint trajectories execute smoothly
6. **Position Hold**: Servos maintain position against external forces

**📊 Performance Benchmarks:**
- **Position Accuracy**: ±0.1° (±10 encoder counts)
- **Repeatability**: ±0.05° for repeated positioning
- **Movement Speed**: Up to 1000 counts/sec stable operation
- **Response Time**: < 100ms from command to movement start
- **Safety Response**: < 200ms emergency stop activation

The arm is now ready for precision positioning applications, pick-and-place operations, assembly tasks, and integration with advanced motion planning systems like MoveIt!