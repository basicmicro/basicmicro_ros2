# Motor Controller Setup Guide

Essential configuration steps using Motion Studio before ROS2 integration. **Critical: Complete this setup before using the ROS2 driver.**

## Why Motion Studio First?

The Basicmicro ROS2 driver assumes your motor controller is **properly configured**. Motion Studio provides:
- **Motor parameter tuning** (critical for safe operation)
- **PID configuration** (required for accurate speed/position control)
- **Safety limits** (prevents damage to motors and hardware)
- **Encoder setup** (essential for odometry and feedback)

## Motion Studio Installation

### Download and Install
1. Visit: https://basicmicro.com/downloads
2. Download "Motion Studio" for your operating system
3. Install with administrative privileges
4. Connect your Basicmicro controller via USB

### Initial Connection
1. Launch Motion Studio
2. Select your controller's COM port
3. Click "Connect"
4. Verify controller version and firmware

## Configuration by Application Type

### Differential Drive Rover Configuration

Perfect for mobile robots, rovers, and ground vehicles.

#### 1. Motor Configuration
```
Motor 1 (M1): Left wheel motor
Motor 2 (M2): Right wheel motor

Motor Settings:
- Motor Type: Brushed DC
- Max Current: [Based on your motors - typically 5-15A]
- Max Voltage: [Match your power supply - 12V/24V typical]
- Acceleration: 2000 counts/sec² (conservative start)
- Deceleration: 2000 counts/sec²
```

#### 2. Encoder Setup
```
Encoder 1: Left wheel encoder
Encoder 2: Right wheel encoder

Encoder Settings:
- Encoder Type: Quadrature
- PPR (Pulses Per Revolution): [Check encoder specs - 360/500/1000 typical]
- Velocity Calculation: Automatic
- Direction: Normal (adjust if wheel goes wrong direction)
```

#### 3. PID Tuning (Critical)
```
Velocity PID (for each motor):
- P (Proportional): 65536 (start value)
- I (Integral): 32768 (start value)  
- D (Derivative): 0 (usually not needed)
- Max Speed: 8000 counts/sec (conservative)

Position PID (if using position control):
- P: 1800
- I: 0
- D: 2000
- Position Tolerance: 100 counts
```

#### 4. Safety Limits
```
Current Limits:
- Motor 1 Limit: [80% of motor rating]
- Motor 2 Limit: [80% of motor rating]

Speed Limits:
- Max Forward Speed: 10000 counts/sec
- Max Reverse Speed: -10000 counts/sec

Voltage Limits:
- Low Voltage Cutoff: [Battery minimum - 10.5V for 12V system]
- High Voltage Cutoff: [15V for 12V system, 30V for 24V system]
```

#### 5. Differential Drive Parameters
```
Wheel Configuration:
- Wheel Separation: [Distance between wheel centers in meters]
- Wheel Radius: [Wheel radius in meters]
- Gear Ratio: [If using gearbox - reduction ratio]

Example for common rover:
- Wheel Separation: 0.35m (35cm between wheels)
- Wheel Radius: 0.075m (7.5cm radius)
- Gear Ratio: 19:1 (if using 19:1 gearbox)
```

### Servo Positioning Arm Configuration

For robotic arms, pan-tilt systems, and precise positioning.

#### 1. Motor Configuration
```
Motor 1 (M1): Base rotation or Joint 1
Motor 2 (M2): Arm elevation or Joint 2

Servo Motor Settings:
- Motor Type: Brushed DC with Encoder
- Max Current: [Lower than DC motors - 2-10A typical]
- Max Voltage: [Match power supply]
- Acceleration: 500 counts/sec² (slower for precision)
- Deceleration: 500 counts/sec²
```

#### 2. Encoder Configuration
```
High-Resolution Encoders:
- Encoder Type: Quadrature
- PPR: 500-2000 (higher for precision)
- Index Channel: Enable if available
- Encoder Direction: Test and adjust

Position Feedback:
- Enable Position Mode
- Set Home Position: 0 counts
- Enable Soft Limits
```

#### 3. Position PID Tuning
```
Position PID (critical for servo control):
- P: 1800 (start value)
- I: 0 (add slowly if needed)
- D: 2000 (damping)
- Position Tolerance: 50 counts (tight tolerance)
- Max Speed in Position Mode: 3000 counts/sec
```

#### 4. Soft Limits (Important for Safety)
```
Joint 1 (Base) Limits:
- Minimum Position: -90000 counts (-90 degrees typical)
- Maximum Position: +90000 counts (+90 degrees typical)

Joint 2 (Elevation) Limits:
- Minimum Position: 0 counts (horizontal)
- Maximum Position: +45000 counts (+45 degrees typical)

Emergency Limits:
- Hard stops at ±95% of soft limits
- Current spike detection for collision
```

#### 5. Homing Configuration
```
Homing Method:
- Home Method: Current Spike (safest)
- Home Speed: 500 counts/sec (slow)
- Home Current Threshold: 150% of normal
- Timeout: 30 seconds
- Home Offset: 0 counts (adjust after first home)
```

### Multi-Motor Industrial Setup

For conveyor systems, multi-axis machines, and industrial automation.

#### 1. Synchronized Motor Configuration
```
Motor 1: Primary drive motor
Motor 2: Secondary/follower motor

Synchronization Settings:
- Enable Master-Slave Mode
- Master: Motor 1
- Slave: Motor 2
- Sync Tolerance: 100 counts
- Sync Recovery Speed: 1000 counts/sec
```

#### 2. Industrial PID Parameters
```
High-Performance PID:
- P: 131072 (higher gain for stiffness)
- I: 65536 (faster integral action)
- D: 0 (avoid in noisy environments)
- Feedforward: 32768 (improves tracking)
```

#### 3. Fault Detection
```
Advanced Monitoring:
- Enable Overcurrent Protection
- Enable Position Error Monitoring
- Enable Communication Timeout
- Enable Temperature Monitoring (if available)

Fault Response:
- Action on Fault: Stop and Hold
- Recovery Mode: Manual Reset Required
- Diagnostic Logging: Enable
```

## Configuration Validation

### Test Each Motor Independently

#### 1. Manual Motor Test
```
In Motion Studio:
1. Select Motor 1
2. Set speed to 500 counts/sec
3. Click "Forward" - motor should turn smoothly
4. Click "Stop" - motor should stop immediately
5. Click "Reverse" - motor should turn opposite direction
6. Repeat for Motor 2
```

#### 2. Encoder Feedback Test
```
Check Encoder Readings:
1. Turn motor shaft manually
2. Watch encoder count change
3. Verify direction matches motor direction
4. Ensure no missed counts or glitches
```

#### 3. PID Response Test
```
Speed Control Test:
1. Set target speed: 1000 counts/sec
2. Enable speed control
3. Motor should reach speed smoothly (2-3 seconds)
4. Speed should be stable (±50 counts/sec)
5. Response should not oscillate
```

### Common Configuration Mistakes

#### Wrong Encoder Direction
**Symptom:** Motor runs away when speed control enabled
**Fix:** In Motion Studio, reverse encoder direction for affected motor

#### PID Too Aggressive
**Symptom:** Motor oscillates or makes noise
**Fix:** Reduce P gain by 50%, increase D gain slightly

#### Current Limit Too Low
**Symptom:** Motor stops under load
**Fix:** Increase current limit gradually, monitor motor temperature

#### Wrong Gear Ratio
**Symptom:** ROS2 odometry doesn't match actual movement
**Fix:** Verify and correct gear ratio in Motion Studio and ROS2 parameters

## Motion Studio to ROS2 Parameter Mapping

### Critical Parameters to Record

```
From Motion Studio → ROS2 Parameter File:

Motor Configuration:
- Max Speed (counts/sec) → max_speed_counts_per_sec
- Acceleration → max_acceleration  
- Current Limit → current_limit_amps

Encoder Configuration:
- PPR → encoder_counts_per_rev
- Gear Ratio → gear_ratio

PID Values:
- Velocity P → velocity_p_gain
- Velocity I → velocity_i_gain  
- Velocity D → velocity_d_gain
- Position P → position_p_gain

Physical Parameters:
- Wheel Separation → wheel_separation_m
- Wheel Radius → wheel_radius_m
```

### Create ROS2 Parameter File

After Motion Studio configuration, create `config/my_robot_parameters.yaml`:

```yaml
basicmicro_driver:
  ros__parameters:
    # Hardware Configuration
    port: "/dev/ttyACM0"
    baud: 38400
    address: 128  # 0x80
    
    # Motor Parameters (from Motion Studio)
    max_speed_counts_per_sec: 8000
    max_acceleration: 2000
    current_limit_amps: 10.0
    
    # Encoder Parameters  
    encoder_counts_per_rev: 500
    gear_ratio: 19.0
    
    # Physical Robot Parameters
    wheel_separation_m: 0.35
    wheel_radius_m: 0.075
    
    # Control Parameters
    control_frequency: 50.0
    diagnostic_frequency: 1.0
```

## Final Validation Steps

### 1. Motion Studio Final Check
- [ ] Motors respond correctly to manual commands
- [ ] Encoders count properly in both directions
- [ ] PID control is stable and responsive
- [ ] Safety limits are properly configured
- [ ] No error messages or warnings

### 2. Save Configuration
```
In Motion Studio:
1. File → Save Configuration
2. Save as: "my_robot_config.rbc"
3. Note configuration location for future reference
```

### 3. Test with ROS2 Driver
```bash
# Launch with your parameters
ros2 run basicmicro_driver basicmicro_node.py --ros-args \
  -p port:=/dev/ttyACM0 \
  --params-file config/my_robot_parameters.yaml

# Test basic movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

## Troubleshooting Motion Studio

### Connection Issues
- **Check USB cable** and Windows Device Manager
- **Try different COM port** or baud rate
- **Update controller firmware** if available
- **Restart Motion Studio** with administrator privileges

### Configuration Not Saving
- **Run Motion Studio as Administrator**
- **Check controller memory status**
- **Verify firmware supports configuration saving**

### Erratic Motor Behavior
- **Reduce PID gains** and increase gradually
- **Check power supply stability**
- **Verify encoder wiring** and connections
- **Test with different acceleration values**

## Configuration Backup

Always backup your working configuration:

```
Motion Studio Files to Save:
- Configuration file (.rbc)
- PID tuning screenshots
- Parameter value list
- Test results documentation
```

**Proper Motion Studio configuration is essential for reliable ROS2 operation. Take time to tune parameters correctly.**

---
*Next: [Quick Start Guide](quick_start.md) to test your configured controller with ROS2.*