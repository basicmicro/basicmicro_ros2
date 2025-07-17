# Hardware Interface Reference

The Basicmicro ROS2 driver provides a comprehensive hardware interface that integrates seamlessly with the ROS2 ecosystem through ros2_control. This document covers the hardware interface implementation, motion strategies, lifecycle management, and configuration.

## Overview

The `BasicmicroHardwareInterface` class implements the ROS2 `hardware_interface::SystemInterface` standard, providing:

- **ros2_control Integration**: Full compatibility with the ROS2 control framework
- **Multiple Motion Strategies**: Support for different control modes (DUTY, SPEED, SPEED_ACCEL, POSITION)
- **Lifecycle Management**: Proper initialization, activation, and deactivation sequences
- **Real-time Performance**: 100Hz control loop with minimal latency
- **Safety Systems**: Emergency stop, limit enforcement, and error recovery

## Motion Strategies

The hardware interface supports multiple motion strategies, each optimized for different use cases:

### DUTY Strategy
Direct PWM duty cycle control for open-loop operation.

**Use Cases:**
- Testing motor connectivity
- Manual motor control
- Applications requiring direct power control
- Emergency stop sequences

**Parameters:**
- `duty_m1`: Motor 1 duty cycle (-32767 to +32767)
- `duty_m2`: Motor 2 duty cycle (-32767 to +32767)

**Hardware Commands:**
- `DutyM1M2()`: Direct duty cycle control
- `DutyAccelM1M2()`: Duty cycle with acceleration limits

### SPEED Strategy
Closed-loop velocity control using encoder feedback.

**Use Cases:**
- Differential drive robots
- Conveyor systems
- Applications requiring consistent velocity

**Parameters:**
- `speed_m1`: Motor 1 velocity (counts/second)
- `speed_m2`: Motor 2 velocity (counts/second)

**Hardware Commands:**
- `SpeedM1M2()`: Velocity control
- Emergency stop automatically sets speed to zero

### SPEED_ACCEL Strategy
Velocity control with acceleration/deceleration limits.

**Use Cases:**
- Smooth trajectory execution
- Applications requiring controlled acceleration
- Precision motion control

**Parameters:**
- `speed_m1`: Motor 1 target velocity (counts/second)
- `speed_m2`: Motor 2 target velocity (counts/second)
- `accel_m1`: Motor 1 acceleration limit (counts/second²)
- `accel_m2`: Motor 2 acceleration limit (counts/second²)

**Hardware Commands:**
- `SpeedAccelM1M2()`: Velocity control with acceleration limits

### POSITION Strategy
Absolute position control with encoder feedback.

**Use Cases:**
- Robotic arms
- Servo positioning
- Multi-axis CNC applications
- Precision positioning systems

**Parameters:**
- `position_m1`: Motor 1 target position (encoder counts)
- `position_m2`: Motor 2 target position (encoder counts)
- `speed_m1`: Motor 1 maximum velocity (counts/second)
- `speed_m2`: Motor 2 maximum velocity (counts/second)
- `accel_m1`: Motor 1 acceleration (counts/second²)
- `accel_m2`: Motor 2 acceleration (counts/second²)

**Hardware Commands:**
- `SpeedAccelDistanceM1M2()`: Position control with velocity and acceleration limits

## Lifecycle Management

The hardware interface follows the ROS2 lifecycle standard with proper state transitions:

### Initialization (`on_init`)
- Parameter loading and validation
- Hardware connection establishment
- Motion strategy configuration
- Unit converter initialization

```yaml
# Required parameters
port: "/dev/ttyACM0"              # USB Serial or "/dev/ttyAMA0" for Async Serial
baud_rate: 38400                  # Communication baud rate
address: 128                      # Controller address (0x80)
motion_strategy: "SPEED_ACCEL"    # Motion control strategy
```

### Configuration (`on_configure`)
- Hardware interface parameter extraction
- Controller communication verification
- Initial state reading (encoders, speeds, diagnostics)
- Safety system initialization

**Configuration Validation:**
- Port accessibility check
- Controller version verification
- Encoder functionality test
- Buffer status validation

### Activation (`on_activate`)
- Real-time control loop initialization
- Hardware interface activation
- Motor state synchronization
- Command interface enablement

**Activation Sequence:**
1. Read current motor positions and velocities
2. Initialize command interfaces with current state
3. Enable hardware command execution
4. Start diagnostic monitoring

### Deactivation (`on_deactivate`)
- Safe motor stop (duty cycle to zero)
- Command interface disabling
- Hardware interface deactivation
- Resource cleanup

**Deactivation Safety:**
1. Emergency stop all motors
2. Wait for complete motor stop
3. Disable command processing
4. Preserve encoder positions

## Parameter Configuration

### Communication Parameters
```yaml
hardware:
  port: "/dev/ttyACM0"        # USB Serial or "/dev/ttyAMA0" (Async Serial)
  baud_rate: 38400           # Communication speed
  address: 128               # Controller address (0x80-0x87)
  timeout: 1000              # Communication timeout (ms)
```

#### Communication Interfaces

**Async Serial - TTL/RS-232 (Recommended for Production)**
- Superior noise immunity for industrial environments
- Reliable long-distance communication
- Connect via `/dev/ttyAMA*`, `/dev/ttyUSB*` (USB-to-Serial adapter), or built-in serial ports
- Recommended for permanent installations and harsh environments

**USB Serial (Setup and Testing)**
- Direct USB connection via `/dev/ttyACM*`
- Convenient for initial setup and development
- Good for bench testing and controller configuration
- Not recommended for production due to noise susceptibility

Both interfaces use the same Packet Serial protocol with full command set and CRC validation. The ROS2 driver works identically with either communication method.

### Motion Control Parameters
```yaml
motion:
  strategy: "SPEED_ACCEL"    # Control strategy
  max_speed: 3000           # Maximum velocity (counts/sec)
  max_accel: 1000           # Maximum acceleration (counts/sec²)
  position_tolerance: 10     # Position tolerance (counts)
```

### Safety Parameters
```yaml
safety:
  emergency_stop_enabled: true     # Enable emergency stop
  velocity_limit: 5000            # Maximum safe velocity
  position_limit_enabled: false   # Enable position limits
  min_position: -1000000          # Minimum position (counts)
  max_position: 1000000           # Maximum position (counts)
```

### Unit Conversion Parameters
```yaml
unit_conversion:
  wheel_radius: 0.05              # Wheel radius (meters)
  gear_ratio: 1.0                 # Gear reduction ratio
  encoder_counts_per_rev: 1440    # Encoder resolution
```

## ROS2 Control Integration

### Joint Configuration
```xml
<ros2_control name="BasicmicroSystem" type="system">
  <hardware>
    <plugin>basicmicro_driver/BasicmicroHardwareInterface</plugin>
    <param name="port">/dev/ttyACM0</param>
    <param name="baud_rate">38400</param>
    <param name="address">128</param>
    <param name="motion_strategy">SPEED_ACCEL</param>
  </hardware>
  
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Controller Manager Integration
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # 100Hz control loop
    
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController
      
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

### Available Interfaces

**Command Interfaces:**
- `velocity`: Joint velocity commands (rad/s)
- `position`: Joint position commands (rad) [POSITION strategy only]
- `effort`: Joint effort commands (Nm) [DUTY strategy only]

**State Interfaces:**
- `position`: Current joint position (rad)
- `velocity`: Current joint velocity (rad/s)
- `effort`: Current joint effort (Nm)

## Error Handling and Recovery

### Communication Errors
- **Timeout Handling**: Automatic retry with exponential backoff
- **Connection Loss**: Safe motor stop and connection re-establishment
- **CRC Errors**: Command verification and retransmission

### Hardware Errors
- **Encoder Failure**: Switch to open-loop mode with user notification
- **Motor Overload**: Automatic current limiting and thermal protection
- **Power Supply Issues**: Voltage monitoring and safe shutdown

### Safety Systems
- **Emergency Stop**: Immediate motor shutdown with position preservation
- **Velocity Limiting**: Software velocity limits with hardware enforcement
- **Position Limits**: Configurable soft limits with hard stop capability

## Performance Characteristics

### Control Loop Performance
- **Update Rate**: 100Hz (10ms cycle time)
- **Command Latency**: <5ms from ROS2 command to motor response
- **Position Accuracy**: ±1 encoder count
- **Velocity Regulation**: ±5% at steady state

### Communication Performance
- **Packet Success Rate**: >99.9% under normal conditions
- **Throughput**: Up to 500 commands/second
- **Buffer Utilization**: <80% for optimal performance
- **Recovery Time**: <100ms after communication failure

## Tuning Guidelines

### PID Controller Tuning
Motion Studio configuration for optimal performance:

**For Differential Drive Robots:**
- P Gain: 0x00010000 (65536)
- I Gain: 0x00008000 (32768)
- D Gain: 0x00004000 (16384)
- QPPS: Set to 80% of measured maximum speed

**For Servo Positioning:**
- P Gain: 0x00020000 (131072)
- I Gain: 0x00010000 (65536)
- D Gain: 0x00008000 (32768)
- Position Deadband: 1-5 encoder counts

### Performance Optimization
- **Control Frequency**: 100Hz for smooth operation
- **Buffer Management**: Maintain <80% buffer utilization
- **Communication**: Use highest reliable baud rate (115200 recommended)
- **Error Recovery**: Enable automatic retry with reasonable timeouts

## Example Configurations

### Differential Drive Robot
```yaml
hardware_interface:
  port: "/dev/ttyACM0"
  baud_rate: 38400
  address: 128
  motion_strategy: "SPEED_ACCEL"
  max_speed: 3000
  max_accel: 1000
  wheel_radius: 0.1
  gear_ratio: 30.0
  encoder_counts_per_rev: 1440
```

### Two-Axis Robotic Arm
```yaml
hardware_interface:
  port: "/dev/ttyACM0"
  baud_rate: 38400
  address: 128
  motion_strategy: "POSITION"
  max_speed: 1500
  max_accel: 500
  position_tolerance: 5
  gear_ratio: 100.0
  encoder_counts_per_rev: 2048
```

### Industrial Conveyor
```yaml
hardware_interface:
  port: "/dev/ttyACM0"
  baud_rate: 115200
  address: 128
  motion_strategy: "SPEED"
  max_speed: 5000
  safety_velocity_limit: 4000
  encoder_counts_per_rev: 1000
```

## Troubleshooting

### Common Issues

**Hardware Interface Fails to Start:**
- Check serial port permissions (`sudo usermod -a -G dialout $USER`)
- Verify controller is powered and connected
- Test communication: `python3 -c "from basicmicro import Basicmicro; controller = Basicmicro('/dev/ttyACM0', 38400); print('Connected:', controller.Open())"`

**Position Drift:**
- Verify encoder connections and quality
- Check for electrical noise in encoder signals
- Tune PID parameters in Motion Studio

**Velocity Oscillation:**
- Reduce PID D gain
- Increase velocity deadband
- Check mechanical backlash and compliance

**Communication Timeouts:**
- Lower baud rate for long cable runs
- Check USB cable quality and length
- Verify proper grounding

### Diagnostic Commands

**Test Hardware Communication:**
```bash
ros2 run basicmicro_driver test_communication.py --port /dev/ttyACM0
```

**Monitor Real-time Performance:**
```bash
ros2 topic echo /diagnostics | grep basicmicro
```

**Check Control Loop Timing:**
```bash
ros2 topic hz /joint_states
```

For additional troubleshooting, see the [Testing Documentation](testing_overview.md) and [Hardware Validation Guide](hardware_validation.md).