# Configuration Parameter Guide

This document provides a complete reference for configuring the Basicmicro ROS2 driver, including parameter organization by functionality, robot-specific configurations, tuning guidelines, and example setups.

## Parameter Organization

Parameters are organized into logical groups for easy configuration management:

- **Hardware Parameters**: Physical connection and communication settings
- **Motion Control Parameters**: Control strategy and motion limits
- **Unit Conversion Parameters**: Physical unit mappings and conversions
- **Safety Parameters**: Limits, protections, and emergency systems
- **Performance Parameters**: Timing, buffering, and optimization settings
- **Diagnostic Parameters**: Monitoring, logging, and health reporting

## Hardware Parameters

### Communication Settings

```yaml
hardware:
  # Serial Communication
  port: "/dev/ttyACM0"              # USB Serial or "/dev/ttyAMA0" for Async Serial
  baud_rate: 38400                  # Communication baud rate (9600-460800)
  address: 128                      # Controller address (128-135, 0x80-0x87)
  timeout_ms: 1000                  # Communication timeout in milliseconds
  
  # Connection Management
  auto_reconnect: true              # Automatic reconnection on failure
  max_reconnect_attempts: 5         # Maximum reconnection attempts
  reconnect_delay_ms: 1000         # Delay between reconnection attempts
  
  # Protocol Settings
  packet_validation: true           # Enable CRC packet validation
  retry_count: 3                   # Command retry attempts
  response_timeout_ms: 100         # Individual command timeout
```

**Parameter Details:**
- `port`: Serial device path (USB: `/dev/ttyACM*`, Async Serial: `/dev/ttyAMA*` or `/dev/ttyUSB*`, Windows: `COM*`)
- `baud_rate`: Higher rates reduce latency but may be less reliable over long cables
- `address`: Each controller on bus must have unique address
- `timeout_ms`: Total operation timeout including retries

### Device Identification

```yaml
hardware:
  # Device Information
  controller_type: "auto"           # "roboclaw", "mcp", "auto"
  expected_version: ""              # Expected firmware version (empty = any)
  device_name: "basicmicro_controller"  # Human-readable device name
  
  # Hardware Features
  encoder_support: true             # Controller has encoder inputs
  temperature_monitoring: true      # Temperature sensor available
  current_monitoring: true          # Current sensing available
  voltage_monitoring: true          # Voltage monitoring available
```

## Motion Control Parameters

### Motion Strategy Configuration

```yaml
motion_control:
  # Control Strategy
  strategy: "SPEED_ACCEL"           # "DUTY", "SPEED", "SPEED_ACCEL", "POSITION"
  
  # Velocity Limits
  max_velocity: 3.0                 # Maximum velocity (rad/s)
  min_velocity: -3.0                # Minimum velocity (rad/s)
  default_velocity: 1.0             # Default velocity for movements
  
  # Acceleration Limits
  max_acceleration: 2.0             # Maximum acceleration (rad/s²)
  min_acceleration: -2.0            # Minimum acceleration (rad/s²)
  default_acceleration: 1.0         # Default acceleration
  
  # Position Limits (POSITION strategy)
  position_limit_enabled: false     # Enable software position limits
  min_position: -6.28               # Minimum position (rad)
  max_position: 6.28                # Maximum position (rad)
  position_tolerance: 0.01          # Position tolerance (rad)
```

### PID Control Parameters

```yaml
motion_control:
  # PID Configuration (set via Motion Studio)
  pid_velocity_p: 65536             # Velocity P gain (0x00010000)
  pid_velocity_i: 32768             # Velocity I gain (0x00008000)
  pid_velocity_d: 16384             # Velocity D gain (0x00004000)
  
  pid_position_p: 131072            # Position P gain (0x00020000)
  pid_position_i: 65536             # Position I gain (0x00010000)
  pid_position_d: 32768             # Position D gain (0x00008000)
  
  # Control Loop Settings
  qpps: 8000                        # Quadrature pulses per second at max speed
  deadband: 2                       # Position deadband (encoder counts)
  velocity_deadband: 10             # Velocity deadband (counts/sec)
```

## Unit Conversion Parameters

### Physical System Configuration

```yaml
unit_conversion:
  # Wheel/Linear System
  wheel_radius: 0.1                 # Wheel radius (meters)
  wheel_separation: 0.5             # Distance between wheels (meters)
  track_width: 0.5                  # Alternative name for wheel_separation
  
  # Mechanical System
  gear_ratio: 30.0                  # Gear reduction ratio (motor:output)
  encoder_counts_per_rev: 1440      # Encoder counts per motor revolution
  
  # Conversion Factors
  meters_per_count: 0.000436        # Linear distance per encoder count
  radians_per_count: 0.004363       # Angular distance per encoder count
  
  # Duty Cycle Mapping
  max_duty_cycle: 0.95              # Maximum duty cycle (0.0-1.0)
  duty_cycle_deadband: 0.02         # Duty cycle deadband
```

### Robot-Specific Conversions

```yaml
# Differential Drive Robot
unit_conversion:
  wheel_radius: 0.1                 # 100mm wheels
  gear_ratio: 30.0                  # 30:1 gearbox
  encoder_counts_per_rev: 1440      # Quadrature encoder
  wheel_separation: 0.5             # 500mm wheelbase

# Robotic Arm (revolute joints)
unit_conversion:
  gear_ratio: 100.0                 # High-precision gearing
  encoder_counts_per_rev: 2048      # High-resolution encoder
  joint_offset: [0.0, 0.0]         # Joint zero offsets (rad)
  
# Linear Actuator
unit_conversion:
  lead_screw_pitch: 0.005           # 5mm per revolution
  gear_ratio: 1.0                   # Direct drive
  encoder_counts_per_rev: 1000      # Linear encoder
```

## Safety Parameters

### Protection Systems

```yaml
safety:
  # Emergency Stop
  emergency_stop_enabled: true      # Enable emergency stop functionality
  emergency_stop_deceleration: 5.0  # Emergency stop deceleration (rad/s²)
  auto_stop_on_error: true          # Auto-stop on communication errors
  
  # Velocity Protection
  velocity_limit_enabled: true      # Enable velocity limiting
  max_safe_velocity: 4.0           # Maximum safe velocity (rad/s)
  velocity_ramp_rate: 2.0          # Velocity change rate limit (rad/s²)
  
  # Position Protection
  soft_limit_enabled: false        # Enable software position limits
  soft_limit_margin: 0.1           # Margin before hard limits (rad)
  hard_limit_action: "stop"        # Action on limit: "stop", "reverse"
  
  # Thermal Protection
  temperature_limit: 80.0          # Maximum temperature (°C)
  temperature_action: "reduce_power" # Action: "stop", "reduce_power"
  
  # Current Protection
  current_limit: 15.0              # Maximum current (A)
  current_limit_duration: 1.0     # Time before current limiting (s)
```

### Collision Detection

```yaml
safety:
  # Collision Detection
  collision_detection_enabled: false # Enable collision detection
  collision_threshold: 2.0          # Current spike threshold (A)
  collision_response: "stop"        # Response: "stop", "reverse", "compliant"
  
  # Stall Detection
  stall_detection_enabled: true     # Enable stall detection
  stall_velocity_threshold: 0.1     # Minimum velocity for stall (rad/s)
  stall_current_threshold: 5.0      # Current threshold for stall (A)
  stall_time_threshold: 2.0         # Time before stall detected (s)
```

## Performance Parameters

### Control Loop Timing

```yaml
performance:
  # Update Rates
  control_frequency: 100.0          # Hardware interface update rate (Hz)
  diagnostic_frequency: 1.0         # Diagnostic publishing rate (Hz)
  status_frequency: 10.0           # Status monitoring rate (Hz)
  
  # Buffer Management
  command_buffer_size: 32          # Command buffer size (commands)
  buffer_warning_level: 0.8        # Buffer utilization warning (0.0-1.0)
  buffer_overflow_action: "drop_oldest" # Action: "drop_oldest", "reject_new"
  
  # Communication Optimization
  batch_commands: true             # Batch multiple commands
  command_timeout: 0.1             # Individual command timeout (s)
  heartbeat_interval: 1.0          # Heartbeat interval (s)
```

### Thread and Process Settings

```yaml
performance:
  # Threading
  use_separate_thread: true        # Use separate thread for hardware communication
  thread_priority: 50             # Thread priority (0-99)
  thread_affinity: -1             # CPU affinity (-1 = any)
  
  # Memory Management
  preallocate_buffers: true       # Preallocate communication buffers
  buffer_pool_size: 10            # Number of preallocated buffers
  
  # Real-time Settings
  enable_realtime: false          # Enable real-time scheduling
  realtime_priority: 80           # Real-time priority
```

## Diagnostic Parameters

### Monitoring Configuration

```yaml
diagnostics:
  # Health Monitoring
  enable_diagnostics: true         # Enable diagnostic publishing
  diagnostic_level: "info"        # Level: "debug", "info", "warn", "error"
  
  # Communication Health
  monitor_communication: true     # Monitor communication health
  comm_error_threshold: 0.05      # Error rate threshold (0.0-1.0)
  comm_timeout_threshold: 5       # Consecutive timeout threshold
  
  # Performance Monitoring
  monitor_performance: true       # Monitor performance metrics
  latency_warning_ms: 10         # Latency warning threshold (ms)
  throughput_warning: 0.8        # Throughput warning threshold
  
  # Data Logging
  log_commands: false            # Log all commands
  log_responses: false           # Log all responses
  log_errors: true              # Log error conditions
  log_file_path: "/tmp/basicmicro.log" # Log file location
```

## Robot-Specific Configurations

### Differential Drive Robot Configuration

```yaml
# rover_config.yaml
hardware:
  port: "/dev/ttyACM0"
  baud_rate: 38400
  address: 128

motion_control:
  strategy: "SPEED_ACCEL"
  max_velocity: 2.0                # 2 rad/s maximum
  max_acceleration: 1.0            # Smooth acceleration
  
unit_conversion:
  wheel_radius: 0.1                # 100mm wheels
  wheel_separation: 0.5            # 500mm wheelbase
  gear_ratio: 30.0                 # 30:1 planetary gearbox
  encoder_counts_per_rev: 1440     # Standard quadrature

safety:
  velocity_limit_enabled: true
  max_safe_velocity: 3.0
  emergency_stop_enabled: true

performance:
  control_frequency: 50.0          # 50Hz for mobile robot
  buffer_warning_level: 0.7        # Conservative buffering
```

### Two-Axis Robotic Arm Configuration

```yaml
# arm_config.yaml
hardware:
  port: "/dev/ttyAMA0"            # Async Serial for production
  baud_rate: 115200               # Higher baud for precision
  address: 128

motion_control:
  strategy: "POSITION"            # Position control for servos
  max_velocity: 1.5               # Moderate speed for precision
  max_acceleration: 0.8           # Smooth motion
  position_tolerance: 0.01        # High precision
  
unit_conversion:
  gear_ratio: 100.0               # High-precision harmonic drive
  encoder_counts_per_rev: 2048    # High-resolution encoder
  joint_offset: [0.0, 0.0]       # Calibrated joint zeros
  
safety:
  position_limit_enabled: true
  min_position: [-3.14, -1.57]   # Joint limits
  max_position: [3.14, 1.57]
  soft_limit_margin: 0.1
  
performance:
  control_frequency: 100.0        # High frequency for precision
  enable_realtime: true           # Real-time for servo control
```

### Industrial Conveyor Configuration

```yaml
# conveyor_config.yaml
hardware:
  port: "/dev/ttyUSB0"           # USB-to-Serial adapter for Async Serial
  baud_rate: 115200
  address: 128

motion_control:
  strategy: "SPEED"              # Simple speed control
  max_velocity: 5.0              # High speed operation
  default_velocity: 3.0          # Standard operating speed
  
unit_conversion:
  gear_ratio: 10.0               # Low reduction for speed
  encoder_counts_per_rev: 1000   # Industrial encoder
  belt_circumference: 1.0        # 1m belt circumference
  
safety:
  velocity_limit_enabled: true
  current_limit: 20.0            # High current for industrial load
  stall_detection_enabled: true
  
performance:
  control_frequency: 20.0        # Lower frequency for industrial
  batch_commands: true           # Optimize for throughput
```

## Configuration File Management

### File Structure

```
config/
├── robot_configs/
│   ├── rover_config.yaml         # Differential drive robot
│   ├── arm_config.yaml           # Two-axis robotic arm
│   ├── conveyor_config.yaml      # Industrial conveyor
│   └── custom_config.yaml        # User-specific configuration
├── hardware_configs/
│   ├── roboclaw_2x15a.yaml      # RoboClaw 2x15A controller
│   ├── roboclaw_2x30a.yaml      # RoboClaw 2x30A controller
│   └── mcp_advanced.yaml        # MCP Advanced controller
└── tuning_configs/
    ├── high_precision.yaml       # High-precision tuning
    ├── high_speed.yaml          # High-speed tuning
    └── robust_outdoor.yaml      # Robust outdoor operation
```

### Loading Configuration

```bash
# Use specific configuration file
ros2 launch basicmicro_driver basicmicro_driver.launch.py \
  config_file:=/path/to/rover_config.yaml

# Override specific parameters
ros2 run basicmicro_driver basicmicro_node.py \
  --ros-args -p port:=/dev/ttyACM1 -p max_velocity:=1.5
```

### Configuration Validation

```python
# Validate configuration before use
from basicmicro_driver.config_validator import validate_config

config = load_yaml_config('rover_config.yaml')
is_valid, errors = validate_config(config)

if not is_valid:
    for error in errors:
        print(f"Configuration error: {error}")
```

## Parameter Tuning Guidelines

### Initial Setup Process

1. **Hardware Verification**
   - Test basic communication
   - Verify encoder functionality
   - Check motor direction and limits

2. **Motion Studio Configuration**
   - Set appropriate PID values
   - Configure QPPS for maximum speed
   - Test motion in Motion Studio first

3. **ROS2 Parameter Tuning**
   - Start with conservative values
   - Gradually increase performance parameters
   - Monitor diagnostics during tuning

4. **Performance Optimization**
   - Adjust control frequency based on application
   - Optimize buffer settings for smooth motion
   - Fine-tune safety parameters

### Common Tuning Scenarios

**High Precision Applications:**
- Increase `control_frequency` to 100Hz+
- Reduce `position_tolerance` to 0.005 rad
- Enable `enable_realtime` for consistent timing
- Use high-resolution encoders (2048+ counts/rev)

**High Speed Applications:**
- Increase `max_velocity` gradually
- Optimize `gear_ratio` for speed vs torque
- Increase `buffer_warning_level` to 0.9
- Use higher `baud_rate` for faster communication

**Robust Outdoor Operation:**
- Enable comprehensive safety systems
- Increase `timeout_ms` for reliable communication
- Add temperature and current monitoring
- Use conservative acceleration limits

### Parameter Relationships

**Velocity and Acceleration:**
```
max_acceleration ≤ max_velocity / 2.0  # Avoid overshoot
position_tolerance ≥ velocity_tolerance * control_period
```

**Buffer and Frequency:**
```
buffer_size ≥ control_frequency * command_duration
command_timeout ≤ 1.0 / control_frequency
```

**Safety Margins:**
```
max_safe_velocity ≤ 0.8 * hardware_max_velocity
soft_limit_margin ≥ max_velocity² / (2 * max_acceleration)
```

## Troubleshooting Configuration Issues

### Common Problems

**Parameter Not Applied:**
- Check parameter namespace
- Verify parameter type (string vs int vs float)
- Restart node after parameter changes

**Communication Failures:**
- Verify `port` and `baud_rate` settings
- Check `address` matches controller configuration
- Test with lower baud rate

**Poor Motion Quality:**
- Review Motion Studio PID settings
- Adjust `control_frequency`
- Check mechanical system for binding

**Performance Issues:**
- Monitor buffer utilization
- Adjust `command_timeout`
- Optimize thread settings

For complete configuration examples and validation tools, see the [Testing Documentation](testing_overview.md) and [Hardware Validation Guide](hardware_validation.md).