# Basicmicro ROS2 Driver

A comprehensive ROS2 driver for Basicmicro motor controllers (RoboClaw and MCP series) using the ros2_control framework. This driver provides high-performance motion control with advanced features including trajectory execution, servo positioning, and comprehensive diagnostics.

## Quick Start

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/acidtech/basicmicro_ros2.git
   cd basicmicro_ros2
   ```

2. **Run the automated installer:**
   ```bash
   ./install.sh
   ```
   
   > **Note**: The install script has execute permissions set in the repository, so you can run it directly after cloning.

The installer will:
- Check and install ROS2 if needed
- Clone the required Basicmicro Python library dependency
- Set up a ROS2 workspace with the driver
- Install all dependencies
- Build the package
- Configure serial port permissions

### Manual Installation (Advanced)

If you prefer manual installation:

1. **Clone the repository:**
   ```bash
   git clone https://github.com/acidtech/basicmicro_ros2.git
   cd basicmicro_ros2
   ```

2. **Clone the dependency:**
   ```bash
   cd ..
   git clone https://github.com/acidtech/basicmicro_python.git
   ```

3. **Set up ROS2 workspace:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cp -r basicmicro_ros2 ~/ros2_ws/src/basicmicro_driver
   cd ~/ros2_ws
   ```

4. **Install dependencies:**
   ```bash
   # Install Basicmicro Python library
   cd ../basicmicro_python
   pip install -e .
   cd ~/ros2_ws
   
   # Install ROS2 dependencies
   rosdep install --from-paths src/basicmicro_driver --ignore-src -r -y
   ```

5. **Build the package:**
   ```bash
   colcon build --packages-select basicmicro_driver
   source install/setup.bash
   ```

### Basic Usage

1. **Connect your Basicmicro controller** to the system via USB/Serial
2. **Identify the serial port** (typically `/dev/ttyACM0` on Linux, `COM3` on Windows)
3. **Launch the driver** with a predefined configuration:

```bash
# Differential drive robot
ros2 launch basicmicro_driver basicmicro_driver.launch.py \
    robot_type:=diff_drive \
    port:=/dev/ttyACM0

# Industrial robot
ros2 launch basicmicro_driver basicmicro_driver.launch.py \
    robot_type:=industrial \
    port:=/dev/ttyACM0 \
    wheel_radius:=0.15 \
    wheel_separation:=0.8
```

### Verify Operation

```bash
# Check node status
ros2 node list
ros2 node info /basicmicro_driver

# Test velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.2}}' --once

# Monitor diagnostics
ros2 topic echo /diagnostics
```

## Robot Configuration

The driver supports four predefined robot configurations that can be selected via the `robot_type` parameter:

### 1. Differential Drive (`diff_drive`)
- **Use Case**: Educational robots, small autonomous vehicles
- **Features**: Basic differential drive kinematics, standard sensors
- **Default Parameters**: 0.1m wheel radius, 0.3m separation
- **Example Robots**: TurtleBot-style platforms, research robots

### 2. Industrial (`industrial`)
- **Use Case**: Heavy-duty applications, AGVs, material handling
- **Features**: High torque motors, enhanced safety, payload platform
- **Default Parameters**: 0.15m wheel radius, 0.8m separation, current monitoring
- **Example Robots**: Warehouse AGVs, industrial carriers

### 3. Multi-Controller (`multi_controller`)
- **Use Case**: Complex robots with multiple independent drive systems
- **Features**: 4WD/6WD support, coordinated motion control
- **Default Parameters**: Multiple controller addresses, synchronized operation
- **Example Robots**: All-terrain vehicles, large mobile platforms

### 4. Custom (`custom`)
- **Use Case**: Specialized applications requiring unique configurations
- **Features**: Full parameter customization, advanced motion strategies
- **Default Parameters**: Configurable via YAML files
- **Example Robots**: Research platforms, specialized industrial equipment

## Motion Control Features

### Basic Motion Commands

```bash
# Velocity control (standard ROS2 interface)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.5}}'

# Emergency stop
ros2 service call /emergency_stop std_srvs/srv/Empty
```

### Advanced Motion Services

```bash
# Change motion strategy
ros2 service call /set_motion_strategy basicmicro_driver/srv/SetMotionStrategy \
  '{strategy: "speed_accel"}'

# Distance-based movement
ros2 service call /move_distance basicmicro_driver/srv/MoveDistance \
  '{left_distance: 1.0, right_distance: 1.0, speed: 0.5, acceleration: 1.0}'

# Absolute position control (servo mode)
ros2 service call /move_to_absolute_position basicmicro_driver/srv/MoveToAbsolutePosition \
  '{left_position_radians: 3.14, right_position_radians: 3.14, max_speed: 0.5}'
```

### Trajectory Execution

```bash
# Execute complex trajectory
ros2 service call /execute_trajectory basicmicro_driver/srv/ExecuteTrajectory \
  '{trajectory_points: [
    {command_type: "distance", left_distance: 1.0, right_distance: 1.0, speed: 0.5, acceleration: 1.0},
    {command_type: "distance", left_distance: 0.5, right_distance: -0.5, speed: 0.3, acceleration: 0.8}
  ], trajectory_type: "distance"}'
```

## Configuration Parameters

### Hardware Connection
- `port`: Serial port path (default: `/dev/ttyACM0`)
- `baud`: Communication baud rate (default: `38400`)
- `address`: Controller address (default: `128`)

### Robot Physical Parameters
- `wheel_radius`: Wheel radius in meters (default: `0.1`)
- `wheel_separation`: Distance between wheels in meters (default: `0.3`)
- `encoder_counts_per_rev`: Encoder resolution (default: `1000`)
- `gear_ratio`: Motor gear ratio (default: `1.0`)

### Motion Control Parameters
- `motion_strategy`: Motion control strategy (`duty`, `speed`, `speed_accel`, `duty_accel`)
- `buffer_depth`: Command buffer size (default: `4`, max: `32`)
- `default_acceleration`: Default acceleration rate (default: `1000`)

### Servo Parameters (Position Control)
- `encoder_type`: Encoder type (`incremental` or `absolute`)
- `auto_home_on_startup`: Automatic homing on startup (default: `false`)
- `position_limits_enabled`: Enable position limits (default: `false`)
- `limit_violation_behavior`: Limit violation response (`hard_stop`, `soft_stop`, `warning`)

## Safety Features

### Emergency Stop
- **Service Interface**: `/emergency_stop` (std_srvs/srv/Empty)
- **Topic Interface**: `/emergency_stop` (std_msgs/msg/Empty)
- **Behavior**: Immediately stops motors and clears all command buffers

### Position Limits
```bash
# Configure position limits
ros2 service call /set_position_limits basicmicro_driver/srv/SetPositionLimits \
  '{enable_limits: true, 
    left_min_position: -10.0, left_max_position: 10.0,
    right_min_position: -10.0, right_max_position: 10.0,
    violation_behavior: "soft_stop", decel_rate: 2.0}'
```

### Diagnostic Monitoring
```bash
# Monitor system health
ros2 topic echo /diagnostics

# Check hardware status
ros2 topic echo /basicmicro_status
```

## Testing and Validation

### Run Test Suite
```bash
cd basicmicro_driver

# Run all tests
python test/test_runner.py --unit --integration --performance --regression --coverage

# Run specific test categories
python -m pytest test/unit/ -v                    # Unit tests
python -m pytest test/integration/ -v             # Integration tests  
python -m pytest test/performance/ -v             # Performance tests
python -m pytest test/regression/ -v              # Regression tests
python -m pytest test/hardware/ -v --hardware     # Hardware tests (optional)
```

### Hardware-Free Testing
The driver includes comprehensive mock interfaces for testing without hardware:
```bash
# Test without hardware dependencies
python -m pytest test/unit/ test/test_package_structure.py test/test_unit_converter.py -v
```

## Performance Characteristics

### Target Performance
- **Command Latency**: <5ms for simple commands, <10ms for complex commands
- **Sensor Reading**: >100Hz sustained operation
- **Memory Usage**: <100MB for typical operation
- **CPU Usage**: <50% on modern hardware during normal operation

### Optimization Features
- Batch sensor reading for efficiency
- Command buffering for smooth motion
- Configurable monitoring rates
- Cython-optimized critical paths
- Asynchronous I/O for responsiveness

## Supported Controllers

### RoboClaw Series
- **Models**: All current RoboClaw models
- **Features**: Basic and advanced motion control, servo positioning
- **Homing**: Home pin (backward direction only), auto-homing on startup
- **Communication**: USB, Serial, Packet Serial Protocol

### MCP Series
- **Models**: All current MCP models  
- **Features**: Enhanced servo capabilities, bidirectional homing
- **Homing**: Flexible home pin configuration, current limit homing
- **Communication**: USB, Serial, advanced diagnostic features

## Integration Examples

### With Navigation Stack
```bash
# Launch with nav2 integration
ros2 launch basicmicro_driver basicmicro_driver.launch.py \
    robot_type:=diff_drive \
    port:=/dev/ttyACM0 \
    use_nav2:=true
```

### With MoveIt2
```bash
# Launch with MoveIt2 support
ros2 launch basicmicro_driver basicmicro_driver.launch.py \
    robot_type:=industrial \
    port:=/dev/ttyACM0 \
    use_moveit:=true
```

### Custom URDF Integration
```xml
<ros2_control name="BasicmicroSystem" type="system">
  <hardware>
    <plugin>basicmicro_driver/BasicmicroHardwareInterface</plugin>
    <param name="port">/dev/ttyACM0</param>
    <param name="baud">38400</param>
    <param name="address">128</param>
    <param name="wheel_radius">0.1</param>
    <param name="wheel_separation">0.3</param>
    <param name="motion_strategy">speed_accel</param>
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

## Troubleshooting

### Common Issues

**Problem**: Driver fails to connect to controller
- **Solution**: Check serial port permissions, verify port path, try different baud rates
- **Commands**: 
  ```bash
  ls -la /dev/ttyACM*
  sudo usermod -a -G dialout $USER  # Add user to dialout group
  ```

**Problem**: Poor motion performance
- **Solution**: Adjust motion strategy, tune acceleration parameters, check communication timing
- **Commands**:
  ```bash
  ros2 service call /set_motion_strategy basicmicro_driver/srv/SetMotionStrategy '{strategy: "speed_accel"}'
  ```

**Problem**: Position drift in servo mode
- **Solution**: Perform homing sequence, check encoder connections, verify position limits
- **Commands**:
  ```bash
  ros2 service call /perform_homing basicmicro_driver/srv/PerformHoming \
    '{method_id: "home_pin_backward", direction: "backward", homing_speed: 0.1}'
  ```

### Diagnostic Commands
```bash
# Check controller version and capabilities
ros2 topic echo /diagnostics --once

# Monitor communication status
ros2 topic echo /basicmicro_status

# Test basic connectivity
ros2 service call /get_servo_status basicmicro_driver/srv/GetServoStatus
```

## Development and Contributing

### Building from Source
```bash
# Development build with debug symbols
colcon build --packages-select basicmicro_driver --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run style checks
cd basicmicro_driver
black --check basicmicro_driver/ test/
isort --check-only basicmicro_driver/ test/
flake8 basicmicro_driver/ test/
mypy basicmicro_driver/
```

### Testing Guidelines
- All new features must include unit tests
- Integration tests required for service interfaces
- Performance tests for optimization validation
- Hardware-in-the-loop tests for controller compatibility

### Code Style
- Follow ROS2 Python style guidelines
- Use type hints for all public interfaces
- Include comprehensive docstrings
- Maintain test coverage >90%

## Documentation

- **API Reference**: [docs/api_reference.md](docs/api_reference.md)
- **Tutorial Examples**: [docs/tutorial_examples.md](docs/tutorial_examples.md)
- **Troubleshooting Guide**: [docs/troubleshooting.md](docs/troubleshooting.md)
- **Developer Guide**: [docs/developer_guide.md](docs/developer_guide.md)
- **Launch Files**: [docs/launch_files.md](docs/launch_files.md)
- **URDF Integration**: [docs/urdf_integration_guide.md](docs/urdf_integration_guide.md)

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Support

- **Issues**: Report bugs and feature requests on GitHub Issues
- **Discussions**: Join community discussions on GitHub Discussions  
- **Documentation**: Comprehensive guides in the docs/ directory
- **Examples**: Working examples in the launch/ and urdf/ directories

## Acknowledgments

- Built on the comprehensive [Basicmicro Python Library](https://github.com/acidtech/basicmicro_python)
- Follows ROS2 and ros2_control best practices
- Tested across multiple controller types and robot configurations