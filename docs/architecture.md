# Package Architecture Overview

Complete architectural overview of the Basicmicro ROS2 driver package, showing component relationships and data flow.

## System Overview

The Basicmicro ROS2 driver provides a comprehensive interface between ROS2 and Basicmicro motor controllers, supporting both differential drive robots and servo positioning systems.

### Key Features
- **Full ROS2 Integration**: Topics, services, parameters, and lifecycle management
- **Hardware Abstraction**: Clean interface between ROS2 and Basicmicro hardware
- **Multiple Motion Strategies**: Duty cycle, speed, acceleration, and position control
- **Advanced Buffer Management**: Trajectory execution with real-time monitoring
- **Comprehensive Diagnostics**: System health monitoring and performance tracking
- **Unit Conversion System**: Seamless conversion between ROS2 and controller units

## Component Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                          ROS2 Layer                            │
├─────────────────────────────────────────────────────────────────┤
│  Topics: /cmd_vel, /odom, /joint_states, /diagnostics          │
│  Services: 15 service interfaces (motion, trajectory, config)   │
│  Parameters: Hardware config, control tuning, robot geometry   │
└─────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Core ROS2 Nodes                            │
├─────────────────────────────────────────────────────────────────┤
│  BasicmicroNode: Main integration node                         │
│  Service Nodes: Motion config, trajectory, servo control       │
│  Hardware Interface: ros2_control system interface             │
└─────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Driver Core Modules                         │
├─────────────────────────────────────────────────────────────────┤
│  UnitConverter: ROS2 ↔ Controller unit conversion              │
│  BufferManager: Trajectory buffering and execution             │
│  DiagnosticPublisher: System health monitoring                 │
│  PerformanceMonitor: Real-time performance tracking            │
└─────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────┐
│                  Hardware Communication                        │
├─────────────────────────────────────────────────────────────────┤
│  Basicmicro Python Library: Low-level controller interface     │
│  Serial Communication: USB/UART to motor controller            │
│  Packet Protocol: CRC-validated command/response               │
└─────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Physical Hardware                           │
├─────────────────────────────────────────────────────────────────┤
│  Basicmicro Controller: RoboClaw, MCP series                   │
│  Motors: Brushed DC, gear motors, servo motors                 │
│  Encoders: Quadrature encoders for position/velocity feedback  │
│  Power System: Battery, power supply, motor drivers            │
└─────────────────────────────────────────────────────────────────┘
```

## Core Modules Detail

### BasicmicroNode (310 lines)
**Primary ROS2 integration node**

**Responsibilities:**
- ROS2 topic subscription (`/cmd_vel`) and publishing (`/odom`, `/joint_states`)
- Parameter management and dynamic reconfiguration
- Hardware lifecycle management (initialization, shutdown)
- Real-time sensor reading and data publishing
- Safety monitoring and emergency stop handling

**Key Interfaces:**
```python
# Subscriptions
/cmd_vel (geometry_msgs/Twist) → Velocity commands
/emergency_stop (std_msgs/Bool) → Emergency stop trigger

# Publications  
/odom (nav_msgs/Odometry) → Robot odometry
/joint_states (sensor_msgs/JointState) → Joint positions
/diagnostics (diagnostic_msgs/DiagnosticArray) → System health

# Services
/emergency_stop (std_srvs/Trigger) → Emergency stop service
```

### Hardware Interface (1,021 lines)
**ros2_control system interface implementation**

**Responsibilities:**
- ros2_control lifecycle management
- Hardware command interpretation and execution
- Motion strategy implementation (DUTY, SPEED, SPEED_ACCEL, POSITION)
- Real-time control loop execution
- Hardware state monitoring and reporting

**Motion Strategies:**
```python
DUTY: Direct PWM control (-32767 to +32767)
SPEED: Velocity control with PID (counts/sec)
SPEED_ACCEL: Velocity with acceleration limits
POSITION: Absolute positioning with PID
```

**Lifecycle States:**
```
Unconfigured → Inactive → Active → Inactive → Finalized
     │           │         │        │         │
     └─configure──┘    activate   deactivate cleanup
```

### Unit Converter (335 lines)
**Comprehensive unit conversion system**

**Conversion Categories:**
```python
# Angular conversions
radians ↔ encoder_counts
rad/sec ↔ counts/sec
rad/sec² ↔ counts/sec²

# Linear conversions (for wheels)
meters ↔ encoder_counts  
m/sec ↔ counts/sec
m/sec² ↔ counts/sec²

# Duty cycle mapping
normalized_duty (-1.0 to +1.0) ↔ controller_duty (-32767 to +32767)
```

**Configuration Parameters:**
```yaml
encoder_counts_per_rev: 500
gear_ratio: 19.0
wheel_radius_m: 0.075
wheel_separation_m: 0.35
```

### Buffer Manager (590 lines)
**Advanced trajectory execution with buffering**

**Features:**
- **Trajectory Buffering**: Queue commands for smooth execution
- **Buffer Monitoring**: Real-time buffer utilization tracking
- **Overflow Protection**: Intelligent buffer management
- **Performance Analysis**: Execution timing and success rate monitoring

**Buffer States:**
```
IDLE (0xFF): No commands queued or executing
EXECUTING (0x00): Command executing, buffer empty  
BUFFERED (1-32): Number of commands in buffer
```

### Diagnostic Publisher (1,041 lines)
**Comprehensive system health monitoring**

**Diagnostic Categories:**
```python
Hardware Status: Controller connection, version, communication
Motor Status: Current, voltage, temperature, encoder feedback
Control Status: Motion strategy, PID performance, safety limits
System Status: Buffer utilization, command latency, error rates
```

**Diagnostic Levels:**
```
OK: Normal operation
WARN: Potential issues detected
ERROR: Requires attention
STALE: Data timeout or communication loss
```

### Performance Monitor (1,168 lines)
**Real-time performance tracking and analysis**

**Monitoring Categories:**
- **Command Latency**: ROS2 command to hardware execution time
- **Throughput**: Commands processed per second
- **Buffer Performance**: Utilization trends and overflow detection
- **Hardware Response**: Motor response time and accuracy
- **Error Tracking**: Fault frequency and recovery patterns

## Data Flow Architecture

### Command Flow (ROS2 → Hardware)
```
1. ROS2 Topic/Service → 2. Node Processing → 3. Unit Conversion → 
4. Hardware Interface → 5. Buffer Manager → 6. Basicmicro Library → 
7. Serial Communication → 8. Motor Controller → 9. Physical Motion
```

### Feedback Flow (Hardware → ROS2)
```
1. Encoder Sensors → 2. Motor Controller → 3. Serial Communication → 
4. Basicmicro Library → 5. Hardware Interface → 6. Unit Conversion → 
7. State Estimation → 8. ROS2 Publishers → 9. Topic Publication
```

### Diagnostic Flow
```
1. Hardware Monitoring → 2. Performance Analysis → 3. Diagnostic Publisher → 
4. ROS2 Diagnostics → 5. System Health Dashboard
```

## Service Architecture

### Motion Configuration Services
```python
SetMotionStrategy: Change control mode (duty/speed/position)
SetMotionParameters: Update PID gains, limits, acceleration
GetMotionConfiguration: Retrieve current control parameters
```

### Movement Control Services  
```python
MoveDistance: Execute distance-based movements
SetDutyCycle: Direct PWM control
SetDutyCycleAccel: PWM with acceleration ramping
```

### Servo Positioning Services
```python
MoveToAbsolutePosition: Precise positioning
PerformHoming: Automated homing sequences
SetPositionLimits: Configure software limits
GetServoStatus: Position and status monitoring
```

### Trajectory Services
```python
ExecuteTrajectory: Multi-point trajectory execution
ExecutePositionSequence: Sequential position moves
TrajectoryMonitoring: Real-time trajectory tracking
```

### Configuration Services
```python
SetHomingConfiguration: Homing method and parameters
GetAvailableHomingMethods: Controller capability query
ReleasePositionHold: Servo position release
```

## Integration Patterns

### ROS2 Control Integration
```python
# Hardware interface registration
<ros2_control name="BasicmicroSystem" type="system">
  <hardware>
    <plugin>basicmicro_driver/BasicmicroHardwareInterface</plugin>
    <param name="port">/dev/ttyACM0</param>
    <param name="baud">38400</param>
  </hardware>
  
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Differential Drive Integration
```python
# Kinematics calculations
def cmd_vel_to_wheel_speeds(linear_x, angular_z):
    left_speed = (linear_x - angular_z * wheel_separation / 2.0) / wheel_radius
    right_speed = (linear_x + angular_z * wheel_separation / 2.0) / wheel_radius
    return left_speed, right_speed

def wheel_speeds_to_odometry(left_speed, right_speed):
    linear_x = (left_speed + right_speed) * wheel_radius / 2.0
    angular_z = (right_speed - left_speed) * wheel_radius / wheel_separation
    return linear_x, angular_z
```

### Navigation Stack Integration
```python
# Standard ROS2 navigation interfaces
/cmd_vel (input): Velocity commands from nav2
/odom (output): Odometry for localization
/tf (output): base_link → odom transform
/joint_states (output): Wheel positions for visualization
```

## Configuration Architecture

### Parameter Hierarchy
```yaml
basicmicro_driver:
  ros__parameters:
    # Hardware Configuration
    hardware:
      port: "/dev/ttyACM0"
      baud: 38400
      address: 128
      
    # Control Configuration  
    control:
      motion_strategy: "SPEED_ACCEL"
      max_speed: 8000
      acceleration: 2000
      
    # Robot Configuration
    robot:
      wheel_separation: 0.35
      wheel_radius: 0.075
      encoder_counts_per_rev: 500
      
    # Performance Configuration
    performance:
      control_frequency: 50.0
      diagnostic_frequency: 1.0
      buffer_size: 32
```

### Launch File Architecture
```python
# Hierarchical launch system
basicmicro_driver.launch.py: Complete system launch
├── basicmicro_simple.launch.py: Basic node only
├── arm_control.launch.py: Servo arm configuration  
├── robot_visualization.launch.py: RVIZ integration
└── hardware_validation.launch.py: Testing and validation
```

## Error Handling Architecture

### Fault Detection Layers
```
1. Hardware Level: Controller internal fault detection
2. Communication Level: Serial timeout and CRC validation
3. Driver Level: Command validation and safety checks
4. ROS2 Level: Topic timeout and parameter validation
5. System Level: Performance monitoring and health checks
```

### Recovery Strategies
```python
# Automatic recovery
Communication Timeout → Reconnection attempt
Parameter Out of Range → Clamp to valid range
Buffer Overflow → Queue management

# Manual recovery required
Hardware Fault → Emergency stop + user intervention
Safety Limit Violation → Stop + manual reset
Controller Error → Power cycle + reconfiguration
```

## Performance Characteristics

### Real-Time Performance
```
Control Loop Frequency: 50Hz (20ms cycle)
Command Latency: <50ms (ROS2 to hardware)
Buffer Utilization: <80% typical, 32 command capacity
Diagnostic Update Rate: 1Hz (system health)
```

### Scalability
```
Single Controller: 2 motors + encoders
Multiple Controllers: Up to 8 controllers (address 0x80-0x87)
Mixed Configuration: Rover + arm on separate controllers
Network Scaling: Multiple robots with namespace isolation
```

## Testing Architecture

### Test Categories
```
Unit Tests (61): Individual module functionality
Integration Tests (14): Inter-module communication
Hardware Tests: Real controller validation
Performance Tests: Timing and throughput validation
Regression Tests: Backwards compatibility
```

### Mock Framework
```python
MockBasicmicro: Complete controller simulation
RealisticMotorDynamics: Physics-based motor response
BufferSimulation: Command queuing behavior
CommunicationSimulation: Serial protocol and timing
```

This architecture provides a robust, scalable foundation for professional robotics applications while maintaining clean separation of concerns and comprehensive error handling.

---
*For detailed hardware interface information, see [Hardware Interface Reference](hardware_interface.md). For service usage, see [Service API Reference](service_api.md).*