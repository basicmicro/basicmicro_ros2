# Service API Complete Reference

The Basicmicro ROS2 driver provides a comprehensive service-based API for advanced motor control and configuration. This document covers all 15 service interfaces with detailed parameters, usage examples, error codes, and interaction patterns.

## Service Overview

The driver provides services organized by functionality:

- **Motion Configuration**: Runtime motion strategy changes and parameter updates
- **Movement Control**: Distance-based movements and trajectory execution
- **Position Control**: Servo positioning and homing operations
- **Direct Control**: Duty cycle and velocity control
- **System Operations**: Buffer management, diagnostics, and emergency functions

## Motion Configuration Services

### SetMotionStrategy

Change the motion control strategy at runtime.

**Service Type:** `basicmicro_driver/srv/SetMotionStrategy`

**Parameters:**
- `strategy` (string): Motion strategy ("DUTY", "SPEED", "SPEED_ACCEL", "POSITION")

**Response:**
- `success` (bool): Operation success status
- `message` (string): Status message or error description

**Usage Example:**
```bash
ros2 service call /basicmicro/set_motion_strategy basicmicro_driver/srv/SetMotionStrategy "{strategy: 'SPEED_ACCEL'}"
```

**Code Example:**
```python
from basicmicro_driver.srv import SetMotionStrategy

def change_to_speed_control(node):
    client = node.create_client(SetMotionStrategy, '/basicmicro/set_motion_strategy')
    request = SetMotionStrategy.Request()
    request.strategy = 'SPEED'
    
    future = client.call_async(request)
    response = future.result()
    
    if response.success:
        node.get_logger().info(f"Strategy changed: {response.message}")
    else:
        node.get_logger().error(f"Failed to change strategy: {response.message}")
```

### ConfigureMotion

Configure motion parameters for the current strategy.

**Service Type:** `basicmicro_driver/srv/ConfigureMotion`

**Parameters:**
- `max_speed` (float): Maximum velocity (rad/s)
- `max_acceleration` (float): Maximum acceleration (rad/s²)
- `position_tolerance` (float): Position tolerance (rad)

**Response:**
- `success` (bool): Configuration success
- `message` (string): Configuration status
- `applied_config` (MotionConfig): Applied configuration parameters

**Usage Example:**
```bash
ros2 service call /basicmicro/configure_motion basicmicro_driver/srv/ConfigureMotion "{max_speed: 2.0, max_acceleration: 1.0, position_tolerance: 0.01}"
```

## Movement Control Services

### MoveDistance

Execute distance-based movement for both motors.

**Service Type:** `basicmicro_driver/srv/MoveDistance`

**Parameters:**
- `distance_m1` (float): Motor 1 distance (meters)
- `distance_m2` (float): Motor 2 distance (meters)
- `max_speed` (float): Maximum velocity (m/s)
- `acceleration` (float): Acceleration limit (m/s²)
- `buffered` (bool): Use buffered execution

**Response:**
- `success` (bool): Command acceptance status
- `message` (string): Execution status
- `estimated_duration` (float): Estimated completion time (seconds)

**Usage Example:**
```bash
# Move forward 1 meter
ros2 service call /basicmicro/move_distance basicmicro_driver/srv/MoveDistance "{distance_m1: 1.0, distance_m2: 1.0, max_speed: 0.5, acceleration: 0.2, buffered: false}"
```

**Code Example:**
```python
from basicmicro_driver.srv import MoveDistance

def move_robot_forward(node, distance):
    client = node.create_client(MoveDistance, '/basicmicro/move_distance')
    request = MoveDistance.Request()
    request.distance_m1 = distance
    request.distance_m2 = distance
    request.max_speed = 0.5
    request.acceleration = 0.2
    request.buffered = False
    
    future = client.call_async(request)
    return future
```

### ExecuteTrajectory

Execute multi-point trajectory with position and timing control.

**Service Type:** `basicmicro_driver/srv/ExecuteTrajectory`

**Parameters:**
- `points` (TrajectoryPoint[]): Array of trajectory points
  - `position_m1` (float): Motor 1 position (rad)
  - `position_m2` (float): Motor 2 position (rad)
  - `velocity_m1` (float): Motor 1 velocity (rad/s)
  - `velocity_m2` (float): Motor 2 velocity (rad/s)
  - `time_from_start` (duration): Time from trajectory start
- `replace_running` (bool): Replace currently executing trajectory

**Response:**
- `success` (bool): Trajectory acceptance
- `message` (string): Execution status
- `trajectory_id` (int32): Unique trajectory identifier
- `total_duration` (float): Complete trajectory duration

**Usage Example:**
```bash
# Execute a 3-point trajectory
ros2 service call /basicmicro/execute_trajectory basicmicro_driver/srv/ExecuteTrajectory \
"{ points: [
  {position_m1: 0.0, position_m2: 0.0, velocity_m1: 0.0, velocity_m2: 0.0, time_from_start: {sec: 0, nanosec: 0}},
  {position_m1: 1.57, position_m2: -1.57, velocity_m1: 1.0, velocity_m2: -1.0, time_from_start: {sec: 2, nanosec: 0}},
  {position_m1: 0.0, position_m2: 0.0, velocity_m1: 0.0, velocity_m2: 0.0, time_from_start: {sec: 4, nanosec: 0}}
], replace_running: true }"
```

## Position Control Services

### SetServoPosition

Set absolute servo position for precise positioning applications.

**Service Type:** `basicmicro_driver/srv/SetServoPosition`

**Parameters:**
- `position_m1` (float): Motor 1 target position (rad)
- `position_m2` (float): Motor 2 target position (rad)
- `max_velocity` (float): Maximum velocity (rad/s)
- `acceleration` (float): Acceleration limit (rad/s²)
- `hold_position` (bool): Maintain position after reaching target

**Response:**
- `success` (bool): Command acceptance
- `message` (string): Execution status
- `estimated_time` (float): Estimated movement time
- `final_position_m1` (float): Actual achieved position motor 1
- `final_position_m2` (float): Actual achieved position motor 2

**Usage Example:**
```bash
# Position robot arm joints
ros2 service call /basicmicro/set_servo_position basicmicro_driver/srv/SetServoPosition \
"{position_m1: 1.57, position_m2: -0.785, max_velocity: 1.0, acceleration: 0.5, hold_position: true}"
```

### HomeMotors

Execute homing sequence for motor position calibration.

**Service Type:** `basicmicro_driver/srv/HomeMotors`

**Parameters:**
- `method` (string): Homing method ("limit_switch", "hard_stop", "encoder_index")
- `direction` (string): Homing direction ("forward", "reverse", "both")
- `speed` (float): Homing velocity (rad/s)
- `timeout` (float): Homing timeout (seconds)

**Response:**
- `success` (bool): Homing success
- `message` (string): Homing status
- `home_position_m1` (float): Motor 1 home position
- `home_position_m2` (float): Motor 2 home position
- `encoder_zeroed` (bool): Encoder zero reset status

**Usage Example:**
```bash
# Home both motors using limit switches
ros2 service call /basicmicro/home_motors basicmicro_driver/srv/HomeMotors \
"{method: 'limit_switch', direction: 'reverse', speed: 0.2, timeout: 30.0}"
```

## Direct Control Services

### SetDutyCycle

Direct PWM duty cycle control for open-loop operation.

**Service Type:** `basicmicro_driver/srv/SetDutyCycle`

**Parameters:**
- `duty_m1` (float): Motor 1 duty cycle (-1.0 to +1.0)
- `duty_m2` (float): Motor 2 duty cycle (-1.0 to +1.0)
- `acceleration` (float): Acceleration limit (optional)
- `duration` (float): Duration in seconds (0 = indefinite)

**Response:**
- `success` (bool): Command execution status
- `message` (string): Status message
- `actual_duty_m1` (float): Applied duty cycle motor 1
- `actual_duty_m2` (float): Applied duty cycle motor 2

**Usage Example:**
```bash
# Set 50% forward duty cycle on both motors
ros2 service call /basicmicro/set_duty_cycle basicmicro_driver/srv/SetDutyCycle \
"{duty_m1: 0.5, duty_m2: 0.5, acceleration: 0.0, duration: 0.0}"
```

### SetVelocity

Direct velocity control with real-time feedback.

**Service Type:** `basicmicro_driver/srv/SetVelocity`

**Parameters:**
- `velocity_m1` (float): Motor 1 velocity (rad/s)
- `velocity_m2` (float): Motor 2 velocity (rad/s)
- `acceleration` (float): Acceleration limit (rad/s²)
- `timeout` (float): Command timeout (seconds)

**Response:**
- `success` (bool): Command acceptance
- `message` (string): Status message
- `achieved_velocity_m1` (float): Actual velocity motor 1
- `achieved_velocity_m2` (float): Actual velocity motor 2

**Usage Example:**
```bash
# Set different velocities for differential steering
ros2 service call /basicmicro/set_velocity basicmicro_driver/srv/SetVelocity \
"{velocity_m1: 2.0, velocity_m2: 1.0, acceleration: 1.0, timeout: 5.0}"
```

## System Operation Services

### EmergencyStop

Immediate motor stop with safety priority.

**Service Type:** `basicmicro_driver/srv/EmergencyStop`

**Parameters:**
- `stop_type` (string): Stop type ("immediate", "decelerate", "coast")
- `preserve_position` (bool): Maintain position tracking

**Response:**
- `success` (bool): Stop execution status
- `message` (string): Stop status message
- `stop_time` (float): Actual stop time achieved
- `final_position_m1` (float): Motor 1 final position
- `final_position_m2` (float): Motor 2 final position

**Usage Example:**
```bash
# Immediate emergency stop
ros2 service call /basicmicro/emergency_stop basicmicro_driver/srv/EmergencyStop \
"{stop_type: 'immediate', preserve_position: true}"
```

### GetSystemStatus

Retrieve comprehensive system status and diagnostics.

**Service Type:** `basicmicro_driver/srv/GetSystemStatus`

**Parameters:**
- `include_diagnostics` (bool): Include detailed diagnostics
- `include_buffer_status` (bool): Include buffer information

**Response:**
- `success` (bool): Status retrieval success
- `message` (string): Status message
- `controller_version` (string): Controller firmware version
- `motion_strategy` (string): Current motion strategy
- `motor_status` (MotorStatus[]): Per-motor status information
- `buffer_utilization` (float): Command buffer usage percentage
- `communication_health` (float): Communication success rate

**Usage Example:**
```bash
# Get complete system status
ros2 service call /basicmicro/get_system_status basicmicro_driver/srv/GetSystemStatus \
"{include_diagnostics: true, include_buffer_status: true}"
```

### ClearBuffer

Clear command buffer and reset trajectory execution.

**Service Type:** `basicmicro_driver/srv/ClearBuffer`

**Parameters:**
- `stop_motors` (bool): Stop motors before clearing buffer
- `reset_position` (bool): Reset position tracking

**Response:**
- `success` (bool): Buffer clear success
- `message` (string): Clear status message
- `commands_cleared` (int32): Number of commands removed
- `buffer_state` (string): Final buffer state

**Usage Example:**
```bash
# Clear buffer and stop motors safely
ros2 service call /basicmicro/clear_buffer basicmicro_driver/srv/ClearBuffer \
"{stop_motors: true, reset_position: false}"
```

## Service Interaction Patterns

### Sequential Movement Execution

```python
async def execute_sequential_movements(node):
    # 1. Set motion strategy
    await change_motion_strategy(node, 'SPEED_ACCEL')
    
    # 2. Move forward
    await move_distance(node, 1.0, 1.0)
    
    # 3. Wait for completion
    await wait_for_motion_complete(node)
    
    # 4. Turn in place
    await move_distance(node, 0.5, -0.5)
    
    # 5. Return to start
    await move_distance(node, -1.5, -0.5)
```

### Trajectory Execution with Monitoring

```python
def execute_monitored_trajectory(node, trajectory_points):
    # Start trajectory
    response = call_execute_trajectory(node, trajectory_points)
    trajectory_id = response.trajectory_id
    
    # Monitor execution
    while True:
        status = call_get_system_status(node)
        if trajectory_complete(status, trajectory_id):
            break
        time.sleep(0.1)
    
    return status
```

### Error Recovery Pattern

```python
def robust_position_control(node, target_position):
    max_retries = 3
    for attempt in range(max_retries):
        try:
            response = call_set_servo_position(node, target_position)
            if response.success:
                return response
        except Exception as e:
            if attempt == max_retries - 1:
                # Final attempt - use emergency stop
                call_emergency_stop(node)
                raise
            
            # Retry with reduced speed
            target_position.max_velocity *= 0.8
    
    return None
```

## Error Codes and Troubleshooting

### Common Error Codes

**Communication Errors:**
- `COMM_TIMEOUT`: Controller communication timeout
- `COMM_CRC_ERROR`: CRC validation failure
- `COMM_NO_RESPONSE`: No response from controller

**Motion Errors:**
- `MOTION_LIMIT_EXCEEDED`: Position or velocity limit violation
- `MOTION_COLLISION_DETECTED`: Unexpected resistance or collision
- `MOTION_ENCODER_ERROR`: Encoder feedback failure

**System Errors:**
- `SYSTEM_EMERGENCY_STOP`: Emergency stop activated
- `SYSTEM_OVERTEMPERATURE`: Thermal protection triggered
- `SYSTEM_UNDERVOLTAGE`: Supply voltage too low

### Error Recovery Procedures

**Communication Recovery:**
1. Check physical connections
2. Reduce baud rate if necessary
3. Reset controller and reconnect
4. Verify controller address

**Motion Recovery:**
1. Clear command buffer
2. Reset emergency stop if active
3. Verify motion limits and constraints
4. Re-home motors if position tracking lost

**System Recovery:**
1. Check power supply stability
2. Verify motor current limits
3. Allow cooling time if overheated
4. Inspect mechanical systems for binding

## Performance Optimization

### Service Call Optimization

**Batch Operations:**
```python
# Instead of multiple individual calls
for position in position_list:
    call_set_servo_position(node, position)

# Use trajectory execution
call_execute_trajectory(node, convert_to_trajectory(position_list))
```

**Asynchronous Execution:**
```python
# Non-blocking service calls
future1 = client1.call_async(request1)
future2 = client2.call_async(request2)

# Process results when ready
result1 = await future1
result2 = await future2
```

### Buffer Management

Monitor buffer utilization to prevent overflow:
```python
def check_buffer_health(node):
    status = call_get_system_status(node, include_buffer_status=True)
    if status.buffer_utilization > 0.8:
        node.get_logger().warn("Buffer utilization high: {:.1f}%".format(
            status.buffer_utilization * 100))
        return False
    return True
```

## Complete Service List

| Service Name | Purpose | Strategy Required |
|--------------|---------|-------------------|
| SetMotionStrategy | Change motion control mode | Any |
| ConfigureMotion | Set motion parameters | Any |
| MoveDistance | Distance-based movement | SPEED/SPEED_ACCEL |
| ExecuteTrajectory | Multi-point trajectory | POSITION |
| SetServoPosition | Absolute positioning | POSITION |
| HomeMotors | Position calibration | POSITION |
| SetDutyCycle | Direct PWM control | DUTY |
| SetVelocity | Direct velocity control | SPEED/SPEED_ACCEL |
| EmergencyStop | Safety stop | Any |
| GetSystemStatus | System diagnostics | Any |
| ClearBuffer | Reset command buffer | Any |
| ResetEncoders | Zero encoder positions | Any |
| SetLimits | Configure motion limits | Any |
| GetMotorInfo | Motor specifications | Any |
| CalibrateSystem | System calibration | Any |

For implementation examples and complete code samples, see the [Examples Documentation](rover_example.md) and [Hardware Validation Guide](hardware_validation.md).