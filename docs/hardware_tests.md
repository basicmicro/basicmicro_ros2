# Hardware Tests Guide

The hardware test suite validates real hardware communication and control with actual Basicmicro controllers. These tests provide the ultimate validation that the driver works correctly with physical motors and encoders.

## Overview

**Purpose**: Validate real hardware communication and motor control
- **Hardware Communication Tests**: Controller connection and status
- **Motor Control Validation**: Actual movement with encoder feedback
- **Safety System Tests**: Emergency stop and limit enforcement
- **Performance Validation**: Real-world performance metrics
- **Requirements**: Physical Basicmicro controller connected via USB

## Prerequisites

### Hardware Requirements
```bash
# Supported controllers (tested)
- USB Roboclaw 2x15a v4.4.2 or newer
- USB Roboclaw 2x7a v4.x
- MCP series controllers
- Basicmicro Simple Motor Controllers

# Connection requirements
- USB cable connected to development machine
- Controller powered and configured
- Motors connected (optional for basic tests)
- Encoders connected (recommended for full validation)
```

### Hardware Setup
```bash
# 1. Connect controller via USB
# Controller should appear as /dev/ttyACM0, /dev/ttyACM1, etc. (Linux)
# or COM3, COM4, etc. (Windows)

# 2. Verify device detection
ls -la /dev/ttyACM*
# Expected: /dev/ttyACM0 or /dev/ttyACM1

# 3. Check device permissions (Linux)
groups $USER
# Should include 'dialout' group

# 4. Add user to dialout group if needed
sudo usermod -a -G dialout $USER
# Logout/login or use: newgrp dialout
```

### Software Prerequisites
```bash
# ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build and source workspace
cd /path/to/workspace
colcon build --packages-select basicmicro_driver
source install/setup.bash

# Basicmicro Python library
cd Basicmicro_python
pip install -e .

# Hardware test dependencies
pip install pyserial
```

### Motion Studio Configuration
**CRITICAL**: Controllers must be configured in Motion Studio before ROS2 testing:

```bash
# Required Motion Studio configuration:
1. Set appropriate PID values for motors
2. Configure encoder settings (if using encoders)
3. Set current limits and safety parameters
4. Test basic movement in Motion Studio first
5. Note the USB port and controller address
```

## Running Hardware Tests

### Basic Hardware Connection Test
```bash
# Test basic controller communication
python3 -c "
from basicmicro import Basicmicro
controller = Basicmicro('/dev/ttyACM1', 38400)
print('Connected:', controller.Open())
if controller.Open():
    version = controller.ReadVersion(0x80)
    print('Version:', version[1] if version[0] else 'Connection failed')
    controller.close()
"

# Expected output:
# Connected: True
# Version: USB Roboclaw 2x15a v4.4.2
```

### Hardware Test Suite Execution
```bash
# Run all hardware tests (requires --hardware flag)
python3 -m pytest test/hardware/ --hardware --hardware-port=/dev/ttyACM1 -v

# With specific controller settings
python3 -m pytest test/hardware/ --hardware --hardware-port=/dev/ttyACM1 --hardware-baud=38400 --hardware-address=128 -v

# Windows example
python3 -m pytest test/hardware/ --hardware --hardware-port=COM3 -v
```

### Focused Hardware Testing
```bash
# Test specific hardware functionality
python3 -m pytest test/hardware/test_basic_communication.py --hardware --hardware-port=/dev/ttyACM1 -v
python3 -m pytest test/hardware/test_motor_control.py --hardware --hardware-port=/dev/ttyACM1 -v

# Test with detailed output
python3 -m pytest test/hardware/ --hardware --hardware-port=/dev/ttyACM1 -v --tb=long

# Test with safety checks
python3 -m pytest test/hardware/ --hardware --hardware-port=/dev/ttyACM1 -v --hardware-safety-checks
```

## Hardware Test Categories

### Basic Communication Tests

#### **Controller Version and Status**
```bash
test_controller_connection
test_controller_version_reading
test_controller_status_monitoring
test_communication_error_handling
```

**What these test**:
- USB connection establishment
- Controller identification and version
- Basic communication protocol
- Error handling for communication failures

**Expected results**:
```bash
test_controller_connection PASSED
# Validates: USB connection successful

test_controller_version_reading PASSED  
# Validates: Version string readable (e.g., "USB Roboclaw 2x15a v4.4.2")

test_controller_status_monitoring PASSED
# Validates: Status registers accessible

test_communication_error_handling PASSED
# Validates: Graceful handling of communication timeouts
```

#### **Encoder and Sensor Reading**
```bash
test_encoder_reading
test_speed_reading  
test_current_monitoring
test_voltage_monitoring
```

**What these test**:
- Encoder position reading
- Speed measurement accuracy
- Current draw monitoring
- Voltage level monitoring

**Expected results**:
```bash
test_encoder_reading PASSED
# Validates: Encoder positions readable (if encoders connected)

test_speed_reading PASSED
# Validates: Motor speeds readable and accurate

test_current_monitoring PASSED  
# Validates: Current consumption monitoring

test_voltage_monitoring PASSED
# Validates: Supply voltage monitoring
```

### Motor Control Validation

#### **Basic Motor Movement**
```bash
test_duty_cycle_control
test_speed_control
test_acceleration_control
test_emergency_stop_response
```

**What these test**:
- Direct duty cycle motor control
- Closed-loop speed control (requires encoders)
- Acceleration-limited movement
- Emergency stop functionality

**Expected results**:
```bash
test_duty_cycle_control PASSED
# Validates: Motors respond to duty cycle commands
# Evidence: Encoder position changes detected

test_speed_control PASSED
# Validates: Closed-loop speed control functioning
# Evidence: Actual speed matches commanded speed

test_acceleration_control PASSED
# Validates: Acceleration limits enforced
# Evidence: Smooth acceleration curves

test_emergency_stop_response PASSED
# Validates: Motors stop immediately on emergency command
# Evidence: Motor movement stops within safety timeframe
```

#### **Position and Trajectory Control**
```bash
test_position_control_accuracy
test_trajectory_execution
test_buffer_utilization
test_multi_point_trajectory
```

**What these test**:
- Servo position accuracy
- Trajectory following performance
- Buffer management under load
- Complex multi-point movements

**Expected results**:
```bash
test_position_control_accuracy PASSED
# Validates: Position accuracy within ±1% of target
# Evidence: Final position matches commanded position

test_trajectory_execution PASSED
# Validates: Smooth trajectory following
# Evidence: Trajectory completed within time tolerance

test_buffer_utilization PASSED
# Validates: Buffer management prevents overflows
# Evidence: Buffer status monitoring shows proper utilization

test_multi_point_trajectory PASSED
# Validates: Complex trajectory execution
# Evidence: All waypoints reached within tolerance
```

### Safety System Tests

#### **Emergency Stop and Limits**
```bash
test_emergency_stop_hardware
test_software_safety_limits
test_current_limit_enforcement
test_error_recovery_procedures
```

**What these test**:
- Hardware emergency stop response
- Software safety limit enforcement
- Current limit protection
- Error recovery and resumption

**Safety validation**:
```bash
test_emergency_stop_hardware PASSED
# Validates: Emergency stop stops motors within 100ms
# Evidence: Motor current drops to zero immediately

test_software_safety_limits PASSED
# Validates: Position and speed limits enforced
# Evidence: Commands beyond limits rejected

test_current_limit_enforcement PASSED
# Validates: Current limits prevent motor damage
# Evidence: Current draw limited to configured maximum

test_error_recovery_procedures PASSED
# Validates: System recovers from error conditions
# Evidence: Normal operation resumes after error clearance
```

### Performance Validation

#### **Real-Time Performance**
```bash
test_command_latency
test_feedback_frequency
test_buffer_performance
test_throughput_limits
```

**What these test**:
- Command-to-response latency
- Sensor feedback update frequency
- Buffer management efficiency
- Maximum command throughput

**Performance metrics**:
```bash
test_command_latency PASSED
# Validates: Command latency <50ms average
# Evidence: Timestamped command/response measurement

test_feedback_frequency PASSED
# Validates: Encoder feedback at expected frequency
# Evidence: Sensor updates at 100Hz or configured rate

test_buffer_performance PASSED
# Validates: Buffer utilization optimized
# Evidence: Buffer usage <80% under normal load

test_throughput_limits PASSED
# Validates: Maximum command rate sustainable
# Evidence: System handles expected command frequency
```

## Hardware Test Framework

### Test Parameter Configuration

#### **Hardware Test Parameters**
```python
# Test configuration
hardware_test_config = {
    'port': '/dev/ttyACM1',           # USB device path
    'baud': 38400,                    # Communication baud rate
    'address': 0x80,                  # Controller address (128)
    'test_timeout': 10.0,             # Test timeout (seconds)
    'safety_timeout': 2.0,            # Emergency stop timeout
    'position_tolerance': 0.01,       # Position accuracy (1%)
    'speed_tolerance': 0.05,          # Speed accuracy (5%)
    'current_limit': 10.0,            # Maximum test current (A)
}
```

#### **Motor Test Parameters**
```python
# Safe test movement parameters
test_movements = {
    'duty_cycle_range': [-16384, 16384],    # ±50% duty cycle
    'speed_range': [100, 1000],             # counts/sec
    'position_range': [0, 1000],            # encoder counts
    'acceleration': 500,                     # counts/sec²
    'test_duration': 2.0,                   # seconds per test
}
```

### Safety Mechanisms

#### **Built-in Safety Features**
```python
class HardwareTestSafety:
    def setUp(self):
        # Automatic safety timeouts
        self.emergency_timeout = 2.0  # seconds
        self.movement_timeout = 10.0  # seconds
        
        # Safety limits
        self.max_duty_cycle = 16384   # 50% maximum
        self.max_test_current = 10.0  # 10A maximum
        
        # Automatic cleanup
        self.addCleanup(self.emergency_stop)
        self.addCleanup(self.close_connection)
    
    def tearDown(self):
        # Ensure motors stopped
        self.controller.DutyM1M2(self.address, 0, 0)
        # Verify stopped
        time.sleep(0.1)
        speeds = self.controller.GetSpeeds(self.address)
        assert abs(speeds[1]) < 10  # Near zero speed
```

#### **Test Validation Patterns**
```python
def test_motor_movement_with_validation(self):
    # Record initial state
    initial_encoders = self.controller.GetEncoders(self.address)
    
    # Execute test movement
    result = self.controller.SpeedM1M2(self.address, 500, 500)
    assert result, "Speed command failed"
    
    # Allow movement time
    time.sleep(1.0)
    
    # Validate movement occurred
    final_encoders = self.controller.GetEncoders(self.address)
    left_change = abs(final_encoders[1] - initial_encoders[1])
    right_change = abs(final_encoders[2] - initial_encoders[2])
    
    # Verify actual movement
    assert left_change > 50, f"Left motor did not move sufficiently: {left_change}"
    assert right_change > 50, f"Right motor did not move sufficiently: {right_change}"
    
    # Safety stop
    self.controller.DutyM1M2(self.address, 0, 0)
```

## Expected Hardware Test Results

### Successful Hardware Test Run
```bash
test/hardware/test_basic_communication.py::TestBasicCommunication::test_controller_connection PASSED
test/hardware/test_basic_communication.py::TestBasicCommunication::test_controller_version_reading PASSED
test/hardware/test_basic_communication.py::TestBasicCommunication::test_encoder_reading PASSED
test/hardware/test_motor_control.py::TestMotorControl::test_duty_cycle_control PASSED
test/hardware/test_motor_control.py::TestMotorControl::test_speed_control PASSED
test/hardware/test_motor_control.py::TestMotorControl::test_emergency_stop_response PASSED
test/hardware/test_safety_systems.py::TestSafetySystems::test_emergency_stop_hardware PASSED
test/hardware/test_safety_systems.py::TestSafetySystems::test_software_safety_limits PASSED
test/hardware/test_performance.py::TestPerformance::test_command_latency PASSED
test/hardware/test_performance.py::TestPerformance::test_buffer_performance PASSED
===================== 10 passed in 45.67s ======================
```

### Performance Metrics Example
```bash
# Hardware performance test results
Command Latency: 15.3ms average (target: <50ms) ✅
Feedback Frequency: 98.7Hz (target: 100Hz) ✅
Buffer Utilization: 23.4% average (target: <80%) ✅
Position Accuracy: 0.3% error (target: <1%) ✅
Speed Accuracy: 1.8% error (target: <5%) ✅
Emergency Stop Response: 73ms (target: <100ms) ✅
```

## Troubleshooting Hardware Tests

### Common Hardware Issues

#### **Connection Problems**
```bash
# Device not found
FileNotFoundError: [Errno 2] No such file or directory: '/dev/ttyACM1'

# Solutions:
# 1. Check device connection
ls -la /dev/ttyACM*

# 2. Check USB cable and controller power
# 3. Try different USB port
# 4. Check controller address (0x80 vs 128)
```

#### **Permission Issues (Linux)**
```bash
# Permission denied
PermissionError: [Errno 13] Permission denied: '/dev/ttyACM1'

# Solutions:
# 1. Add user to dialout group
sudo usermod -a -G dialout $USER

# 2. Apply group changes
newgrp dialout

# 3. Verify group membership
groups $USER
```

#### **Communication Failures**
```bash
# Communication timeout
CommunicationError: Controller not responding

# Solutions:
# 1. Check baud rate (38400 vs 115200)
# 2. Check controller address (0x80, 0x81, etc.)
# 3. Verify Motion Studio configuration
# 4. Try manual controller reset
```

#### **Motor Control Issues**
```bash
# Motors not moving
AssertionError: Left motor did not move sufficiently

# Solutions:
# 1. Check motor connections
# 2. Verify Motion Studio PID settings
# 3. Check current limits
# 4. Verify encoder connections (for speed control)
# 5. Test with duty cycle control first
```

### Hardware Validation Checklist

#### **Pre-Test Validation**
```bash
# 1. Controller connection
✅ USB device detected (/dev/ttyACM1)
✅ Device permissions correct (dialout group)
✅ Controller responds to version request
✅ Motion Studio configuration complete

# 2. Motor system validation
✅ Motors connected and powered
✅ Encoders connected (if using feedback)
✅ PID parameters configured in Motion Studio
✅ Current limits set appropriately
✅ Emergency stop accessible
```

#### **Test Execution Validation**
```bash
# 3. Communication tests
✅ Basic controller communication working
✅ Encoder readings available
✅ Status monitoring functional
✅ Error handling working

# 4. Motor control tests
✅ Duty cycle control responds
✅ Speed control accuracy validated
✅ Emergency stop functional
✅ Safety limits enforced

# 5. Performance validation
✅ Command latency within limits
✅ Buffer management working
✅ Feedback frequency adequate
✅ No communication errors
```

### Success Indicators

#### **Hardware Test Success Criteria**
- **All communication tests pass**: Controller responds reliably
- **Motor movement validated**: Actual encoder changes detected
- **Safety systems functional**: Emergency stop working
- **Performance within limits**: Latency and accuracy acceptable
- **No hardware errors**: Clean test execution without timeouts

#### **Production Readiness Indicators**
- **Consistent test results**: Same results across multiple runs
- **Real-world performance**: Metrics suitable for robotics applications
- **Safety compliance**: All safety mechanisms validated
- **Error recovery**: System handles and recovers from failures

The hardware test suite provides the ultimate validation that the ROS2 Basicmicro driver works correctly with real hardware in production robotics applications.