# Unit Tests Guide

The unit test suite validates individual component functionality in isolation using comprehensive mock frameworks. All 61 unit tests pass consistently, providing confidence in core component reliability.

## Overview

**Total Unit Tests**: 61 tests (100% passing)
- **Hardware Interface Tests**: 32 tests
- **Service Unit Tests**: 29 tests  
- **Execution Time**: ~8-12 seconds
- **Dependencies**: Mock framework only (no hardware or ROS2 nodes)

## Running Unit Tests

### Basic Unit Test Execution
```bash
# Run all unit tests
python3 -m pytest test/unit/ -v

# Expected output:
# ==================== 61 passed in 8.42s ====================
```

### Focused Unit Testing
```bash
# Test specific component
python3 -m pytest test/unit/test_hardware_interface_unit.py -v
python3 -m pytest test/unit/test_service_unit.py -v

# Test with detailed output
python3 -m pytest test/unit/ -v --tb=long

# Test with coverage (if pytest-cov installed)
python3 -m pytest test/unit/ --cov=basicmicro_driver --cov-report=term-missing
```

## Hardware Interface Unit Tests (32 tests)

### Test Categories

#### **Initialization and Configuration (8 tests)**
```bash
# Tests: on_init, on_configure, parameter extraction, validation
test_on_init_success
test_on_configure_success  
test_parameter_extraction_success
test_parameter_validation_success
test_invalid_parameter_handling
test_configuration_state_transitions
test_parameter_defaults
test_configuration_error_handling
```

**What these test**:
- Hardware interface lifecycle management
- Parameter extraction from ROS2 parameter server
- Validation of motor controller settings
- Error handling for invalid configurations

#### **Hardware Communication (8 tests)**
```bash
# Tests: write, read, hardware controller interaction
test_write_success
test_write_failure_handling
test_read_success  
test_read_failure_handling
test_hardware_interface_connection
test_communication_error_recovery
test_timeout_handling
test_connection_state_management
```

**What these test**:
- Core write/read operations to hardware
- Error handling for communication failures
- Connection state management
- Timeout and recovery procedures

#### **Motion Control (8 tests)**
```bash
# Tests: Different motion strategies and control modes
test_duty_motion_strategy
test_speed_motion_strategy
test_speed_accel_motion_strategy
test_position_motion_strategy
test_strategy_switching
test_motion_parameter_validation
test_control_mode_transitions
test_motion_limit_enforcement
```

**What these test**:
- All four motion strategies (DUTY, SPEED, SPEED_ACCEL, POSITION)
- Runtime strategy switching
- Parameter validation for each strategy
- Safety limit enforcement

#### **Lifecycle Management (8 tests)**
```bash
# Tests: on_activate, on_deactivate, state transitions
test_on_activate_success
test_on_deactivate_success
test_lifecycle_state_transitions
test_activation_error_handling
test_deactivation_cleanup
test_state_consistency_validation
test_emergency_stop_integration
test_lifecycle_error_recovery
```

**What these test**:
- Proper ROS2 hardware interface lifecycle
- State transition management
- Error handling during lifecycle changes
- Emergency stop integration with lifecycle

### Mock Framework Details

#### **MockBasicmicro Controller**
The unit tests use a comprehensive mock that simulates:
```python
# Controller methods mocked:
- ReadVersion() / GetVersion()  # Controller identification
- GetEncoders()                 # Position feedback
- GetSpeeds() / GetISpeeds()    # Velocity feedback  
- SpeedM1M2()                   # Speed control
- SpeedAccelM1M2()             # Speed with acceleration
- DutyM1M2()                   # Direct duty cycle control
- DutyAccelM1M2()              # Duty with acceleration
- ReadBuffers()                # Buffer status
- ForwardM1() / BackwardM1()   # Direction control
```

#### **Mock Behavior**
```python
# Realistic motor dynamics
def test_speed_control_with_mock():
    # Mock returns realistic encoder changes
    mock_controller.GetEncoders.return_value = (True, 1000, 2000)
    mock_controller.SpeedM1M2.return_value = True
    
    # Test validates both command and response
    result = hardware_interface.write(time=0.0, period=0.1)
    assert result == return_type.OK
    mock_controller.SpeedM1M2.assert_called_once()
```

### Expected Test Results

#### **All Tests Passing (Success Case)**
```bash
test/unit/test_hardware_interface_unit.py::TestInitialization::test_on_init_success PASSED
test/unit/test_hardware_interface_unit.py::TestInitialization::test_on_configure_success PASSED
test/unit/test_hardware_interface_unit.py::TestCommunication::test_write_success PASSED
test/unit/test_hardware_interface_unit.py::TestCommunication::test_read_success PASSED
test/unit/test_hardware_interface_unit.py::TestMotionControl::test_duty_motion_strategy PASSED
test/unit/test_hardware_interface_unit.py::TestMotionControl::test_speed_motion_strategy PASSED
test/unit/test_hardware_interface_unit.py::TestLifecycle::test_on_activate_success PASSED
test/unit/test_hardware_interface_unit.py::TestLifecycle::test_on_deactivate_success PASSED
...
================================= 32/32 passed =================================
```

#### **Failure Analysis**
If tests fail, common issues include:
```bash
# Import errors (environment not sourced)
ImportError: No module named 'basicmicro_driver'
# Solution: source install/setup.bash

# Mock setup issues
AttributeError: Mock object has no attribute 'return_value'
# Solution: Proper mock configuration in test setup

# API mismatches (code changes without test updates)
TypeError: method() missing 1 required positional argument
# Solution: Update test calls to match implementation API
```

## Service Unit Tests (29 tests)

### Test Categories

#### **Motion Configuration Service (5 tests)**
```bash
# Tests: Strategy setting, parameter validation, configuration retrieval
test_set_motion_strategy_success
test_invalid_strategy_handling
test_parameter_validation
test_configuration_retrieval
test_runtime_strategy_switching
```

**What these test**:
- Motion strategy selection and validation
- Parameter changes during runtime
- Configuration state management
- Error handling for invalid strategies

#### **Distance Movement Service (5 tests)**
```bash
# Tests: Distance-based movements, unit conversion, buffering
test_absolute_distance_movement
test_unit_conversion_accuracy
test_buffered_movement_execution
test_movement_parameter_validation
test_distance_limit_enforcement
```

**What these test**:
- Absolute distance movement commands
- Unit conversion from meters to encoder counts
- Buffer management for movement sequences
- Safety limit enforcement

#### **Trajectory Service (5 tests)**
```bash
# Tests: Multi-point trajectories, buffer management, execution monitoring
test_multi_point_trajectory_execution
test_buffer_overflow_handling
test_trajectory_parameter_validation
test_mixed_command_type_handling
test_trajectory_monitoring_integration
```

**What these test**:
- Complex trajectory execution with multiple waypoints
- Buffer overflow prevention and handling
- Mixed motion command types in sequences
- Integration with trajectory monitoring

#### **Servo Position Service (5 tests)**
```bash
# Tests: Absolute positioning, position hold, status monitoring
test_absolute_position_control
test_position_hold_release
test_servo_status_monitoring
test_position_limit_enforcement
test_homing_sequence_execution
```

**What these test**:
- Precise servo positioning control
- Position hold and release functionality
- Status monitoring and feedback
- Safety limit enforcement for positioning

#### **Duty Control Service (5 tests)**
```bash
# Tests: Direct PWM control, acceleration variants, range validation
test_direct_duty_control
test_duty_acceleration_control
test_duty_range_validation
test_duty_safety_limits
test_duty_emergency_stop
```

**What these test**:
- Direct PWM duty cycle control
- Acceleration-limited duty control
- Safety range validation
- Emergency stop integration

#### **Service Logic Testing (4 tests)**
```bash
# Tests: Multi-service coordination, error handling, state management
test_service_interaction_patterns
test_error_propagation_handling
test_service_state_consistency
test_concurrent_service_usage
```

**What these test**:
- Coordination between multiple services
- Error handling across service boundaries
- State consistency during multi-service operations
- Concurrent service usage patterns

### Mock Service Framework

#### **Service Mock Setup**
```python
# Services tested without ROS2 node instantiation
class TestServiceWithMocks(unittest.TestCase):
    def setUp(self):
        # Mock service without ROS2 initialization
        self.service = Mock()
        self.service.hardware_interface = Mock()
        self.service.unit_converter = UnitConverter(...)
        self.service.buffer_manager = Mock()
        
        # Configure mock behavior
        self.service.hardware_interface.controller.SpeedM1M2.return_value = True
```

#### **Request/Response Simulation**
```python
# Mock service requests and responses
class MockMoveDistanceRequest:
    def __init__(self, distance=1.0, max_speed=0.5):
        self.distance = distance
        self.max_speed = max_speed

class MockMoveDistanceResponse:
    def __init__(self):
        self.success = False
        self.message = ""
        self.estimated_duration = 0.0
```

### Expected Service Test Results

#### **All Service Tests Passing**
```bash
test/unit/test_service_unit.py::TestMotionConfigService::test_set_motion_strategy_success PASSED
test/unit/test_service_unit.py::TestMotionConfigService::test_invalid_strategy_handling PASSED
test/unit/test_service_unit.py::TestDistanceMovementService::test_absolute_distance_movement PASSED
test/unit/test_service_unit.py::TestDistanceMovementService::test_unit_conversion_accuracy PASSED
test/unit/test_service_unit.py::TestTrajectoryService::test_multi_point_trajectory_execution PASSED
test/unit/test_service_unit.py::TestTrajectoryService::test_buffer_overflow_handling PASSED
test/unit/test_service_unit.py::TestServoPositionService::test_absolute_position_control PASSED
test/unit/test_service_unit.py::TestServoPositionService::test_position_hold_release PASSED
test/unit/test_service_unit.py::TestDutyControlService::test_direct_duty_control PASSED
test/unit/test_service_unit.py::TestDutyControlService::test_duty_acceleration_control PASSED
...
================================= 29/29 passed =================================
```

## Test Framework Architecture

### Mock vs Real Implementation

#### **Unit Test Philosophy**
```python
# Unit tests validate logic, not communication
def test_distance_movement_logic():
    # Test the calculation and validation logic
    request = MockMoveDistanceRequest(distance=2.0, max_speed=0.5)
    
    # Mock hardware calls
    mock_hardware.SpeedAccelDistanceM1M2.return_value = True
    
    # Test service logic
    response = service.handle_distance_movement(request)
    
    # Validate calculation correctness
    assert response.success == True
    assert response.estimated_duration > 0
    mock_hardware.SpeedAccelDistanceM1M2.assert_called_once()
```

#### **Integration with Hardware Tests**
Unit tests complement hardware tests:
- **Unit tests**: Validate logic and error handling
- **Hardware tests**: Validate actual motor response
- **Integration tests**: Validate complete workflows

### Test Data and Validation

#### **Realistic Test Parameters**
```python
# Tests use realistic robotics parameters
test_distances = [0.1, 0.5, 1.0, 2.0]  # meters
test_speeds = [0.1, 0.3, 0.5, 1.0]     # m/s  
test_accelerations = [0.1, 0.5, 1.0]   # m/s²
test_positions = [0.0, 1.57, 3.14]     # radians

# Unit conversion validation
encoder_counts_per_meter = 1000
wheel_radius = 0.1  # meters
gear_ratio = 20.0
```

#### **Expected Value Validation**
```python
# Tests validate calculated values
def test_unit_conversion_accuracy():
    distance_m = 1.0  # meter
    expected_counts = distance_m * encoder_counts_per_meter
    
    actual_counts = unit_converter.meters_to_counts(distance_m)
    assert abs(actual_counts - expected_counts) < 1e-3
```

## Troubleshooting Unit Tests

### Common Issues

#### **Environment Setup**
```bash
# Verify environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Test import
python3 -c "import basicmicro_driver; print('Success')"
```

#### **Mock Framework Issues**
```python
# Common mock setup problems
# WRONG:
mock_method = Mock()
# Missing return_value setup

# RIGHT:
mock_method = Mock()
mock_method.return_value = True
# or
mock_method.return_value = (True, expected_value)
```

#### **Test Isolation**
```bash
# Run single test for debugging
python3 -m pytest test/unit/test_hardware_interface_unit.py::TestInitialization::test_on_init_success -v -s

# Run with debugging output
python3 -m pytest test/unit/ -v -s --tb=long
```

### Success Indicators

#### **Clean Test Run**
```bash
# All tests passing cleanly
========================================= PASSED =========================================
61 passed in 8.42s
```

#### **Performance Characteristics**
- **Fast execution**: 8-12 seconds for all 61 tests
- **Consistent results**: Same results across multiple runs
- **No hardware dependencies**: Tests run on any development machine
- **Clean teardown**: No hanging processes or resource leaks

The unit test suite provides comprehensive validation of all core components, enabling confident development and refactoring with immediate feedback on component functionality.