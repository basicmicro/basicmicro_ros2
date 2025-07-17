# Testing Overview

The Basicmicro ROS2 driver includes a comprehensive test suite designed to validate functionality across multiple levels of integration, from individual components to complete hardware workflows.

## Test Categories

### Unit Tests (61 tests)
**Purpose**: Validate individual component functionality in isolation
- **Hardware Interface Tests** (32 tests): Core hardware abstraction functionality
- **Service Unit Tests** (29 tests): Individual service logic without ROS2 infrastructure
- **Mock-based**: No hardware or ROS2 nodes required
- **Fast execution**: Complete in seconds

### Integration Tests (14 tests)  
**Purpose**: Validate component interactions and complete workflows
- **Complete Motion Workflows**: Full command-to-execution cycles
- **ROS2 Control Integration**: Hardware interface lifecycle management
- **Service Interaction**: Multi-service coordination patterns
- **Parameter Consistency**: Configuration propagation validation

### Hardware Tests
**Purpose**: Validate real hardware communication and control
- **Basic Communication**: Controller version, status, connectivity
- **Motor Control Validation**: Actual movement with encoder feedback
- **Safety Systems**: Emergency stop and limit enforcement
- **Requires**: Physical Basicmicro controller connection

### Performance Tests
**Purpose**: Measure and validate system performance characteristics
- **Buffer Performance**: Command queuing and execution efficiency
- **Command Latency**: Time from ROS2 command to motor response
- **Throughput Validation**: High-frequency command processing
- **Real-time Constraints**: Timing requirement verification

## Testing Philosophy

### Test-Driven Quality Assurance
The test suite validates that the driver:
1. **Communicates reliably** with Basicmicro controllers
2. **Integrates properly** with ROS2 ecosystem standards
3. **Performs efficiently** under real-world robotics workloads
4. **Handles errors gracefully** with appropriate recovery

### Hardware-First Validation
Critical functionality is validated with actual hardware:
- **Real motor response**: Not just communication success
- **Encoder accuracy**: Position and velocity feedback validation  
- **Buffer utilization**: Actual trajectory execution performance
- **Safety enforcement**: Physical emergency stop verification

### Mock Framework for Development
Comprehensive mock system enables:
- **Hardware-free development**: Work without physical controllers
- **Reproducible testing**: Consistent test environment
- **Error simulation**: Test failure scenarios safely
- **CI/CD integration**: Automated testing without hardware

## Prerequisites

### Basic Testing (Unit + Integration)
```bash
# ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build and source workspace
cd /path/to/workspace
colcon build --packages-select basicmicro_driver
source install/setup.bash

# Python testing framework
pip install pytest pytest-mock
```

### Hardware Testing
```bash
# Additional requirements for hardware tests
pip install pyserial

# Hardware connection
# - Basicmicro controller connected via USB
# - Controller configured in Motion Studio
# - Device accessible (typically /dev/ttyACM0 or /dev/ttyACM1)
```

### Performance Testing
```bash
# ROS2 initialization (for Node-based performance tests)
# Ensure ROS2 environment is properly sourced
source /opt/ros/jazzy/setup.bash

# Hardware controller with adequate performance
# - RoboClaw 2x15A or higher recommended
# - Encoders connected for position feedback
# - Motors with sufficient load for realistic testing
```

## Running Tests

### Quick Test Suite (Unit + Integration)
```bash
# Run core functionality tests (recommended for development)
python3 -m pytest test/unit/ test/integration/ -v

# Expected output:
# test/unit/ - 61/61 PASSED
# test/integration/ - 14/14 PASSED  
# Total: 75/75 tests passing (100% success rate)
```

### Hardware Validation
```bash
# Basic hardware communication
python3 -m pytest test/hardware/ --hardware --hardware-port=/dev/ttyACM1 -v

# With specific controller settings
python3 -m pytest test/hardware/ --hardware --hardware-port=/dev/ttyACM1 --hardware-baud=38400 --hardware-address=128 -v
```

### Performance Benchmarking
```bash
# Performance validation with hardware
python3 -m pytest test/performance/ --hardware --hardware-port=/dev/ttyACM1 -v

# Results include:
# - Buffer utilization percentages
# - Command latency measurements  
# - Trajectory execution accuracy
# - Real-time performance metrics
```

### Complete Test Suite
```bash
# Run all tests (requires hardware connection)
python3 -m pytest test/ --hardware --hardware-port=/dev/ttyACM1 -v

# Comprehensive validation:
# - All unit functionality
# - Complete integration workflows
# - Hardware communication
# - Performance characteristics
```

## Test Environment Setup

### Development Environment
**For code development without hardware:**
1. Source ROS2 environment
2. Build and source workspace
3. Run unit and integration tests
4. Use mock framework for component development

### Hardware Integration Environment  
**For hardware validation and performance testing:**
1. Connect Basicmicro controller via USB
2. Configure controller in Motion Studio
3. Verify device permissions and accessibility
4. Run hardware and performance test suites

### Continuous Integration Environment
**For automated testing (no hardware):**
1. ROS2 environment setup
2. Package dependency installation
3. Unit and integration test execution
4. Mock framework validation

## Expected Results

### Success Criteria
- **Unit Tests**: 61/61 passing (100% success rate)
- **Integration Tests**: 14/14 passing (100% success rate)  
- **Hardware Tests**: All communication and control tests passing
- **Performance Tests**: Metrics within expected ranges

### Test Output Interpretation
```bash
# Successful test run example:
collected 75 items

test/unit/test_hardware_interface_unit.py::TestHardwareInterface::test_write_success PASSED
test/unit/test_hardware_interface_unit.py::TestHardwareInterface::test_read_success PASSED
...
test/integration/test_complete_workflow_integration.py::TestCompleteWorkflow::test_initialization_sequence PASSED

========== 75 passed in 12.34s ==========
```

### Common Test Failures
1. **Import Errors**: Environment not properly sourced
2. **Hardware Connection**: Device not accessible or configured
3. **Permission Errors**: User not in dialout group (Linux)
4. **Timing Issues**: Hardware not responding within expected timeframes

## Troubleshooting

### Environment Issues
```bash
# Verify package installation
python3 -c "import basicmicro_driver; print('Package imported successfully')"

# Check service generation
python3 -c "from basicmicro_driver.srv import MoveDistance; print('Services available')"
```

### Hardware Connection Issues
```bash
# Test basic controller connection
python3 -c "
from basicmicro import Basicmicro
controller = Basicmicro('/dev/ttyACM1', 38400)
print('Connected:', controller.Open())
if controller.Open():
    version = controller.ReadVersion(0x80)
    print('Version:', version[1] if version[0] else 'Failed')
"
```

### Permission Issues (Linux)
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Apply group changes (logout/login or use newgrp)
newgrp dialout

# Verify device access
ls -la /dev/ttyACM*
```

## Integration with Development Workflow

### Pre-commit Testing
```bash
# Quick validation before commits
python3 -m pytest test/unit/ test/integration/ --tb=short
```

### Feature Development Testing
```bash
# Test specific component during development
python3 -m pytest test/unit/test_hardware_interface_unit.py -v
python3 -m pytest test/integration/test_complete_workflow_integration.py -v
```

### Hardware Validation Testing
```bash
# Validate hardware integration after changes
python3 -m pytest test/hardware/ --hardware --hardware-port=/dev/ttyACM1 --tb=short
```

### Performance Regression Testing
```bash
# Check performance impact of changes
python3 -m pytest test/performance/ --hardware --hardware-port=/dev/ttyACM1 --tb=short
```

This testing framework provides confidence that the Basicmicro ROS2 driver functions correctly across all levels of integration, from individual components to complete robotics applications with real hardware.