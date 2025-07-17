# Testing Framework Structure

This document outlines the comprehensive testing framework for the basicmicro_driver package. The framework supports multiple test categories and environments.

## Test Categories

### 1. Unit Tests (`test_*_unit.py`)
- Test individual components in isolation
- Use mock interfaces for hardware dependencies
- Fast execution, no external dependencies
- Target: >95% code coverage

### 2. Integration Tests (`test_*_integration.py`)
- Test component interactions and workflows
- End-to-end functionality testing
- Service interface testing
- ROS2 message flow validation

### 3. Performance Tests (`test_*_performance.py`)
- Benchmark critical operations
- Latency and throughput testing
- Memory usage validation
- CPU utilization monitoring

### 4. Regression Tests (`test_*_regression.py`)
- Ensure no functionality is broken
- Version compatibility testing
- Parameter validation across updates
- Backwards compatibility verification

### 5. Hardware-in-the-Loop Tests (`test_*_hardware.py`)
- Optional tests with actual hardware
- Real controller communication testing
- Physical parameter validation
- Disabled by default, enabled with `--hardware` flag

## Test Structure

```
test/
├── conftest.py                    # Shared fixtures and configuration
├── test_framework_structure.md   # This document
├── test_runner.py                # Custom test runner with reporting
├── test_data/                    # Test data files
│   ├── sample_configurations/
│   └── mock_responses/
├── unit/                         # Unit tests
│   ├── test_hardware_interface_unit.py
│   ├── test_unit_converter_unit.py
│   ├── test_services_unit.py
│   └── test_buffer_management_unit.py
├── integration/                  # Integration tests
│   ├── test_complete_workflow_integration.py
│   ├── test_service_interactions_integration.py
│   ├── test_ros2_control_integration.py
│   └── test_launch_system_integration.py
├── performance/                  # Performance tests
│   ├── test_command_latency_performance.py
│   ├── test_sensor_throughput_performance.py
│   ├── test_memory_usage_performance.py
│   └── test_concurrent_operations_performance.py
├── regression/                   # Regression tests
│   ├── test_api_compatibility_regression.py
│   ├── test_parameter_validation_regression.py
│   ├── test_functionality_regression.py
│   └── test_performance_regression.py
└── hardware/                     # Hardware-in-the-loop tests
    ├── test_real_controller_hardware.py
    ├── test_servo_functionality_hardware.py
    └── test_error_conditions_hardware.py
```

## Test Execution

### Run All Tests
```bash
pytest test/ -v
```

### Run by Category
```bash
# Unit tests only
pytest test/unit/ -v

# Integration tests (requires ROS2)
pytest test/integration/ -v -m ros2

# Performance tests
pytest test/performance/ -v -m performance

# Regression tests
pytest test/regression/ -v -m regression
```

### Run with Coverage
```bash
pytest test/ --cov=basicmicro_driver --cov-report=html
```

### Hardware Tests (Optional)
```bash
pytest test/hardware/ -v -m hardware --hardware-port=/dev/ttyACM0
```

## Test Configuration

### Markers
- `@pytest.mark.unit`: Unit tests
- `@pytest.mark.integration`: Integration tests
- `@pytest.mark.performance`: Performance tests
- `@pytest.mark.regression`: Regression tests
- `@pytest.mark.hardware`: Hardware-in-the-loop tests
- `@pytest.mark.ros2`: Tests requiring ROS2
- `@pytest.mark.slow`: Slow-running tests

### Environment Variables
- `BASICMICRO_TEST_HARDWARE`: Enable hardware tests
- `BASICMICRO_TEST_PORT`: Hardware port for testing
- `BASICMICRO_TEST_TIMEOUT`: Test timeout in seconds
- `BASICMICRO_PERFORMANCE_LIMITS`: JSON file with performance limits

### Performance Limits
Default performance targets:
- Command latency: <5ms
- Sensor reading: >100Hz
- Memory usage: <100MB
- CPU usage: <50%

## Mock Interfaces

### MockBasicmicro
Enhanced mock controller with:
- Realistic timing simulation
- Error condition simulation
- Buffer management simulation
- Controller type simulation (RoboClaw vs MCP)
- Deterministic responses for testing

### Mock ROS2 Services
- Service call simulation
- Parameter server simulation
- Topic publishing/subscription simulation
- Lifecycle management simulation

## Test Data Management

### Configuration Files
- Sample robot configurations
- URDF templates for testing
- Launch file variations
- Parameter sets for different scenarios

### Mock Response Data
- Realistic sensor readings
- Error condition responses
- Performance timing data
- Buffer status sequences

## Continuous Integration

### GitHub Actions Support
```yaml
- name: Run Tests
  run: |
    pytest test/unit/ test/integration/ -v --cov=basicmicro_driver
    pytest test/performance/ -v -m "not slow"
    pytest test/regression/ -v
```

### Test Reporting
- JUnit XML output for CI integration
- Coverage reports (HTML and XML)
- Performance trend tracking
- Test timing analysis

## Quality Metrics

### Coverage Targets
- Unit tests: >95% line coverage
- Integration tests: >90% workflow coverage
- Performance tests: All critical paths covered
- Regression tests: All public APIs covered

### Performance Benchmarks
- Baseline measurements for all operations
- Regression detection (>10% performance degradation)
- Memory leak detection (long-running tests)
- Resource usage monitoring

## Best Practices

### Test Organization
1. One test class per component
2. Descriptive test names
3. Arrange-Act-Assert pattern
4. Proper test isolation
5. Comprehensive error testing

### Mock Usage
1. Use fixtures for consistent mocks
2. Simulate realistic behavior
3. Test both success and failure paths
4. Validate mock interactions
5. Keep mocks simple and focused

### Performance Testing
1. Establish baselines first
2. Test under realistic conditions
3. Include stress testing
4. Monitor resource usage
5. Automate performance validation

### Hardware Testing
1. Provide clear setup instructions
2. Test graceful degradation
3. Validate error recovery
4. Document hardware requirements
5. Make hardware tests optional

This framework ensures comprehensive testing coverage while supporting both development and production environments.