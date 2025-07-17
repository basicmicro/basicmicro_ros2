"""
Pytest Configuration and Shared Fixtures

Provides shared test fixtures and configuration for the basicmicro_driver test suite.
Supports both hardware-free testing with mocks and optional hardware-in-the-loop testing.

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import pytest
import sys
import os
from pathlib import Path
from unittest.mock import Mock, MagicMock
import tempfile
import shutil

# Add package to Python path
test_dir = Path(__file__).parent
package_root = test_dir.parent
sys.path.insert(0, str(package_root))

try:
    # Try to import ROS2 modules for integration tests
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# Import mock interface
from test_mocks.mock_basicmicro import MockBasicmicro, create_mock_controller


def pytest_addoption(parser):
    """Add command line options for testing"""
    parser.addoption("--hardware", action="store_true", help="Enable hardware tests")
    parser.addoption("--hardware-port", default="/dev/ttyACM1", help="Hardware port")
    parser.addoption("--hardware-baud", type=int, default=38400, help="Hardware baud rate")
    parser.addoption("--hardware-address", type=int, default=0x80, help="Hardware address")


def pytest_configure(config):
    """Configure pytest markers and options"""
    config.addinivalue_line(
        "markers", "ros2: Tests requiring ROS2 environment"
    )
    config.addinivalue_line(
        "markers", "hardware: Tests requiring actual hardware"
    )
    config.addinivalue_line(
        "markers", "performance: Performance and benchmarking tests"
    )
    config.addinivalue_line(
        "markers", "regression: Regression testing"
    )


def pytest_collection_modifyitems(config, items):
    """Automatically skip tests based on available dependencies"""
    if not ROS2_AVAILABLE:
        skip_ros2 = pytest.mark.skip(reason="ROS2 not available")
        for item in items:
            if "ros2" in item.keywords:
                item.add_marker(skip_ros2)
    
    # Skip hardware tests unless --hardware flag is provided
    if not config.getoption("--hardware", default=False):
        skip_hardware = pytest.mark.skip(reason="Hardware tests disabled (use --hardware to enable)")
        for item in items:
            if "hardware" in item.keywords:
                item.add_marker(skip_hardware)


# ============================================================================
# Core Test Fixtures
# ============================================================================

@pytest.fixture(scope="session")
def temp_workspace():
    """Create a temporary workspace for testing"""
    workspace = tempfile.mkdtemp(prefix="basicmicro_test_")
    yield Path(workspace)
    shutil.rmtree(workspace)


@pytest.fixture
def mock_basicmicro():
    """Create a mock Basicmicro controller for hardware-free testing"""
    return create_mock_controller("/dev/ttyACM0", 38400)


@pytest.fixture  
def mock_basicmicro_context():
    """Create a mock Basicmicro controller that can be used as context manager"""
    mock = create_mock_controller("/dev/ttyACM0", 38400)
    mock.Open.return_value = True
    mock.Close.return_value = True
    return mock


# ============================================================================
# ROS2 Test Fixtures (only when ROS2 is available)
# ============================================================================

if ROS2_AVAILABLE:
    @pytest.fixture
    def ros2_context():
        """Initialize ROS2 context for testing"""
        if not rclpy.ok():
            rclpy.init()
        yield
        if rclpy.ok():
            rclpy.shutdown()

    @pytest.fixture
    def test_node(ros2_context):
        """Create a test ROS2 node"""
        node = Node('test_node')
        yield node
        node.destroy_node()

    @pytest.fixture
    def executor(ros2_context):
        """Create a ROS2 executor for testing"""
        executor = SingleThreadedExecutor()
        yield executor
        executor.shutdown()

else:
    # Provide dummy fixtures when ROS2 is not available
    @pytest.fixture
    def ros2_context():
        pytest.skip("ROS2 not available")

    @pytest.fixture
    def test_node():
        pytest.skip("ROS2 not available")

    @pytest.fixture
    def executor():
        pytest.skip("ROS2 not available")


# ============================================================================
# Hardware Interface Test Fixtures
# ============================================================================

@pytest.fixture
def mock_hardware_info():
    """Create mock HardwareInfo for testing hardware interface"""
    info = MagicMock()
    info.hardware_parameters = {
        'port': '/dev/ttyACM0',
        'baud': '38400',
        'address': '128',
        'wheel_radius': '0.1',
        'wheel_separation': '0.3',
        'encoder_counts_per_rev': '1000',
        'gear_ratio': '1.0',
        'motion_strategy': 'speed_accel',
        'buffer_depth': '4',
        'default_acceleration': '1000',
        'encoder_type': 'incremental',
        'auto_home_on_startup': 'false',
        'position_limits_enabled': 'false'
    }
    info.joints = [MagicMock(), MagicMock()]
    info.joints[0].name = 'left_wheel_joint'
    info.joints[1].name = 'right_wheel_joint'
    return info


@pytest.fixture
def unit_converter():
    """Create a unit converter for testing"""
    from basicmicro_driver.unit_converter import UnitConverter
    return UnitConverter(
        wheel_radius=0.1,
        encoder_counts_per_rev=1000,
        gear_ratio=1.0
    )


# ============================================================================
# Service Test Fixtures
# ============================================================================

@pytest.fixture
def mock_service_response():
    """Create mock service response objects"""
    def _create_response(service_type, **kwargs):
        response = MagicMock()
        for key, value in kwargs.items():
            setattr(response, key, value)
        return response
    return _create_response


# ============================================================================
# Performance Test Fixtures
# ============================================================================

@pytest.fixture
def performance_timer():
    """Performance timing fixture for benchmarking tests"""
    import time
    
    class Timer:
        def __init__(self):
            self.start_time = None
            self.end_time = None
            
        def start(self):
            self.start_time = time.perf_counter()
            
        def stop(self):
            self.end_time = time.perf_counter()
            
        @property
        def elapsed(self):
            if self.start_time is None or self.end_time is None:
                return None
            return self.end_time - self.start_time
            
        def __enter__(self):
            self.start()
            return self
            
        def __exit__(self, *args):
            self.stop()
    
    return Timer()


@pytest.fixture
def memory_profiler():
    """Memory profiling fixture for performance tests"""
    try:
        import psutil
        import os
        
        class MemoryProfiler:
            def __init__(self):
                self.process = psutil.Process(os.getpid())
                self.initial_memory = None
                self.peak_memory = None
                
            def start(self):
                self.initial_memory = self.process.memory_info().rss
                self.peak_memory = self.initial_memory
                
            def update(self):
                current_memory = self.process.memory_info().rss
                if current_memory > self.peak_memory:
                    self.peak_memory = current_memory
                    
            def get_usage(self):
                if self.initial_memory is None:
                    return None
                current_memory = self.process.memory_info().rss
                return {
                    'initial_mb': self.initial_memory / 1024 / 1024,
                    'current_mb': current_memory / 1024 / 1024,
                    'peak_mb': self.peak_memory / 1024 / 1024,
                    'delta_mb': (current_memory - self.initial_memory) / 1024 / 1024
                }
        
        profiler = MemoryProfiler()
        profiler.start()
        yield profiler
        
    except ImportError:
        # Fallback when psutil is not available
        yield None


# ============================================================================
# Test Utilities
# ============================================================================

@pytest.fixture
def test_data_dir():
    """Path to test data directory"""
    return test_dir / "test_data"


@pytest.fixture
def sample_configurations():
    """Sample robot configurations for testing"""
    return {
        'differential_drive': {
            'wheel_radius': 0.1,
            'wheel_separation': 0.3,
            'encoder_counts_per_rev': 1000,
            'gear_ratio': 1.0,
            'max_speed': 2.0
        },
        'industrial': {
            'wheel_radius': 0.2,
            'wheel_separation': 0.6,
            'encoder_counts_per_rev': 2000,
            'gear_ratio': 10.0,
            'max_speed': 5.0
        },
        'multi_controller': {
            'wheel_radius': 0.15,
            'wheel_separation': 0.5,
            'encoder_counts_per_rev': 1500,
            'gear_ratio': 5.0,
            'max_speed': 3.0,
            'num_controllers': 2
        }
    }


# ============================================================================
# Validation Utilities
# ============================================================================

def assert_approximately_equal(actual, expected, tolerance=1e-6, msg=""):
    """Assert that two values are approximately equal within tolerance"""
    if abs(actual - expected) > tolerance:
        pytest.fail(
            f"{msg}Expected {expected}, got {actual}, "
            f"difference {abs(actual - expected)} > tolerance {tolerance}"
        )


def assert_valid_ros2_message(msg, message_type):
    """Assert that a message is a valid ROS2 message of the expected type"""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available for message validation")
    
    assert isinstance(msg, message_type), f"Expected {message_type}, got {type(msg)}"


def assert_performance_within_limits(elapsed_time, max_time, operation_name="operation"):
    """Assert that an operation completed within performance limits"""
    assert elapsed_time <= max_time, (
        f"{operation_name} took {elapsed_time:.4f}s, "
        f"exceeding limit of {max_time:.4f}s"
    )


# ============================================================================
# Test Markers and Skip Conditions
# ============================================================================

# Skip conditions for different test types
requires_ros2 = pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
requires_hardware = pytest.mark.skip(reason="Hardware tests disabled by default")


# Performance test configuration
PERFORMANCE_LIMITS = {
    'command_latency_ms': 5.0,
    'sensor_read_frequency_hz': 100.0,
    'memory_usage_mb': 100.0,
    'cpu_usage_percent': 50.0
}