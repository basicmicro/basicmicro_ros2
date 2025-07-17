"""
Test suite for BasicMicro Hardware Interface

Tests the ros2_control hardware interface implementation including:
- Parameter extraction and validation
- Controller connection and type detection
- Motion strategy execution
- Unit conversions
- Emergency stop functionality
- Servo capabilities and configuration
"""

import pytest
import sys
import os
from unittest.mock import Mock, patch, MagicMock

# Add the package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from basicmicro_driver.hardware_interface import (
    BasicmicroHardwareInterface,
    ControllerType,
    EncoderType,
    MotionStrategy,
    LimitViolationBehavior
)


class MockHardwareInfo:
    """Mock HardwareInfo for testing"""
    def __init__(self):
        self.hardware_parameters = {}


@pytest.fixture
def hardware_interface():
    """Create a BasicMicro hardware interface instance for testing"""
    return BasicmicroHardwareInterface()


@pytest.fixture
def initialized_hardware_interface():
    """Create a hardware interface with properly initialized unit converter"""
    hw = BasicmicroHardwareInterface()
    
    # Set default parameters
    hw.wheel_radius = 0.1
    hw.encoder_counts_per_rev = 1000  
    hw.gear_ratio = 1.0
    
    # Initialize unit converter for testing
    from basicmicro_driver.unit_converter import UnitConverter
    hw.unit_converter = UnitConverter(
        wheel_radius=hw.wheel_radius,
        encoder_counts_per_rev=hw.encoder_counts_per_rev,
        gear_ratio=hw.gear_ratio
    )
    return hw


@pytest.fixture
def mock_info():
    """Create mock hardware info for testing"""
    return MockHardwareInfo()


@pytest.fixture
def mock_controller():
    """Create a mock BasicMicro controller for testing"""
    from test_mocks.mock_basicmicro import create_mock_controller
    return create_mock_controller()


class TestHardwareInterfaceInitialization:
    """Test hardware interface initialization and configuration"""
    
    def test_init_default_values(self, hardware_interface):
        """Test that hardware interface initializes with correct default values"""
        assert hardware_interface.port == "/dev/ttyACM0"
        assert hardware_interface.baud == 38400
        assert hardware_interface.address == 0x80
        assert hardware_interface.wheel_radius == 0.1
        assert hardware_interface.wheel_separation == 0.3
        assert hardware_interface.encoder_counts_per_rev == 1000
        assert hardware_interface.gear_ratio == 1.0
        assert hardware_interface.motion_strategy == MotionStrategy.SPEED_ACCEL
        assert hardware_interface.buffer_depth == 4
        assert hardware_interface.default_acceleration == 1000
        assert hardware_interface.encoder_type == EncoderType.INCREMENTAL
        assert hardware_interface.auto_home_on_startup is False
        assert hardware_interface.position_limits_enabled is False
        assert hardware_interface.limit_violation_behavior == LimitViolationBehavior.SOFT_STOP
        assert hardware_interface.controller_type == ControllerType.UNKNOWN
        assert hardware_interface.servo_capable is False
        assert hardware_interface.emergency_stop_active is False

    def test_joint_names(self, hardware_interface):
        """Test that joint names are correctly set"""
        assert len(hardware_interface.joint_names) == 2
        assert hardware_interface.joint_names[0] == "left_wheel_joint"
        assert hardware_interface.joint_names[1] == "right_wheel_joint"

    def test_state_command_interfaces_initialization(self, hardware_interface):
        """Test that state and command interfaces are initialized"""
        assert len(hardware_interface.hw_commands_positions_) == 2
        assert len(hardware_interface.hw_commands_velocities_) == 2
        assert len(hardware_interface.hw_states_positions_) == 2
        assert len(hardware_interface.hw_states_velocities_) == 2
        
        # All should be initialized to zero
        assert all(x == 0.0 for x in hardware_interface.hw_commands_positions_)
        assert all(x == 0.0 for x in hardware_interface.hw_commands_velocities_)
        assert all(x == 0.0 for x in hardware_interface.hw_states_positions_)
        assert all(x == 0.0 for x in hardware_interface.hw_states_velocities_)


class TestParameterValidation:
    """Test parameter extraction and validation"""
    
    def test_validate_parameters_success(self, hardware_interface):
        """Test parameter validation with valid parameters"""
        assert hardware_interface._validate_parameters() is True

    def test_validate_parameters_invalid_wheel_radius(self, hardware_interface):
        """Test parameter validation with invalid wheel radius"""
        hardware_interface.wheel_radius = -0.1
        assert hardware_interface._validate_parameters() is False
        
        hardware_interface.wheel_radius = 0.0
        assert hardware_interface._validate_parameters() is False

    def test_validate_parameters_invalid_wheel_separation(self, hardware_interface):
        """Test parameter validation with invalid wheel separation"""
        hardware_interface.wheel_separation = -0.1
        assert hardware_interface._validate_parameters() is False
        
        hardware_interface.wheel_separation = 0.0
        assert hardware_interface._validate_parameters() is False

    def test_validate_parameters_invalid_encoder_counts(self, hardware_interface):
        """Test parameter validation with invalid encoder counts"""
        hardware_interface.encoder_counts_per_rev = -1000
        assert hardware_interface._validate_parameters() is False
        
        hardware_interface.encoder_counts_per_rev = 0
        assert hardware_interface._validate_parameters() is False

    def test_validate_parameters_invalid_gear_ratio(self, hardware_interface):
        """Test parameter validation with invalid gear ratio"""
        hardware_interface.gear_ratio = -1.0
        assert hardware_interface._validate_parameters() is False
        
        hardware_interface.gear_ratio = 0.0
        assert hardware_interface._validate_parameters() is False

    def test_validate_parameters_invalid_address(self, hardware_interface):
        """Test parameter validation with invalid address"""
        hardware_interface.address = 0x79  # Below valid range
        assert hardware_interface._validate_parameters() is False
        
        hardware_interface.address = 0x88  # Above valid range
        assert hardware_interface._validate_parameters() is False

    def test_extract_connection_parameters(self, hardware_interface, mock_info):
        """Test extraction of connection parameters"""
        # Mock the _get_parameter method to return test values
        with patch.object(hardware_interface, '_get_parameter') as mock_get:
            mock_get.side_effect = lambda info, name, default: {
                'port': '/dev/ttyUSB0',
                'baud': '115200',
                'address': '0x81'
            }.get(name, default)
            
            hardware_interface._extract_connection_parameters(mock_info)
            
            assert hardware_interface.port == '/dev/ttyUSB0'
            assert hardware_interface.baud == 115200
            assert hardware_interface.address == 0x81

    def test_extract_connection_parameters_decimal_address(self, hardware_interface, mock_info):
        """Test extraction of connection parameters with decimal address"""
        with patch.object(hardware_interface, '_get_parameter') as mock_get:
            mock_get.side_effect = lambda info, name, default: {
                'address': '129'  # Decimal equivalent of 0x81
            }.get(name, default)
            
            hardware_interface._extract_connection_parameters(mock_info)
            assert hardware_interface.address == 129

    def test_extract_physical_parameters(self, hardware_interface, mock_info):
        """Test extraction of physical parameters"""
        with patch.object(hardware_interface, '_get_parameter') as mock_get:
            mock_get.side_effect = lambda info, name, default: {
                'wheel_radius': '0.15',
                'wheel_separation': '0.4',
                'encoder_counts_per_rev': '2000',
                'gear_ratio': '2.5'
            }.get(name, default)
            
            hardware_interface._extract_physical_parameters(mock_info)
            
            assert hardware_interface.wheel_radius == 0.15
            assert hardware_interface.wheel_separation == 0.4
            assert hardware_interface.encoder_counts_per_rev == 2000
            assert hardware_interface.gear_ratio == 2.5

    def test_extract_motion_parameters(self, hardware_interface, mock_info):
        """Test extraction of motion parameters"""
        with patch.object(hardware_interface, '_get_parameter') as mock_get:
            mock_get.side_effect = lambda info, name, default: {
                'motion_strategy': 'duty',
                'buffer_depth': '8',
                'default_acceleration': '2000'
            }.get(name, default)
            
            hardware_interface._extract_motion_parameters(mock_info)
            
            assert hardware_interface.motion_strategy == MotionStrategy.DUTY
            assert hardware_interface.buffer_depth == 8
            assert hardware_interface.default_acceleration == 2000

    def test_extract_servo_parameters(self, hardware_interface, mock_info):
        """Test extraction of servo parameters"""
        with patch.object(hardware_interface, '_get_parameter') as mock_get:
            mock_get.side_effect = lambda info, name, default: {
                'encoder_type': 'absolute',
                'auto_home_on_startup': 'true',
                'position_limits_enabled': 'true',
                'min_position_left': '-500000.0',
                'max_position_left': '500000.0',
                'min_position_right': '-600000.0',
                'max_position_right': '600000.0',
                'limit_violation_behavior': 'hard_stop',
                'limit_decel_rate': '1500.0'
            }.get(name, default)
            
            hardware_interface._extract_servo_parameters(mock_info)
            
            assert hardware_interface.encoder_type == EncoderType.ABSOLUTE
            assert hardware_interface.auto_home_on_startup is True
            assert hardware_interface.position_limits_enabled is True
            assert hardware_interface.min_position_left == -500000.0
            assert hardware_interface.max_position_left == 500000.0
            assert hardware_interface.min_position_right == -600000.0
            assert hardware_interface.max_position_right == 600000.0
            assert hardware_interface.limit_violation_behavior == LimitViolationBehavior.HARD_STOP
            assert hardware_interface.limit_decel_rate == 1500.0


class TestControllerConnection:
    """Test controller connection and type detection"""
    
    @patch('basicmicro_driver.hardware_interface.Basicmicro')
    def test_establish_connection_success(self, mock_basicmicro_class, hardware_interface):
        """Test successful controller connection"""
        mock_controller = Mock()
        mock_controller.ReadVersion.return_value = (True, "RoboClaw 2x15A v4.1.34")
        mock_basicmicro_class.return_value = mock_controller
        
        hardware_interface.controller = mock_controller
        
        result = hardware_interface._establish_connection()
        
        assert result is True
        mock_controller.Open.assert_called_once()
        mock_controller.ReadVersion.assert_called_once_with(hardware_interface.address)

    @patch('basicmicro_driver.hardware_interface.Basicmicro')
    def test_establish_connection_failure(self, mock_basicmicro_class, hardware_interface):
        """Test failed controller connection"""
        mock_controller = Mock()
        mock_controller.ReadVersion.return_value = (False, "")
        mock_basicmicro_class.return_value = mock_controller
        
        hardware_interface.controller = mock_controller
        
        result = hardware_interface._establish_connection()
        
        assert result is False

    def test_detect_controller_type_roboclaw(self, hardware_interface):
        """Test RoboClaw controller type detection"""
        mock_controller = Mock()
        mock_controller.ReadVersion.return_value = (True, "RoboClaw 2x15A v4.1.34")
        hardware_interface.controller = mock_controller
        
        hardware_interface._detect_controller_type()
        
        assert hardware_interface.controller_type == ControllerType.ROBOCLAW
        assert hardware_interface.servo_capable is True

    def test_detect_controller_type_mcp(self, hardware_interface):
        """Test MCP controller type detection"""
        mock_controller = Mock()
        mock_controller.ReadVersion.return_value = (True, "MCP Advanced Motor Controller v2.3.1")
        hardware_interface.controller = mock_controller
        
        hardware_interface._detect_controller_type()
        
        assert hardware_interface.controller_type == ControllerType.MCP
        assert hardware_interface.servo_capable is True

    def test_detect_controller_type_unknown(self, hardware_interface):
        """Test unknown controller type detection"""
        mock_controller = Mock()
        mock_controller.ReadVersion.return_value = (True, "Unknown Controller v1.0.0")
        hardware_interface.controller = mock_controller
        
        hardware_interface._detect_controller_type()
        
        assert hardware_interface.controller_type == ControllerType.UNKNOWN
        assert hardware_interface.servo_capable is False

    def test_detect_controller_type_failure(self, hardware_interface):
        """Test controller type detection failure"""
        mock_controller = Mock()
        mock_controller.ReadVersion.return_value = (False, "")
        hardware_interface.controller = mock_controller
        
        hardware_interface._detect_controller_type()
        
        assert hardware_interface.controller_type == ControllerType.UNKNOWN
        assert hardware_interface.servo_capable is False


class TestUnitConversions:
    """Test unit conversion integration with hardware interface"""
    
    def test_unit_converter_integration(self, initialized_hardware_interface):
        """Test that hardware interface properly uses unit converter"""
        hw = initialized_hardware_interface
        
        # Test that unit converter is properly initialized
        assert hw.unit_converter is not None
        assert hw.unit_converter.wheel_radius == 0.1
        assert hw.unit_converter.encoder_counts_per_rev == 1000
        assert hw.unit_converter.gear_ratio == 1.0
        
        # Test basic conversions through the unit converter
        result = hw.unit_converter.counts_to_radians(1000)
        expected = 2.0 * 3.14159265359  # 2π
        assert abs(result - expected) < 1e-6
        
        result = hw.unit_converter.radians_to_counts(2.0 * 3.14159265359)
        assert result == 1000

    def test_unit_converter_with_gear_ratio(self, initialized_hardware_interface):
        """Test unit converter with different gear ratios"""
        hw = initialized_hardware_interface
        
        # Create new converter with gear ratio 2.0
        from basicmicro_driver.unit_converter import UnitConverter
        hw.unit_converter = UnitConverter(
            wheel_radius=0.1,
            encoder_counts_per_rev=1000,
            gear_ratio=2.0
        )
        
        # With gear ratio 2.0, one wheel revolution = 2000 encoder counts
        result = hw.unit_converter.counts_to_radians(2000)
        expected = 2.0 * 3.14159265359  # 2π
        assert abs(result - expected) < 1e-6
        
        # Reverse conversion
        result = hw.unit_converter.radians_to_counts(2.0 * 3.14159265359)
        assert result == 2000


class TestEmergencyStop:
    """Test emergency stop functionality"""
    
    def test_emergency_stop_success(self, hardware_interface):
        """Test successful emergency stop"""
        mock_controller = Mock()
        hardware_interface.controller = mock_controller
        
        result = hardware_interface.emergency_stop()
        
        assert result is True
        assert hardware_interface.emergency_stop_active is True
        mock_controller.DutyM1M2.assert_called_once_with(hardware_interface.address, 0, 0)

    def test_emergency_stop_no_controller(self, hardware_interface):
        """Test emergency stop with no controller"""
        hardware_interface.controller = None
        
        result = hardware_interface.emergency_stop()
        
        assert result is False

    def test_emergency_stop_controller_failure(self, hardware_interface):
        """Test emergency stop with controller communication failure"""
        mock_controller = Mock()
        mock_controller.DutyM1M2.side_effect = Exception("Communication error")
        hardware_interface.controller = mock_controller
        
        result = hardware_interface.emergency_stop()
        
        assert result is False


class TestMotionStrategy:
    """Test motion strategy execution"""
    
    def test_execute_velocity_command_duty(self, initialized_hardware_interface):
        """Test velocity command execution with duty strategy"""
        mock_controller = Mock()
        initialized_hardware_interface.controller = mock_controller
        initialized_hardware_interface.motion_strategy = MotionStrategy.DUTY
        
        result = initialized_hardware_interface._execute_velocity_command(1.0, -1.0)
        
        assert result == 0  # return_type.OK
        mock_controller.DutyM1M2.assert_called_once()

    def test_execute_velocity_command_duty_accel(self, initialized_hardware_interface):
        """Test velocity command execution with duty acceleration strategy"""
        mock_controller = Mock()
        initialized_hardware_interface.controller = mock_controller
        initialized_hardware_interface.motion_strategy = MotionStrategy.DUTY_ACCEL
        
        result = initialized_hardware_interface._execute_velocity_command(1.0, -1.0)
        
        assert result == 0  # return_type.OK
        mock_controller.DutyAccelM1M2.assert_called_once()

    def test_execute_velocity_command_speed(self, initialized_hardware_interface):
        """Test velocity command execution with speed strategy"""
        mock_controller = Mock()
        initialized_hardware_interface.controller = mock_controller
        initialized_hardware_interface.motion_strategy = MotionStrategy.SPEED
        
        result = initialized_hardware_interface._execute_velocity_command(1.0, -1.0)
        
        assert result == 0  # return_type.OK
        mock_controller.SpeedM1M2.assert_called_once()

    def test_execute_velocity_command_speed_accel(self, initialized_hardware_interface):
        """Test velocity command execution with speed acceleration strategy"""
        mock_controller = Mock()
        initialized_hardware_interface.controller = mock_controller
        initialized_hardware_interface.motion_strategy = MotionStrategy.SPEED_ACCEL
        
        result = initialized_hardware_interface._execute_velocity_command(1.0, -1.0)
        
        assert result == 0  # return_type.OK
        mock_controller.SpeedAccelM1M2.assert_called_once()


class TestReadWriteOperations:
    """Test read and write operations"""
    
    def test_read_success(self, initialized_hardware_interface):
        """Test successful sensor reading"""
        mock_controller = Mock()
        mock_controller.GetEncoders.return_value = (True, 1000, 1500)
        mock_controller.GetSpeeds.return_value = (True, 500, -300)
        initialized_hardware_interface.controller = mock_controller
        
        result = initialized_hardware_interface.read(None, None)
        
        assert result == 0  # return_type.OK
        assert initialized_hardware_interface.hw_states_positions_[0] != 0.0  # Should be updated
        assert initialized_hardware_interface.hw_states_positions_[1] != 0.0
        assert initialized_hardware_interface.hw_states_velocities_[0] != 0.0
        assert initialized_hardware_interface.hw_states_velocities_[1] != 0.0

    def test_read_emergency_stop_active(self, hardware_interface):
        """Test reading during emergency stop"""
        hardware_interface.emergency_stop_active = True
        
        result = hardware_interface.read(None, None)
        
        assert result == 0  # return_type.OK (should return OK but not read)

    def test_write_success(self, initialized_hardware_interface):
        """Test successful velocity command writing"""
        mock_controller = Mock()
        initialized_hardware_interface.controller = mock_controller
        initialized_hardware_interface.hw_commands_velocities_ = [1.0, -1.0]
        
        result = initialized_hardware_interface.write(None, None)
        
        assert result == 0  # return_type.OK

    def test_write_emergency_stop_active(self, hardware_interface):
        """Test writing during emergency stop"""
        hardware_interface.emergency_stop_active = True
        
        result = hardware_interface.write(None, None)
        
        assert result == 0  # return_type.OK (should return OK but not write)


class TestLifecycleMethods:
    """Test ros2_control lifecycle methods"""
    
    def test_on_init_success(self, hardware_interface, mock_info):
        """Test successful initialization"""
        with patch.object(hardware_interface, '_extract_connection_parameters'), \
             patch.object(hardware_interface, '_extract_physical_parameters'), \
             patch.object(hardware_interface, '_extract_motion_parameters'), \
             patch.object(hardware_interface, '_extract_servo_parameters'), \
             patch.object(hardware_interface, '_validate_parameters', return_value=True):
            
            result = hardware_interface.on_init(mock_info)
            assert result == 0  # return_type.OK

    def test_on_init_validation_failure(self, hardware_interface, mock_info):
        """Test initialization with parameter validation failure"""
        with patch.object(hardware_interface, '_extract_connection_parameters'), \
             patch.object(hardware_interface, '_extract_physical_parameters'), \
             patch.object(hardware_interface, '_extract_motion_parameters'), \
             patch.object(hardware_interface, '_extract_servo_parameters'), \
             patch.object(hardware_interface, '_validate_parameters', return_value=False):
            
            result = hardware_interface.on_init(mock_info)
            assert result == 1  # return_type.ERROR

    @patch('basicmicro_driver.hardware_interface.Basicmicro')
    def test_on_configure_success(self, mock_basicmicro_class, hardware_interface):
        """Test successful configuration"""
        mock_controller = Mock()
        mock_controller.ReadVersion.return_value = (True, "RoboClaw 2x15A v4.1.34")
        mock_basicmicro_class.return_value = mock_controller
        
        with patch.object(hardware_interface, '_establish_connection', return_value=True), \
             patch.object(hardware_interface, '_detect_controller_type'), \
             patch.object(hardware_interface, '_configure_servo_parameters'), \
             patch.object(hardware_interface, '_perform_auto_homing'):
            
            result = hardware_interface.on_configure(None)
            assert result == 0  # return_type.OK

    @patch('basicmicro_driver.hardware_interface.Basicmicro')
    def test_on_configure_connection_failure(self, mock_basicmicro_class, hardware_interface):
        """Test configuration with connection failure"""
        mock_controller = Mock()
        mock_basicmicro_class.return_value = mock_controller
        
        with patch.object(hardware_interface, '_establish_connection', return_value=False):
            result = hardware_interface.on_configure(None)
            assert result == 1  # return_type.ERROR

    def test_on_activate_success(self, hardware_interface):
        """Test successful activation"""
        mock_controller = Mock()
        hardware_interface.controller = mock_controller
        
        with patch.object(hardware_interface, '_initialize_state_interfaces'):
            result = hardware_interface.on_activate(None)
            assert result == 0  # return_type.OK
            assert hardware_interface.emergency_stop_active is False

    def test_on_deactivate_success(self, hardware_interface):
        """Test successful deactivation"""
        mock_controller = Mock()
        hardware_interface.controller = mock_controller
        
        result = hardware_interface.on_deactivate(None)
        
        assert result == 0  # return_type.OK
        mock_controller.DutyM1M2.assert_called_once_with(hardware_interface.address, 0, 0)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])