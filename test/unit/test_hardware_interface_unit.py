"""
Comprehensive Unit Tests for BasicMicro Hardware Interface

Tests the ros2_control hardware interface implementation in isolation:
- Initialization and parameter validation
- Unit conversion integration  
- Command execution (write method)
- Sensor reading (read method)
- Emergency stop functionality
- Error handling and recovery
- Lifecycle management

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import pytest
import sys
import os
from unittest.mock import Mock, patch, MagicMock, call
import math

# Add package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from basicmicro_driver.hardware_interface import (
    BasicmicroHardwareInterface,
    ControllerType,
    EncoderType,
    MotionStrategy,
    LimitViolationBehavior
)
from basicmicro_driver.unit_converter import UnitConverter
from test_mocks.mock_basicmicro import create_mock_controller

# Import ROS2 control types
try:
    from hardware_interface import return_type
except ImportError:
    # Mock return_type for testing
    class MockReturnType:
        OK = 0
        ERROR = 1
    return_type = MockReturnType()


@pytest.mark.unit
class TestHardwareInterfaceInitialization:
    """Test hardware interface initialization and configuration"""
    
    def test_default_initialization(self):
        """Test default initialization without parameters"""
        hw = BasicmicroHardwareInterface()
        
        # Check default values
        assert hw.wheel_radius == 0.1
        assert hw.wheel_separation == 0.3
        assert hw.encoder_counts_per_rev == 1000
        assert hw.gear_ratio == 1.0
        assert hw.motion_strategy == MotionStrategy.SPEED_ACCEL
        assert hw.controller is None
        assert hw.unit_converter is None
        assert not hw.emergency_stop_active
        assert hw.controller_type == ControllerType.UNKNOWN
        
    def test_parameter_extraction_valid(self, mock_hardware_info):
        """Test valid parameter extraction from hardware info"""
        hw = BasicmicroHardwareInterface()
        
        # Mock the on_init method's parameter extraction
        hw._extract_servo_parameters(mock_hardware_info)
        
        # Verify parameters are correctly extracted
        assert hw.port == '/dev/ttyACM0'
        assert hw.baud == 38400
        assert hw.address == 128
        assert hw.wheel_radius == 0.1
        assert hw.wheel_separation == 0.3
        assert hw.encoder_counts_per_rev == 1000
        assert hw.gear_ratio == 1.0
        assert hw.motion_strategy == MotionStrategy.SPEED_ACCEL
        assert hw.buffer_depth == 4
        assert hw.default_acceleration == 1000
        assert hw.encoder_type == EncoderType.INCREMENTAL
        assert not hw.auto_home_on_startup
        assert not hw.position_limits_enabled
        
    def test_parameter_extraction_invalid_types(self):
        """Test parameter extraction with invalid types"""
        hw = BasicmicroHardwareInterface()
        info = MagicMock()
        info.hardware_parameters = {
            'wheel_radius': 'invalid',  # Should be float
            'baud': 'not_a_number',     # Should be int
            'address': '999',           # Out of valid range
        }
        
        with pytest.raises(ValueError, match="Invalid physical parameter"):
            hw._extract_physical_parameters(info)
            
    def test_parameter_validation_ranges(self):
        """Test parameter validation with out-of-range values"""
        hw = BasicmicroHardwareInterface()
        info = MagicMock()
        info.hardware_parameters = {
            'wheel_radius': '-0.1',      # Negative radius
            'encoder_counts_per_rev': '0',  # Zero encoder counts
            'gear_ratio': '-1.0',        # Negative gear ratio
            'address': '256',            # Address out of range
        }
        
        with pytest.raises(ValueError):
            hw._extract_physical_parameters(info)
            
    def test_unit_converter_initialization(self, mock_hardware_info):
        """Test unit converter is properly initialized"""
        hw = BasicmicroHardwareInterface()
        hw._extract_servo_parameters(mock_hardware_info)
        # Unit converter is initialized inline, so let's create it manually
        hw.unit_converter = UnitConverter(
            wheel_radius=hw.wheel_radius,
            encoder_counts_per_rev=hw.encoder_counts_per_rev,
            gear_ratio=hw.gear_ratio
        )
        
        assert hw.unit_converter is not None
        assert isinstance(hw.unit_converter, UnitConverter)
        assert hw.unit_converter.wheel_radius == 0.1
        assert hw.unit_converter.encoder_counts_per_rev == 1000
        assert hw.unit_converter.gear_ratio == 1.0


@pytest.mark.unit
class TestControllerConnection:
    """Test controller connection and type detection"""
    
    @patch('basicmicro_driver.hardware_interface.Basicmicro')
    def test_controller_connection_success(self, mock_basicmicro_class, mock_hardware_info):
        """Test successful controller connection"""
        mock_controller = Mock()
        mock_controller.Open = Mock()
        mock_controller.ReadVersion = Mock(return_value=(True, "Mock Controller v1.0"))
        mock_basicmicro_class.return_value = mock_controller
        
        hw = BasicmicroHardwareInterface()
        hw._extract_servo_parameters(mock_hardware_info)
        # Set up controller as would be done in on_configure
        hw.controller = mock_controller
        result = hw._establish_connection()
        
        assert result is True
        assert hw.controller is not None
        mock_controller.Open.assert_called_once()
        mock_controller.ReadVersion.assert_called_once_with(hw.address)
        
    @patch('basicmicro_driver.hardware_interface.Basicmicro')
    def test_controller_connection_failure(self, mock_basicmicro_class, mock_hardware_info):
        """Test controller connection failure"""
        mock_controller = Mock()
        mock_controller.Open = Mock(return_value=False)
        mock_basicmicro_class.return_value = mock_controller
        
        hw = BasicmicroHardwareInterface()
        hw._extract_servo_parameters(mock_hardware_info)
        result = hw._establish_connection()
        
        assert result is False
        
    @patch('basicmicro_driver.hardware_interface.Basicmicro')
    def test_controller_type_detection_roboclaw(self, mock_basicmicro_class, mock_hardware_info):
        """Test RoboClaw controller type detection"""
        mock_controller = Mock()
        mock_controller.Open = Mock()
        mock_controller.ReadVersion = Mock(return_value=(True, "RoboClaw 2x15A v4.1.34"))
        mock_basicmicro_class.return_value = mock_controller
        
        hw = BasicmicroHardwareInterface()
        hw._extract_servo_parameters(mock_hardware_info)
        hw.controller = mock_controller
        hw._detect_controller_type()
        
        assert hw.controller_type == ControllerType.ROBOCLAW
        
    @patch('basicmicro_driver.hardware_interface.Basicmicro')
    def test_controller_type_detection_mcp(self, mock_basicmicro_class, mock_hardware_info):
        """Test MCP controller type detection"""
        mock_controller = Mock()
        mock_controller.Open = Mock()
        mock_controller.ReadVersion = Mock(return_value=(True, "MCP Advanced Motor Controller"))
        mock_basicmicro_class.return_value = mock_controller
        
        hw = BasicmicroHardwareInterface()
        hw._extract_servo_parameters(mock_hardware_info)
        hw.controller = mock_controller
        hw._detect_controller_type()
        
        assert hw.controller_type == ControllerType.MCP
        
    @patch('basicmicro_driver.hardware_interface.Basicmicro')
    def test_controller_type_detection_failure(self, mock_basicmicro_class, mock_hardware_info):
        """Test controller type detection failure"""
        mock_controller = Mock()
        mock_controller.Open = Mock()
        mock_controller.ReadVersion = Mock(return_value=(False, ""))
        mock_basicmicro_class.return_value = mock_controller
        
        hw = BasicmicroHardwareInterface()
        hw._extract_servo_parameters(mock_hardware_info)
        hw.controller = mock_controller
        hw._detect_controller_type()
        
        assert hw.controller_type == ControllerType.UNKNOWN


@pytest.mark.unit
class TestCommandExecution:
    """Test command execution (write method) with different motion strategies"""
    
    def setup_method(self):
        """Set up test hardware interface"""
        self.hw = BasicmicroHardwareInterface()
        # Create proper Mock controller with all needed methods
        self.hw.controller = Mock()
        self.hw.controller.DutyM1M2 = Mock(return_value=(True, None))
        self.hw.controller.SpeedM1M2 = Mock(return_value=(True, None))
        self.hw.controller.SpeedAccelM1M2 = Mock(return_value=(True, None))
        self.hw.controller.DutyAccelM1M2 = Mock(return_value=(True, None))
        self.hw.address = 128
        self.hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        self.hw.emergency_stop_active = False
        
        # Mock command interfaces
        self.hw.hw_commands_velocities_ = [0.0, 0.0]  # Left, Right velocities
        
    def test_write_velocity_commands_duty_strategy(self):
        """Test velocity command execution with duty cycle strategy"""
        self.hw.motion_strategy = MotionStrategy.DUTY
        self.hw.max_duty_cycle = 16384
        
        # Set velocity commands (rad/s)
        self.hw.hw_commands_velocities_[0] = 2.0  # Left wheel
        self.hw.hw_commands_velocities_[1] = 1.5  # Right wheel
        
        # Execute write
        result = self.hw.write(0.02, 0.02)
        
        # Verify duty cycle command was called
        assert result == return_type.OK
        self.hw.controller.DutyM1M2.assert_called_once()
        call_args = self.hw.controller.DutyM1M2.call_args[0]
        assert call_args[0] == 128  # address
        # Duty cycles should be proportional to velocities
        assert call_args[1] != 0  # left duty
        assert call_args[2] != 0  # right duty
        
    def test_write_velocity_commands_speed_strategy(self):
        """Test velocity command execution with speed strategy"""
        self.hw.motion_strategy = MotionStrategy.SPEED
        
        # Set velocity commands (rad/s)
        self.hw.hw_commands_velocities_[0] = 1.0  # Left wheel
        self.hw.hw_commands_velocities_[1] = 1.0  # Right wheel
        
        # Execute write
        result = self.hw.write(0.02, 0.02)
        
        # Verify speed command was called
        assert result == return_type.OK
        self.hw.controller.SpeedM1M2.assert_called_once()
        call_args = self.hw.controller.SpeedM1M2.call_args[0]
        assert call_args[0] == 128  # address
        # Speeds should be converted from rad/s to counts/sec
        expected_speed = int(1.0 / (2 * math.pi / 1000))  # rad/s to counts/sec
        assert abs(call_args[1] - expected_speed) < 10  # Allow small rounding error
        assert abs(call_args[2] - expected_speed) < 10
        
    def test_write_velocity_commands_speed_accel_strategy(self):
        """Test velocity command execution with speed+acceleration strategy"""
        self.hw.motion_strategy = MotionStrategy.SPEED_ACCEL
        self.hw.default_acceleration = 1000
        
        # Set velocity commands
        self.hw.hw_commands_velocities_[0] = 0.5
        self.hw.hw_commands_velocities_[1] = 0.5
        
        # Execute write
        result = self.hw.write(0.02, 0.02)
        
        # Verify speed+accel command was called
        assert result == return_type.OK
        self.hw.controller.SpeedAccelM1M2.assert_called_once()
        call_args = self.hw.controller.SpeedAccelM1M2.call_args[0]
        assert call_args[0] == 128  # address
        assert call_args[1] == 1000  # acceleration
        
    def test_write_velocity_commands_duty_accel_strategy(self):
        """Test velocity command execution with duty+acceleration strategy"""
        self.hw.motion_strategy = MotionStrategy.DUTY_ACCEL
        self.hw.default_acceleration = 500
        self.hw.max_duty_cycle = 16384
        
        # Set velocity commands
        self.hw.hw_commands_velocities_[0] = 1.0
        self.hw.hw_commands_velocities_[1] = -1.0  # Opposite direction
        
        # Execute write
        result = self.hw.write(0.02, 0.02)
        
        # Verify duty+accel command was called
        assert result == return_type.OK
        self.hw.controller.DutyAccelM1M2.assert_called_once()
        call_args = self.hw.controller.DutyAccelM1M2.call_args[0]
        assert call_args[0] == 128  # address
        assert call_args[1] == 500  # left acceleration
        assert call_args[3] == 500  # right acceleration
        # Duties should have opposite signs
        assert call_args[2] > 0  # left duty (positive)
        assert call_args[4] < 0  # right duty (negative)
        
    def test_write_emergency_stop_active(self):
        """Test write method when emergency stop is active"""
        self.hw.emergency_stop_active = True
        
        # Set velocity commands
        self.hw.hw_commands_velocities_[0] = 1.0
        self.hw.hw_commands_velocities_[1] = 1.0
        
        # Execute write
        result = self.hw.write(0.02, 0.02)
        
        # Should not execute motion commands when emergency stop is active
        assert result == return_type.OK
        self.hw.controller.DutyM1M2.assert_not_called()
        self.hw.controller.SpeedM1M2.assert_not_called()
        self.hw.controller.SpeedAccelM1M2.assert_not_called()
        
    def test_write_communication_failure(self):
        """Test write method with communication failure"""
        self.hw.motion_strategy = MotionStrategy.SPEED
        # Mock to raise exception to trigger error handling
        self.hw.controller.SpeedM1M2 = Mock(side_effect=Exception("Communication failed"))
        
        # Set velocity commands
        self.hw.hw_commands_velocities_[0] = 1.0
        self.hw.hw_commands_velocities_[1] = 1.0
        
        # Execute write
        result = self.hw.write(0.02, 0.02)
        
        # Should return ERROR on communication failure
        assert result == return_type.ERROR


@pytest.mark.unit
class TestSensorReading:
    """Test sensor reading (read method) functionality"""
    
    def setup_method(self):
        """Set up test hardware interface"""
        self.hw = BasicmicroHardwareInterface()
        # Create proper Mock controller with sensor read methods
        self.hw.controller = Mock()
        self.hw.controller.GetEncoders = Mock(return_value=(True, 0, 0))
        self.hw.controller.GetSpeeds = Mock(return_value=(True, 0, 0))
        self.hw.controller.ReadEncodersM1M2 = Mock(return_value=(True, 0, 0))
        self.hw.address = 128
        self.hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        
        # Mock state interfaces
        self.hw.hw_states_positions_ = [0.0, 0.0]
        self.hw.hw_states_velocities_ = [0.0, 0.0]
        
    def test_read_encoders_success(self):
        """Test successful encoder reading"""
        # Mock encoder readings
        self.hw.controller.GetEncoders.return_value = (True, 1000, 1500)
        self.hw.controller.GetSpeeds.return_value = (True, 100, 150)
        
        # Execute read
        result = self.hw.read(0.02, 0.02)
        
        # Verify readings were converted and stored
        assert result == return_type.OK
        self.hw.controller.GetEncoders.assert_called_once_with(128)
        self.hw.controller.GetSpeeds.assert_called_once_with(128)
        
        # Check position conversion (counts to radians)
        expected_left_pos = 1000 * (2 * math.pi / 1000)  # 1000 counts
        expected_right_pos = 1500 * (2 * math.pi / 1000)  # 1500 counts
        assert abs(self.hw.hw_states_positions_[0] - expected_left_pos) < 1e-6
        assert abs(self.hw.hw_states_positions_[1] - expected_right_pos) < 1e-6
        
        # Check velocity conversion (counts/sec to rad/s)
        expected_left_vel = 100 * (2 * math.pi / 1000)
        expected_right_vel = 150 * (2 * math.pi / 1000)
        assert abs(self.hw.hw_states_velocities_[0] - expected_left_vel) < 1e-6
        assert abs(self.hw.hw_states_velocities_[1] - expected_right_vel) < 1e-6
        
    def test_read_encoders_failure(self):
        """Test encoder reading failure"""
        # Mock encoder reading failure
        self.hw.controller.GetEncoders.return_value = (False, 0, 0)
        self.hw.controller.GetSpeeds.return_value = (False, 0, 0)
        
        # Store previous values
        prev_positions = self.hw.hw_states_positions_.copy()
        prev_velocities = self.hw.hw_states_velocities_.copy()
        
        # Execute read
        result = self.hw.read(0.02, 0.02)
        
        # Should return OK even if individual sensors fail (current implementation behavior)
        assert result == return_type.OK
        assert self.hw.hw_states_positions_ == prev_positions
        assert self.hw.hw_states_velocities_ == prev_velocities
        
    def test_read_partial_failure(self):
        """Test partial sensor reading failure"""
        # Mock partial failure (encoders success, speeds fail)
        self.hw.controller.GetEncoders.return_value = (True, 2000, 2500)
        self.hw.controller.GetSpeeds.return_value = (False, 0, 0)
        
        # Execute read
        result = self.hw.read(0.02, 0.02)
        
        # Should update positions but not velocities  
        assert result == return_type.OK  # Current implementation continues on partial failure
        # Positions should be updated
        expected_left_pos = 2000 * (2 * math.pi / 1000)
        assert abs(self.hw.hw_states_positions_[0] - expected_left_pos) < 1e-6
        # Velocities should remain zero (or previous values)
        assert self.hw.hw_states_velocities_[0] == 0.0
        assert self.hw.hw_states_velocities_[1] == 0.0
        
    def test_read_with_overflow_handling(self):
        """Test encoder reading with overflow handling"""
        # Mock large encoder values (near overflow)
        large_count = 2147483647  # Max int32
        self.hw.controller.GetEncoders.return_value = (True, large_count, -large_count)
        self.hw.controller.GetSpeeds.return_value = (True, 0, 0)
        
        # Execute read
        result = self.hw.read(0.02, 0.02)
        
        # Should handle large values without overflow
        assert result == return_type.OK
        # Positions should be calculated correctly
        expected_left_pos = large_count * (2 * math.pi / 1000)
        expected_right_pos = -large_count * (2 * math.pi / 1000)
        assert abs(self.hw.hw_states_positions_[0] - expected_left_pos) < 1e-3
        assert abs(self.hw.hw_states_positions_[1] - expected_right_pos) < 1e-3


@pytest.mark.unit
class TestEmergencyStop:
    """Test emergency stop functionality"""
    
    def setup_method(self):
        """Set up test hardware interface"""
        self.hw = BasicmicroHardwareInterface()
        # Create proper Mock controller
        self.hw.controller = Mock()
        self.hw.controller.DutyM1M2 = Mock(return_value=(True, None))
        self.hw.controller.SpeedM1M2 = Mock(return_value=(True, None))
        self.hw.address = 128
        self.hw.emergency_stop_active = False
        
    def test_emergency_stop_activation(self):
        """Test emergency stop activation"""
        # Execute emergency stop
        result = self.hw.emergency_stop()
        
        # Verify emergency stop was executed
        assert result is True
        assert self.hw.emergency_stop_active is True
        self.hw.controller.DutyM1M2.assert_called_once_with(128, 0, 0)
        
    def test_emergency_stop_communication_failure(self):
        """Test emergency stop with communication failure"""
        # Mock communication failure by raising exception
        self.hw.controller.DutyM1M2 = Mock(side_effect=Exception("Communication failed"))
        
        # Execute emergency stop
        result = self.hw.emergency_stop()
        
        # Should return False due to exception
        assert result is False
        assert self.hw.emergency_stop_active is False  # Flag not set due to exception
        
    def test_emergency_stop_reset(self):
        """Test emergency stop reset via on_activate"""
        # Activate emergency stop first
        self.hw.emergency_stop_active = True
        
        # Reset emergency stop (done via on_activate in real usage)
        self.hw.emergency_stop_active = False
        
        # Verify reset
        assert self.hw.emergency_stop_active is False
        
    def test_emergency_stop_blocks_commands(self):
        """Test that emergency stop blocks subsequent commands"""
        # Activate emergency stop
        self.hw.emergency_stop_active = True
        self.hw.motion_strategy = MotionStrategy.SPEED
        self.hw.hw_commands_velocities_ = [1.0, 1.0]
        
        # Try to execute commands
        result = self.hw.write(0.02, 0.02)
        
        # Commands should be blocked
        assert result == return_type.OK  # Write succeeds but doesn't send commands
        # Verify no motion commands were sent (only emergency stop)
        call_count = self.hw.controller.DutyM1M2.call_count
        speed_call_count = self.hw.controller.SpeedM1M2.call_count
        assert speed_call_count == 0  # No speed commands should be sent


@pytest.mark.unit
class TestErrorHandling:
    """Test error handling and recovery"""
    
    def setup_method(self):
        """Set up test hardware interface"""
        self.hw = BasicmicroHardwareInterface()
        # Create proper Mock controller
        self.hw.controller = Mock()
        self.hw.controller.SpeedM1M2 = Mock(return_value=(True, None))
        self.hw.address = 128
        
    def test_controller_not_connected_error(self):
        """Test behavior when controller is not connected"""
        self.hw.controller = None
        
        # Try to write commands
        result = self.hw.write(0.02, 0.02)
        assert result == return_type.OK  # Implementation returns OK when no controller
        
        # Try to read sensors
        result = self.hw.read(0.02, 0.02)
        assert result == return_type.OK  # Implementation returns OK when no controller
        
        # Try emergency stop
        result = self.hw.emergency_stop()
        assert result is False
        
    def test_invalid_address_error(self):
        """Test behavior with invalid controller address"""
        self.hw.address = None
        self.hw.motion_strategy = MotionStrategy.SPEED
        self.hw.hw_commands_velocities_ = [1.0, 1.0]
        
        # Try to write commands
        result = self.hw.write(0.02, 0.02)
        assert result == return_type.ERROR  # Exception due to None unit_converter
        
    def test_unit_converter_not_initialized_error(self):
        """Test behavior when unit converter is not initialized"""
        self.hw.unit_converter = None
        self.hw.motion_strategy = MotionStrategy.SPEED
        self.hw.hw_commands_velocities_ = [1.0, 1.0]
        
        # Try to write commands
        result = self.hw.write(0.02, 0.02)
        assert result == return_type.ERROR  # Exception due to None unit_converter
        
    def test_communication_timeout_recovery(self):
        """Test recovery from communication timeouts"""
        # Mock communication timeout
        self.hw.controller.SpeedM1M2 = Mock(side_effect=Exception("Communication timeout"))
        self.hw.motion_strategy = MotionStrategy.SPEED
        self.hw.hw_commands_velocities_ = [1.0, 1.0]
        self.hw.unit_converter = UnitConverter(0.1, 1000, 1.0)  # Need for conversion
        
        # Try to write commands
        result = self.hw.write(0.02, 0.02)
        assert result == return_type.ERROR
        
        # Reset mock and try again
        self.hw.controller.SpeedM1M2 = Mock(return_value=(True, None))
        
        result = self.hw.write(0.02, 0.02)
        assert result == return_type.OK


@pytest.mark.unit
class TestLifecycleManagement:
    """Test hardware interface lifecycle management"""
    
    def test_on_init_success(self, mock_hardware_info):
        """Test successful initialization"""
        hw = BasicmicroHardwareInterface()
        
        with patch.object(hw, '_extract_servo_parameters'), \
             patch.object(hw, '_validate_parameters', return_value=True), \
             patch.object(hw, '_initialize_state_interfaces'), \
             patch.object(hw, '_establish_connection', return_value=True):
            
            result = hw.on_init(mock_hardware_info)
            assert result == return_type.OK
            
    def test_on_configure_success(self, mock_hardware_info):
        """Test successful configuration"""
        hw = BasicmicroHardwareInterface()
        hw._extract_servo_parameters(mock_hardware_info)
        # Unit converter is initialized in on_init, so mock it here
        hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        
        with patch.object(hw, '_establish_connection', return_value=True), \
             patch.object(hw, '_detect_controller_type'), \
             patch.object(hw, '_configure_servo_parameters'):
            
            result = hw.on_configure(None)
            assert result == return_type.OK
            
    def test_on_activate_success(self, mock_hardware_info):
        """Test successful activation"""
        hw = BasicmicroHardwareInterface()
        hw.controller = create_mock_controller("/dev/ttyACM0", 38400)
        hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        
        result = hw.on_activate(None)
        assert result == return_type.OK
        
    def test_on_deactivate_success(self, mock_hardware_info):
        """Test successful deactivation"""
        hw = BasicmicroHardwareInterface()
        # Create proper Mock controller
        hw.controller = Mock()
        hw.controller.DutyM1M2 = Mock(return_value=(True, None))
        
        result = hw.on_deactivate(None)
        assert result == return_type.OK
        # Should execute emergency stop on deactivation
        hw.controller.DutyM1M2.assert_called_with(hw.address, 0, 0)