"""
Functionality Regression Tests

Tests to ensure that core functionality remains intact across code changes:
- API compatibility and behavior
- Configuration parameter handling
- Motion command execution
- Sensor reading functionality
- Error handling consistency
- Service interface stability

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import pytest
import sys
import os
import json
import math
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock

# Add package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from basicmicro_driver.hardware_interface import BasicmicroHardwareInterface
from basicmicro_driver.unit_converter import UnitConverter
from test_mocks.mock_basicmicro import create_mock_controller


@pytest.mark.regression
class TestHardwareInterfaceRegression:
    """Regression tests for hardware interface functionality"""
    
    def setup_method(self):
        """Set up regression test environment"""
        self.hw = BasicmicroHardwareInterface()
        self.hw.controller = create_mock_controller("/dev/ttyACM0", 38400)
        self.hw.address = 128
        self.hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        self.hw.emergency_stop_active = False
        
        # Set up interfaces
        self.hw.hw_commands_velocities_ = [0.0, 0.0]
        self.hw.hw_states_positions_ = [0.0, 0.0]
        self.hw.hw_states_velocities_ = [0.0, 0.0]
        
        # Reference test data
        self.reference_data = self._load_reference_data()
        
    def _load_reference_data(self):
        """Load reference test data for regression testing"""
        # Define reference test data (this would ideally be loaded from a file)
        return {
            'velocity_commands': [
                {'input': [1.0, 1.0], 'strategy': 'speed_accel', 'expected_calls': 1},
                {'input': [0.5, -0.5], 'strategy': 'speed', 'expected_calls': 1},
                {'input': [2.0, 1.5], 'strategy': 'duty', 'expected_calls': 1},
            ],
            'sensor_readings': [
                {'encoders': (True, 1000, 1500), 'speeds': (True, 100, 150), 
                 'expected_positions': [6.283185, 9.424778], 'expected_velocities': [0.628318, 0.942477]},
                {'encoders': (True, 0, 0), 'speeds': (True, 0, 0), 
                 'expected_positions': [0.0, 0.0], 'expected_velocities': [0.0, 0.0]},
                {'encoders': (True, -500, 250), 'speeds': (True, -50, 25), 
                 'expected_positions': [-3.141592, 1.570796], 'expected_velocities': [-0.314159, 0.157079]},
            ],
            'unit_conversions': [
                {'radians': 1.0, 'expected_counts': 159},
                {'radians': 3.14159, 'expected_counts': 500},
                {'rad_per_sec': 1.0, 'expected_counts_per_sec': 159},
                {'meters': 1.0, 'expected_counts': 1592},
            ]
        }
        
    def test_velocity_command_regression(self):
        """Test that velocity commands produce consistent results"""
        for test_case in self.reference_data['velocity_commands']:
            # Reset mocks
            self.hw.controller.reset_mock()
            
            # Configure strategy
            if test_case['strategy'] == 'speed_accel':
                self.hw.motion_strategy = self.hw.MotionStrategy.SPEED_ACCEL
                self.hw.controller.SpeedAccelM1M2.return_value = True
            elif test_case['strategy'] == 'speed':
                self.hw.motion_strategy = self.hw.MotionStrategy.SPEED
                self.hw.controller.SpeedM1M2.return_value = True
            elif test_case['strategy'] == 'duty':
                self.hw.motion_strategy = self.hw.MotionStrategy.DUTY
                self.hw.max_duty_cycle = 16384
                self.hw.controller.DutyM1M2.return_value = True
                
            # Set commands
            self.hw.hw_commands_velocities_[0] = test_case['input'][0]
            self.hw.hw_commands_velocities_[1] = test_case['input'][1]
            
            # Execute
            result = self.hw.write()
            
            # Verify consistent behavior
            assert result is True, f"Command execution failed for {test_case['strategy']}"
            
            # Verify correct method was called expected number of times
            if test_case['strategy'] == 'speed_accel':
                assert self.hw.controller.SpeedAccelM1M2.call_count == test_case['expected_calls']
            elif test_case['strategy'] == 'speed':
                assert self.hw.controller.SpeedM1M2.call_count == test_case['expected_calls']
            elif test_case['strategy'] == 'duty':
                assert self.hw.controller.DutyM1M2.call_count == test_case['expected_calls']
                
    def test_sensor_reading_regression(self):
        """Test that sensor readings produce consistent results"""
        for test_case in self.reference_data['sensor_readings']:
            # Reset state
            self.hw.hw_states_positions_ = [0.0, 0.0]
            self.hw.hw_states_velocities_ = [0.0, 0.0]
            
            # Configure mock responses
            self.hw.controller.GetEncoders.return_value = test_case['encoders']
            self.hw.controller.GetSpeeds.return_value = test_case['speeds']
            
            # Execute
            result = self.hw.read()
            
            # Verify result
            expected_success = test_case['encoders'][0] and test_case['speeds'][0]
            assert result == expected_success, f"Sensor reading result inconsistent"
            
            if expected_success:
                # Verify position conversions (with tolerance for floating point)
                for i, expected_pos in enumerate(test_case['expected_positions']):
                    actual_pos = self.hw.hw_states_positions_[i]
                    assert abs(actual_pos - expected_pos) < 1e-5, \
                        f"Position {i} regression: expected {expected_pos}, got {actual_pos}"
                        
                # Verify velocity conversions
                for i, expected_vel in enumerate(test_case['expected_velocities']):
                    actual_vel = self.hw.hw_states_velocities_[i]
                    assert abs(actual_vel - expected_vel) < 1e-5, \
                        f"Velocity {i} regression: expected {expected_vel}, got {actual_vel}"
                        
    def test_emergency_stop_regression(self):
        """Test that emergency stop behavior is consistent"""
        # Test emergency stop activation
        result = self.hw.emergency_stop()
        assert result is True
        assert self.hw.emergency_stop_active is True
        self.hw.controller.DutyM1M2.assert_called_once_with(128, 0, 0)
        
        # Test that commands are blocked after emergency stop
        self.hw.hw_commands_velocities_[0] = 1.0
        self.hw.hw_commands_velocities_[1] = 1.0
        
        # Reset mock to check subsequent calls
        self.hw.controller.reset_mock()
        
        result = self.hw.write()
        assert result is True  # Write succeeds but doesn't send commands
        
        # Verify no motion commands were sent
        assert self.hw.controller.SpeedAccelM1M2.call_count == 0
        assert self.hw.controller.SpeedM1M2.call_count == 0
        assert self.hw.controller.DutyM1M2.call_count == 0
        
    def test_unit_conversion_regression(self):
        """Test that unit conversions produce consistent results"""
        for test_case in self.reference_data['unit_conversions']:
            if 'radians' in test_case:
                # Test radians to counts conversion
                result = self.hw.unit_converter.radians_to_counts(test_case['radians'])
                expected = test_case['expected_counts']
                assert abs(result - expected) <= 1, \
                    f"Radians conversion regression: {test_case['radians']} rad -> expected {expected}, got {result}"
                    
                # Test round-trip conversion
                back_to_radians = self.hw.unit_converter.counts_to_radians(result)
                assert abs(back_to_radians - test_case['radians']) < 1e-6, \
                    f"Round-trip radians conversion failed"
                    
            elif 'rad_per_sec' in test_case:
                # Test rad/s to counts/sec conversion
                result = self.hw.unit_converter.rad_per_sec_to_counts_per_sec(test_case['rad_per_sec'])
                expected = test_case['expected_counts_per_sec']
                assert abs(result - expected) <= 1, \
                    f"Velocity conversion regression: {test_case['rad_per_sec']} rad/s -> expected {expected}, got {result}"
                    
            elif 'meters' in test_case:
                # Test meters to counts conversion
                result = self.hw.unit_converter.meters_to_counts(test_case['meters'])
                expected = test_case['expected_counts']
                assert abs(result - expected) <= 1, \
                    f"Distance conversion regression: {test_case['meters']} m -> expected {expected}, got {result}"
                    
    def test_error_handling_regression(self):
        """Test that error handling behavior is consistent"""
        # Test communication failure handling
        self.hw.controller.GetEncoders.return_value = (False, 0, 0)
        self.hw.controller.GetSpeeds.return_value = (False, 0, 0)
        
        # Store previous state
        prev_positions = [1.0, 2.0]
        prev_velocities = [0.1, 0.2]
        self.hw.hw_states_positions_ = prev_positions.copy()
        self.hw.hw_states_velocities_ = prev_velocities.copy()
        
        # Execute read with failure
        result = self.hw.read()
        assert result is False
        
        # Verify state was preserved
        assert self.hw.hw_states_positions_ == prev_positions
        assert self.hw.hw_states_velocities_ == prev_velocities
        
        # Test command failure handling
        self.hw.controller.SpeedAccelM1M2.return_value = False
        self.hw.motion_strategy = self.hw.MotionStrategy.SPEED_ACCEL
        
        result = self.hw.write()
        assert result is False
        
    def test_parameter_validation_regression(self):
        """Test that parameter validation behavior is consistent"""
        # Test invalid parameters still raise appropriate errors
        with pytest.raises(ValueError):
            UnitConverter(-0.1, 1000, 1.0)  # Negative wheel radius
            
        with pytest.raises(ValueError):
            UnitConverter(0.1, 0, 1.0)  # Zero encoder counts
            
        with pytest.raises(ValueError):
            UnitConverter(0.1, 1000, 0.0)  # Zero gear ratio
            
        # Test valid parameters still work
        converter = UnitConverter(0.05, 2000, 2.0)
        assert converter.wheel_radius == 0.05
        assert converter.encoder_counts_per_rev == 2000
        assert converter.gear_ratio == 2.0


@pytest.mark.regression
class TestAPICompatibilityRegression:
    """Test API compatibility and interface stability"""
    
    def test_hardware_interface_api_stability(self):
        """Test that hardware interface API remains stable"""
        hw = BasicmicroHardwareInterface()
        
        # Verify required methods exist
        required_methods = [
            'on_init', 'on_configure', 'on_activate', 'on_deactivate',
            'read', 'write', 'emergency_stop'
        ]
        
        for method_name in required_methods:
            assert hasattr(hw, method_name), f"Missing required method: {method_name}"
            assert callable(getattr(hw, method_name)), f"Method {method_name} is not callable"
            
        # Verify required attributes exist
        required_attributes = [
            'hw_commands_velocities_', 'hw_states_positions_', 'hw_states_velocities_',
            'controller', 'unit_converter', 'emergency_stop_active'
        ]
        
        for attr_name in required_attributes:
            assert hasattr(hw, attr_name), f"Missing required attribute: {attr_name}"
            
    def test_unit_converter_api_stability(self):
        """Test that unit converter API remains stable"""
        converter = UnitConverter(0.1, 1000, 1.0)
        
        # Verify required methods exist
        required_methods = [
            'radians_to_counts', 'counts_to_radians',
            'rad_per_sec_to_counts_per_sec', 'counts_per_sec_to_rad_per_sec',
            'meters_to_counts', 'counts_to_meters',
            'rad_per_sec_to_duty', 'get_conversion_info'
        ]
        
        for method_name in required_methods:
            assert hasattr(converter, method_name), f"Missing required method: {method_name}"
            assert callable(getattr(converter, method_name)), f"Method {method_name} is not callable"
            
        # Test method signatures haven't changed
        assert converter.radians_to_counts(1.0) is not None
        assert converter.counts_to_radians(100) is not None
        assert converter.rad_per_sec_to_counts_per_sec(1.0) is not None
        assert converter.counts_per_sec_to_rad_per_sec(100) is not None
        assert converter.meters_to_counts(1.0) is not None
        assert converter.counts_to_meters(1000) is not None
        
    def test_mock_interface_compatibility(self):
        """Test that mock interface remains compatible with expected API"""
        mock_controller = create_mock_controller("/dev/ttyACM0", 38400)
        
        # Verify expected mock methods exist
        expected_methods = [
            'Open', 'Close', 'DutyM1M2', 'SpeedM1M2', 'SpeedAccelM1M2',
            'DutyAccelM1M2', 'GetEncoders', 'GetSpeeds', 'ReadVersion'
        ]
        
        for method_name in expected_methods:
            assert hasattr(mock_controller, method_name), f"Missing mock method: {method_name}"
            assert callable(getattr(mock_controller, method_name)), f"Mock method {method_name} is not callable"
            
        # Test that mock methods return expected types
        assert isinstance(mock_controller.Open(), bool)
        assert isinstance(mock_controller.DutyM1M2(128, 0, 0), bool)
        assert isinstance(mock_controller.GetEncoders(128), tuple)
        assert len(mock_controller.GetEncoders(128)) == 3  # (success, left, right)


@pytest.mark.regression
class TestConfigurationRegression:
    """Test configuration handling regression"""
    
    def test_parameter_extraction_regression(self):
        """Test that parameter extraction behavior is consistent"""
        # Create mock hardware info with known configuration
        hardware_info = MagicMock()
        hardware_info.hardware_parameters = {
            'port': '/dev/ttyACM0',
            'baud': '38400',
            'address': '128',
            'wheel_radius': '0.1',
            'wheel_separation': '0.3',
            'encoder_counts_per_rev': '1000',
            'gear_ratio': '1.0',
            'motion_strategy': 'speed_accel',
            'buffer_depth': '4',
            'default_acceleration': '1000'
        }
        hardware_info.joints = [MagicMock(), MagicMock()]
        hardware_info.joints[0].name = 'left_wheel_joint'
        hardware_info.joints[1].name = 'right_wheel_joint'
        
        # Test parameter extraction
        hw = BasicmicroHardwareInterface()
        hw._extract_parameters(hardware_info)
        
        # Verify parameters were extracted correctly (regression test)
        assert hw.port == '/dev/ttyACM0'
        assert hw.baud == 38400
        assert hw.address == 128
        assert hw.wheel_radius == 0.1
        assert hw.wheel_separation == 0.3
        assert hw.encoder_counts_per_rev == 1000
        assert hw.gear_ratio == 1.0
        assert hw.motion_strategy == hw.MotionStrategy.SPEED_ACCEL
        assert hw.buffer_depth == 4
        assert hw.default_acceleration == 1000
        
    def test_default_parameter_regression(self):
        """Test that default parameters haven't changed"""
        hw = BasicmicroHardwareInterface()
        
        # Verify default values (regression test)
        assert hw.motion_strategy == hw.MotionStrategy.SPEED_ACCEL
        assert hw.emergency_stop_active is False
        assert hw.controller_type == hw.ControllerType.UNKNOWN
        assert hw.encoder_type == hw.EncoderType.INCREMENTAL
        assert hw.auto_home_on_startup is False
        assert hw.position_limits_enabled is False
        
    def test_motion_strategy_enum_regression(self):
        """Test that motion strategy enum values haven't changed"""
        # These values must remain stable for configuration compatibility
        hw = BasicmicroHardwareInterface()
        
        # Test enum exists and has expected values
        assert hasattr(hw, 'MotionStrategy')
        assert hasattr(hw.MotionStrategy, 'DUTY')
        assert hasattr(hw.MotionStrategy, 'SPEED')
        assert hasattr(hw.MotionStrategy, 'SPEED_ACCEL')
        assert hasattr(hw.MotionStrategy, 'DUTY_ACCEL')
        
        # Test string conversion for configuration
        strategies = ['duty', 'speed', 'speed_accel', 'duty_accel']
        for strategy in strategies:
            # Should be able to convert string to enum
            enum_value = getattr(hw.MotionStrategy, strategy.upper().replace('_ACCEL', '_ACCEL'))
            assert enum_value is not None


@pytest.mark.regression
class TestBehaviorConsistencyRegression:
    """Test behavioral consistency regression"""
    
    def test_continuous_operation_consistency(self):
        """Test that continuous operation behavior is consistent"""
        hw = BasicmicroHardwareInterface()
        hw.controller = create_mock_controller("/dev/ttyACM0", 38400)
        hw.address = 128
        hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        hw.emergency_stop_active = False
        
        # Set up interfaces
        hw.hw_commands_velocities_ = [0.0, 0.0]
        hw.hw_states_positions_ = [0.0, 0.0]
        hw.hw_states_velocities_ = [0.0, 0.0]
        
        # Configure for consistent testing
        hw.motion_strategy = hw.MotionStrategy.SPEED_ACCEL
        hw.controller.GetEncoders.return_value = (True, 1000, 1500)
        hw.controller.GetSpeeds.return_value = (True, 100, 150)
        hw.controller.SpeedAccelM1M2.return_value = True
        
        # Perform multiple cycles and verify consistency
        expected_position_increment = 1000 * (2 * math.pi / 1000)  # Left wheel
        
        for cycle in range(10):
            # Set predictable commands
            hw.hw_commands_velocities_[0] = 1.0
            hw.hw_commands_velocities_[1] = 1.0
            
            # Execute cycle
            read_result = hw.read()
            write_result = hw.write()
            
            # Verify consistent results
            assert read_result is True, f"Read failed on cycle {cycle}"
            assert write_result is True, f"Write failed on cycle {cycle}"
            
            # Verify position updates are consistent
            if cycle == 0:
                initial_position = hw.hw_states_positions_[0]
            else:
                # Position should be updated consistently
                assert hw.hw_states_positions_[0] >= initial_position
                
    def test_error_recovery_consistency(self):
        """Test that error recovery behavior is consistent"""
        hw = BasicmicroHardwareInterface()
        hw.controller = create_mock_controller("/dev/ttyACM0", 38400)
        hw.address = 128
        hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        
        # Set up interfaces
        hw.hw_commands_velocities_ = [0.0, 0.0]
        hw.hw_states_positions_ = [1.0, 2.0]  # Non-zero initial state
        hw.hw_states_velocities_ = [0.1, 0.2]
        
        # Test multiple error/recovery cycles
        for cycle in range(5):
            # Store state before error
            prev_positions = hw.hw_states_positions_.copy()
            prev_velocities = hw.hw_states_velocities_.copy()
            
            # Simulate error
            hw.controller.GetEncoders.return_value = (False, 0, 0)
            hw.controller.GetSpeeds.return_value = (False, 0, 0)
            
            result = hw.read()
            assert result is False, f"Error should be detected on cycle {cycle}"
            
            # Verify state preservation
            assert hw.hw_states_positions_ == prev_positions, f"Position not preserved on cycle {cycle}"
            assert hw.hw_states_velocities_ == prev_velocities, f"Velocity not preserved on cycle {cycle}"
            
            # Recover
            hw.controller.GetEncoders.return_value = (True, 1000, 1500)
            hw.controller.GetSpeeds.return_value = (True, 100, 150)
            
            result = hw.read()
            assert result is True, f"Recovery failed on cycle {cycle}"
            
            # Verify state was updated after recovery
            assert hw.hw_states_positions_ != prev_positions, f"Position not updated after recovery on cycle {cycle}"