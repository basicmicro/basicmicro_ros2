"""
Tests for Enhanced Library Integration and Error Monitoring (Task 1.7)

This module tests the comprehensive error monitoring, diagnostic capabilities,
buffer management, and advanced position control functions that integrate
with the Basicmicro Python library.
"""

import pytest
import sys
import os
from unittest.mock import Mock, patch, MagicMock

# Add src directory to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from basicmicro_driver.hardware_interface import BasicmicroHardwareInterface
from test_mocks.mock_basicmicro import MockBasicmicro, ControllerType


class TestComprehensiveDiagnostics:
    """Test comprehensive diagnostic data collection"""
    
    @pytest.fixture
    def mock_controller(self):
        """Create a mock controller with all diagnostic capabilities"""
        mock = MockBasicmicro("/dev/ttyMOCK", 38400)
        mock.Open()
        return mock
    
    @pytest.fixture
    def initialized_interface(self, mock_controller):
        """Create initialized hardware interface with mock controller"""
        interface = BasicmicroHardwareInterface()
        
        # Mock the hardware info for initialization
        mock_info = Mock()
        mock_info.hardware_parameters = {
            "port": "/dev/ttyMOCK",
            "baud": "38400",
            "address": "0x80",
            "wheel_radius": "0.1",
            "wheel_separation": "0.3",
            "encoder_counts_per_rev": "1000",
            "gear_ratio": "1.0",
            "motion_strategy": "speed_accel",
            "buffer_depth": "4",
            "default_acceleration": "1000"
        }
        mock_info.joints = [Mock(name="left_wheel"), Mock(name="right_wheel")]
        
        # Initialize the interface
        interface.on_init(mock_info)
        interface.controller = mock_controller
        interface.address = 0x80
        
        return interface
    
    def test_read_comprehensive_diagnostics_success(self, initialized_interface):
        """Test successful comprehensive diagnostics reading"""
        diagnostics = initialized_interface.read_comprehensive_diagnostics()
        
        # Verify all diagnostic categories are present
        assert 'error_limits_exceeded' in diagnostics
        assert 'comprehensive_status' in diagnostics
        assert 'motor_currents' in diagnostics
        assert 'temperatures' in diagnostics
        assert 'voltages' in diagnostics
        assert 'speed_errors' in diagnostics
        assert 'position_errors' in diagnostics
        assert 'buffer_status' in diagnostics
        
        # Verify data structure
        assert isinstance(diagnostics['motor_currents'], dict)
        assert 'left_current_amps' in diagnostics['motor_currents']
        assert 'right_current_amps' in diagnostics['motor_currents']
        
        assert isinstance(diagnostics['temperatures'], dict)
        assert 'temp1_celsius' in diagnostics['temperatures']
        assert 'temp2_celsius' in diagnostics['temperatures']
        
        assert isinstance(diagnostics['voltages'], dict)
        assert 'main_battery_volts' in diagnostics['voltages']
        assert 'logic_battery_volts' in diagnostics['voltages']
    
    def test_read_comprehensive_diagnostics_no_controller(self, initialized_interface):
        """Test diagnostics reading with no controller"""
        initialized_interface.controller = None
        diagnostics = initialized_interface.read_comprehensive_diagnostics()
        
        # Should return empty dictionary when no controller
        assert diagnostics == {}
    
    def test_read_comprehensive_diagnostics_communication_failure(self, initialized_interface):
        """Test diagnostics reading with communication failures"""
        # Set high failure rate for testing
        initialized_interface.controller.set_communication_failure_rate(1.0)
        
        diagnostics = initialized_interface.read_comprehensive_diagnostics()
        
        # Should still return dictionary structure but may be incomplete
        assert isinstance(diagnostics, dict)
    
    def test_check_error_limits_success(self, initialized_interface):
        """Test successful error limit checking"""
        # Initially no error limits exceeded
        assert not initialized_interface.check_error_limits()
        
        # Trigger error state
        initialized_interface.controller.trigger_error_state(True)
        assert initialized_interface.check_error_limits()
        
        # Clear error state
        initialized_interface.controller.trigger_error_state(False)
        assert not initialized_interface.check_error_limits()
    
    def test_check_error_limits_no_controller(self, initialized_interface):
        """Test error limit checking with no controller"""
        initialized_interface.controller = None
        assert not initialized_interface.check_error_limits()


class TestBufferStatusMonitoring:
    """Test buffer status monitoring and interpretation"""
    
    @pytest.fixture
    def initialized_interface(self):
        """Create initialized interface for buffer testing"""
        interface = BasicmicroHardwareInterface()
        mock_controller = MockBasicmicro("/dev/ttyMOCK", 38400)
        mock_controller.Open()
        
        # Mock initialization
        mock_info = Mock()
        mock_info.hardware_parameters = {
            "port": "/dev/ttyMOCK", "baud": "38400", "address": "0x80",
            "wheel_radius": "0.1", "wheel_separation": "0.3",
            "encoder_counts_per_rev": "1000", "gear_ratio": "1.0",
            "motion_strategy": "speed_accel", "buffer_depth": "4",
            "default_acceleration": "1000"
        }
        mock_info.joints = [Mock(name="left_wheel"), Mock(name="right_wheel")]
        
        interface.on_init(mock_info)
        interface.controller = mock_controller
        interface.address = 0x80
        
        return interface
    
    def test_interpret_buffer_status_idle(self, initialized_interface):
        """Test buffer status interpretation for idle state"""
        status = initialized_interface._interpret_buffer_status(0xFF)
        
        assert status['status'] == 'IDLE'
        assert status['commands_in_buffer'] == 0
        assert status['executing'] == False
        assert 'No commands in buffer' in status['description']
    
    def test_interpret_buffer_status_executing(self, initialized_interface):
        """Test buffer status interpretation for executing state"""
        status = initialized_interface._interpret_buffer_status(0)
        
        assert status['status'] == 'EXECUTING'
        assert status['commands_in_buffer'] == 0
        assert status['executing'] == True
        assert 'Last command is executing' in status['description']
    
    def test_interpret_buffer_status_buffered(self, initialized_interface):
        """Test buffer status interpretation for buffered commands"""
        for command_count in [1, 5, 15, 32]:
            status = initialized_interface._interpret_buffer_status(command_count)
            
            assert status['status'] == 'BUFFERED'
            assert status['commands_in_buffer'] == command_count
            assert status['executing'] == False
            assert f'{command_count} commands' in status['description']
    
    def test_get_buffer_status_success(self, initialized_interface):
        """Test successful buffer status retrieval"""
        buffer_status = initialized_interface.get_buffer_status()
        
        assert buffer_status['available'] == True
        assert 'status' in buffer_status
        assert 'utilization_percent' in buffer_status
        assert 'available_slots' in buffer_status
        assert 'raw_value' in buffer_status
        
        # Should start idle
        assert buffer_status['status'] == 'IDLE'
        assert buffer_status['utilization_percent'] == 0
        assert buffer_status['available_slots'] == 32
    
    def test_get_buffer_status_no_controller(self, initialized_interface):
        """Test buffer status with no controller"""
        initialized_interface.controller = None
        buffer_status = initialized_interface.get_buffer_status()
        
        assert buffer_status['available'] == False


class TestServoErrorMonitoring:
    """Test servo-specific error monitoring and analysis"""
    
    @pytest.fixture
    def initialized_interface(self):
        """Create initialized interface for servo testing"""
        interface = BasicmicroHardwareInterface()
        mock_controller = MockBasicmicro("/dev/ttyMOCK", 38400)
        mock_controller.Open()
        
        # Mock initialization
        mock_info = Mock()
        mock_info.hardware_parameters = {
            "port": "/dev/ttyMOCK", "baud": "38400", "address": "0x80",
            "wheel_radius": "0.1", "wheel_separation": "0.3",
            "encoder_counts_per_rev": "1000", "gear_ratio": "1.0",
            "motion_strategy": "speed_accel", "buffer_depth": "4",
            "default_acceleration": "1000"
        }
        mock_info.joints = [Mock(name="left_wheel"), Mock(name="right_wheel")]
        
        interface.on_init(mock_info)
        interface.controller = mock_controller
        interface.address = 0x80
        
        return interface
    
    def test_get_servo_errors_success(self, initialized_interface):
        """Test successful servo error retrieval"""
        servo_errors = initialized_interface.get_servo_errors()
        
        # Verify structure
        assert 'position_errors_counts' in servo_errors
        assert 'position_errors_radians' in servo_errors
        assert 'speed_errors_counts_per_sec' in servo_errors
        assert 'speed_errors_rad_per_sec' in servo_errors
        assert 'error_analysis' in servo_errors
        
        # Verify unit conversions
        pos_counts = servo_errors['position_errors_counts']
        pos_rads = servo_errors['position_errors_radians']
        
        assert isinstance(pos_counts, dict)
        assert 'left' in pos_counts and 'right' in pos_counts
        assert isinstance(pos_rads, dict)
        assert 'left' in pos_rads and 'right' in pos_rads
    
    def test_analyze_servo_errors_acceptable(self, initialized_interface):
        """Test servo error analysis with acceptable errors"""
        # Mock small errors (within thresholds)
        servo_errors = {
            'position_errors_counts': {'left': 50, 'right': -30},
            'speed_errors_counts_per_sec': {'left': 100, 'right': -200}
        }
        
        analysis = initialized_interface._analyze_servo_errors(servo_errors)
        
        assert analysis['position_errors_acceptable'] == True
        assert analysis['speed_errors_acceptable'] == True
        assert len(analysis['warnings']) == 0
    
    def test_analyze_servo_errors_excessive(self, initialized_interface):
        """Test servo error analysis with excessive errors"""
        # Mock large errors (exceed thresholds)
        servo_errors = {
            'position_errors_counts': {'left': 150, 'right': -200},  # Exceed 100 threshold
            'speed_errors_counts_per_sec': {'left': 600, 'right': -800}  # Exceed 500 threshold
        }
        
        analysis = initialized_interface._analyze_servo_errors(servo_errors)
        
        assert analysis['position_errors_acceptable'] == False
        assert analysis['speed_errors_acceptable'] == False
        assert len(analysis['warnings']) > 0
        
        # Check specific warning messages
        warnings_text = ' '.join(analysis['warnings'])
        assert 'left motor position error exceeds threshold' in warnings_text
        assert 'right motor position error exceeds threshold' in warnings_text
        assert 'left motor speed error exceeds threshold' in warnings_text
        assert 'right motor speed error exceeds threshold' in warnings_text


class TestSystemHealthMonitoring:
    """Test comprehensive system health monitoring"""
    
    @pytest.fixture
    def initialized_interface(self):
        """Create initialized interface for health monitoring testing"""
        interface = BasicmicroHardwareInterface()
        mock_controller = MockBasicmicro("/dev/ttyMOCK", 38400)
        mock_controller.Open()
        
        # Mock initialization
        mock_info = Mock()
        mock_info.hardware_parameters = {
            "port": "/dev/ttyMOCK", "baud": "38400", "address": "0x80",
            "wheel_radius": "0.1", "wheel_separation": "0.3",
            "encoder_counts_per_rev": "1000", "gear_ratio": "1.0",
            "motion_strategy": "speed_accel", "buffer_depth": "4",
            "default_acceleration": "1000"
        }
        mock_info.joints = [Mock(name="left_wheel"), Mock(name="right_wheel")]
        
        interface.on_init(mock_info)
        interface.controller = mock_controller
        interface.address = 0x80
        
        return interface
    
    def test_monitor_system_health_ok(self, initialized_interface):
        """Test system health monitoring with normal conditions"""
        health = initialized_interface.monitor_system_health()
        
        assert health['overall_status'] == 'OK'
        assert isinstance(health['errors'], list)
        assert isinstance(health['warnings'], list)
        assert isinstance(health['diagnostics'], dict)
        
        # Should have no errors with default mock values
        assert len(health['errors']) == 0
    
    def test_monitor_system_health_error_limits(self, initialized_interface):
        """Test system health monitoring with error limits exceeded"""
        # Trigger error state
        initialized_interface.controller.trigger_error_state(True)
        
        health = initialized_interface.monitor_system_health()
        
        assert health['overall_status'] == 'ERROR'
        assert len(health['errors']) > 0
        
        error_text = ' '.join(health['errors'])
        assert 'Controller error limits exceeded' in error_text
        assert 'Power-On-Reset required' in error_text
    
    def test_monitor_system_health_voltage_warnings(self, initialized_interface):
        """Test system health monitoring with voltage warnings"""
        # Mock low voltage condition
        mock_controller = initialized_interface.controller
        mock_controller._voltages = [9.0, 4.5]  # Low voltages
        
        health = initialized_interface.monitor_system_health()
        
        assert health['overall_status'] == 'WARNING'
        assert len(health['warnings']) > 0
        
        warnings_text = ' '.join(health['warnings'])
        assert 'Low main battery voltage' in warnings_text
    
    def test_monitor_system_health_current_warnings(self, initialized_interface):
        """Test system health monitoring with high current warnings"""
        # Mock high current condition
        mock_controller = initialized_interface.controller
        mock_controller._currents = [13.0, 14.0]  # High currents (>80% of 15A)
        
        health = initialized_interface.monitor_system_health()
        
        assert health['overall_status'] == 'WARNING'
        assert len(health['warnings']) > 0
        
        warnings_text = ' '.join(health['warnings'])
        assert 'High left motor current' in warnings_text
        assert 'High right motor current' in warnings_text
    
    def test_monitor_system_health_temperature_warnings(self, initialized_interface):
        """Test system health monitoring with temperature warnings"""
        # Mock high temperature condition
        mock_controller = initialized_interface.controller
        mock_controller._temperatures = [75.0, 80.0]  # High temperatures
        
        health = initialized_interface.monitor_system_health()
        
        assert health['overall_status'] == 'WARNING'
        assert len(health['warnings']) > 0
        
        warnings_text = ' '.join(health['warnings'])
        assert 'High temperature sensor' in warnings_text


class TestAdvancedPositionControl:
    """Test advanced position control and servo functions"""
    
    @pytest.fixture
    def initialized_interface(self):
        """Create initialized interface for position control testing"""
        interface = BasicmicroHardwareInterface()
        mock_controller = MockBasicmicro("/dev/ttyMOCK", 38400)
        mock_controller.Open()
        
        # Mock initialization
        mock_info = Mock()
        mock_info.hardware_parameters = {
            "port": "/dev/ttyMOCK", "baud": "38400", "address": "0x80",
            "wheel_radius": "0.1", "wheel_separation": "0.3",
            "encoder_counts_per_rev": "1000", "gear_ratio": "1.0",
            "motion_strategy": "speed_accel", "buffer_depth": "4",
            "default_acceleration": "1000"
        }
        mock_info.joints = [Mock(name="left_wheel"), Mock(name="right_wheel")]
        
        interface.on_init(mock_info)
        interface.controller = mock_controller
        interface.address = 0x80
        
        return interface
    
    def test_execute_absolute_position_command_success(self, initialized_interface):
        """Test successful absolute position command execution"""
        success = initialized_interface.execute_absolute_position_command(
            left_pos_rad=1.0,
            right_pos_rad=-0.5,
            max_speed_rad_s=2.0,
            acceleration_rad_s2=1.0,
            deceleration_rad_s2=1.5,
            buffer_command=False
        )
        
        assert success == True
        
        # Verify controller is in position mode
        state = initialized_interface.controller.get_internal_state()
        assert state['position_mode'] == True
    
    def test_execute_absolute_position_command_buffered(self, initialized_interface):
        """Test buffered absolute position command execution"""
        success = initialized_interface.execute_absolute_position_command(
            left_pos_rad=2.0,
            right_pos_rad=1.0,
            max_speed_rad_s=1.5,
            acceleration_rad_s2=0.5,
            deceleration_rad_s2=0.8,
            buffer_command=True
        )
        
        assert success == True
        
        # Check buffer status after buffered command
        buffer_status = initialized_interface.get_buffer_status()
        assert buffer_status['commands_in_buffer'] > 0
    
    def test_execute_absolute_position_command_no_controller(self, initialized_interface):
        """Test position command with no controller"""
        initialized_interface.controller = None
        
        success = initialized_interface.execute_absolute_position_command(
            left_pos_rad=1.0, right_pos_rad=1.0,
            max_speed_rad_s=1.0, acceleration_rad_s2=1.0,
            deceleration_rad_s2=1.0
        )
        
        assert success == False
    
    def test_execute_distance_command_success(self, initialized_interface):
        """Test successful distance command execution"""
        success = initialized_interface.execute_distance_command(
            left_distance_m=1.0,
            right_distance_m=0.8,
            speed_m_s=0.5,
            acceleration_m_s2=0.2,
            buffer_command=False
        )
        
        assert success == True
    
    def test_execute_distance_command_buffered(self, initialized_interface):
        """Test buffered distance command execution"""
        success = initialized_interface.execute_distance_command(
            left_distance_m=0.5,
            right_distance_m=0.5,
            speed_m_s=0.3,
            acceleration_m_s2=0.1,
            buffer_command=True
        )
        
        assert success == True
        
        # Check buffer status after buffered command
        buffer_status = initialized_interface.get_buffer_status()
        assert buffer_status['commands_in_buffer'] > 0
    
    def test_clear_motion_buffers_success(self, initialized_interface):
        """Test successful motion buffer clearing"""
        # First add some commands to buffer
        initialized_interface.execute_distance_command(
            left_distance_m=1.0, right_distance_m=1.0,
            speed_m_s=0.5, acceleration_m_s2=0.2, buffer_command=True
        )
        
        # Verify buffer has commands
        buffer_status = initialized_interface.get_buffer_status()
        assert buffer_status['commands_in_buffer'] > 0
        
        # Clear buffers
        success = initialized_interface.clear_motion_buffers()
        assert success == True
        
        # Verify buffers are cleared
        buffer_status = initialized_interface.get_buffer_status()
        assert buffer_status['status'] == 'IDLE'
    
    def test_clear_motion_buffers_no_controller(self, initialized_interface):
        """Test buffer clearing with no controller"""
        initialized_interface.controller = None
        
        success = initialized_interface.clear_motion_buffers()
        assert success == False


class TestIntegrationScenarios:
    """Test integration scenarios combining multiple enhanced features"""
    
    @pytest.fixture
    def initialized_interface(self):
        """Create initialized interface for integration testing"""
        interface = BasicmicroHardwareInterface()
        mock_controller = MockBasicmicro("/dev/ttyMOCK", 38400)
        mock_controller.Open()
        
        # Mock initialization
        mock_info = Mock()
        mock_info.hardware_parameters = {
            "port": "/dev/ttyMOCK", "baud": "38400", "address": "0x80",
            "wheel_radius": "0.1", "wheel_separation": "0.3",
            "encoder_counts_per_rev": "1000", "gear_ratio": "1.0",
            "motion_strategy": "speed_accel", "buffer_depth": "4",
            "default_acceleration": "1000"
        }
        mock_info.joints = [Mock(name="left_wheel"), Mock(name="right_wheel")]
        
        interface.on_init(mock_info)
        interface.controller = mock_controller
        interface.address = 0x80
        
        return interface
    
    def test_servo_operation_with_error_monitoring(self, initialized_interface):
        """Test servo operation combined with comprehensive error monitoring"""
        # Execute position command
        success = initialized_interface.execute_absolute_position_command(
            left_pos_rad=1.0, right_pos_rad=1.0,
            max_speed_rad_s=1.0, acceleration_rad_s2=0.5,
            deceleration_rad_s2=0.5
        )
        assert success == True
        
        # Monitor servo errors
        servo_errors = initialized_interface.get_servo_errors()
        assert 'position_errors_counts' in servo_errors
        assert 'error_analysis' in servo_errors
        
        # Check system health during servo operation
        health = initialized_interface.monitor_system_health()
        assert health['overall_status'] in ['OK', 'WARNING']  # Should not be ERROR
        
        # Verify buffer status during operation
        buffer_status = initialized_interface.get_buffer_status()
        assert buffer_status['available'] == True
    
    def test_buffered_trajectory_with_monitoring(self, initialized_interface):
        """Test buffered trajectory execution with comprehensive monitoring"""
        # Execute multiple buffered commands
        for i in range(3):
            success = initialized_interface.execute_distance_command(
                left_distance_m=0.1 * (i + 1),
                right_distance_m=0.1 * (i + 1),
                speed_m_s=0.2,
                acceleration_m_s2=0.1,
                buffer_command=True
            )
            assert success == True
        
        # Monitor buffer utilization
        buffer_status = initialized_interface.get_buffer_status()
        assert buffer_status['commands_in_buffer'] >= 2  # At least 2 commands should be buffered
        assert buffer_status['utilization_percent'] > 0
        
        # Monitor system health during buffered execution
        health = initialized_interface.monitor_system_health()
        assert 'diagnostics' in health
        
        # Check comprehensive diagnostics
        diagnostics = initialized_interface.read_comprehensive_diagnostics()
        assert 'buffer_status' in diagnostics
        assert diagnostics['buffer_status']['commands_in_buffer'] >= 2  # At least 2 commands should be buffered
    
    def test_error_recovery_scenario(self, initialized_interface):
        """Test error detection and recovery scenario"""
        # Trigger error condition
        initialized_interface.controller.trigger_error_state(True)
        
        # Check error detection
        assert initialized_interface.check_error_limits() == True
        
        # Monitor system health during error
        health = initialized_interface.monitor_system_health()
        assert health['overall_status'] == 'ERROR'
        assert len(health['errors']) > 0
        
        # Attempt emergency stop/clear buffers
        success = initialized_interface.clear_motion_buffers()
        assert success == True
        
        # Clear error state (simulating power-on-reset)
        initialized_interface.controller.trigger_error_state(False)
        
        # Verify recovery
        assert initialized_interface.check_error_limits() == False
        health = initialized_interface.monitor_system_health()
        assert health['overall_status'] in ['OK', 'WARNING']