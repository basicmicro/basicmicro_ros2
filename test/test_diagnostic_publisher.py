#!/usr/bin/env python3
"""
Test suite for Basicmicro Diagnostic Publisher

Tests comprehensive error reporting and diagnostic system functionality including:
- Hardware status monitoring and diagnostics
- Motion control status and performance reporting  
- Communication status tracking and failure detection
- Safety status monitoring and alerts
- ROS2 diagnostic framework integration
- Diagnostic data collection and analysis

Author: Claude Code Assistant
Date: 2025-07-10
"""

import unittest
import time
import threading
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# Add the package directory to the path for testing
package_dir = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, package_dir)

try:
    from basicmicro_driver.diagnostic_publisher import (
        DiagnosticPublisher, DiagnosticLevel, DiagnosticCategory, DiagnosticData,
        HardwareStatus, MotionControlStatus, CommunicationStatus, SafetyStatus
    )
    from test_mocks.mock_basicmicro import MockBasicmicro
    DIAGNOSTIC_PUBLISHER_AVAILABLE = True
except ImportError as e:
    print(f"Import error: {e}")
    DIAGNOSTIC_PUBLISHER_AVAILABLE = False


class MockHardwareInterface:
    """Mock hardware interface for testing diagnostic publisher"""
    
    def __init__(self):
        self.status_data = (True, 0x00)  # success, status flags
        self.voltage_data = (True, 120, 50)  # success, main_voltage*10, logic_voltage*10
        self.current_data = (True, 150, 200)  # success, motor1_current*100, motor2_current*100
        self.temperature_data = (True, 250, 280)  # success, temp1*10, temp2*10
        self.error_data = (True, False)  # success, error_state
        self.position_error_data = (True, 50, -75)  # success, left_error, right_error
        self.speed_error_data = (True, 25, -30)  # success, left_error, right_error
        self.speed_data = (True, 1000, 1200)  # success, left_speed, right_speed
        self.emergency_stop_active = False
        self.position_limits_enabled = True
        
        # Communication statistics
        self.comm_stats = {
            'total_commands': 1000,
            'successful_commands': 950,
            'failed_commands': 50,
            'timeout_errors': 5
        }
        
        self.latency_stats = {
            'average_ms': 15.0,
            'max_ms': 45.0
        }
        
    def get_status(self):
        return self.status_data
        
    def get_voltages(self):
        return self.voltage_data
        
    def read_currents(self):
        return self.current_data
        
    def get_temperatures(self):
        return self.temperature_data
        
    def read_error(self):
        return self.error_data
        
    def get_position_errors(self):
        return self.position_error_data
        
    def get_speed_errors(self):
        return self.speed_error_data
        
    def get_speeds(self):
        return self.speed_data
        
    def is_emergency_stop_active(self):
        return self.emergency_stop_active
        
    def get_communication_statistics(self):
        return self.comm_stats
        
    def get_latency_statistics(self):
        return self.latency_stats


class MockTrajectoryMonitor:
    """Mock trajectory monitor for testing"""
    
    def __init__(self):
        self.monitoring_status = {
            'trajectory_active': False,
            'monitoring_active': True,
            'current_errors': {'position': 0, 'speed': 0}
        }
        
    def get_monitoring_status(self):
        return self.monitoring_status


class MockBufferManager:
    """Mock buffer manager for testing"""
    
    def __init__(self):
        self.buffer_status = {
            'left_buffer': 5,
            'right_buffer': 3
        }
        
    def get_buffer_status(self):
        return self.buffer_status


@unittest.skipUnless(DIAGNOSTIC_PUBLISHER_AVAILABLE, "Diagnostic publisher not available")
class TestDiagnosticPublisherCore(unittest.TestCase):
    """Test core diagnostic publisher functionality"""
    
    def setUp(self):
        """Set up test environment"""
        self.mock_hardware = MockHardwareInterface()
        self.mock_trajectory_monitor = MockTrajectoryMonitor()
        self.mock_buffer_manager = MockBufferManager()
        
        self.diagnostic_publisher = DiagnosticPublisher(
            hardware_interface=self.mock_hardware,
            trajectory_monitor=self.mock_trajectory_monitor,
            buffer_manager=self.mock_buffer_manager
        )
        
    def tearDown(self):
        """Clean up after tests"""
        if hasattr(self.diagnostic_publisher, 'stop_diagnostic_monitoring'):
            self.diagnostic_publisher.stop_diagnostic_monitoring()
    
    def test_diagnostic_publisher_initialization(self):
        """Test diagnostic publisher initializes correctly"""
        self.assertIsNotNone(self.diagnostic_publisher)
        self.assertEqual(self.diagnostic_publisher.hardware_interface, self.mock_hardware)
        self.assertEqual(self.diagnostic_publisher.trajectory_monitor, self.mock_trajectory_monitor)
        self.assertEqual(self.diagnostic_publisher.buffer_manager, self.mock_buffer_manager)
        
        # Check default parameters
        self.assertEqual(self.diagnostic_publisher.diagnostic_rate, 5.0)
        self.assertEqual(self.diagnostic_publisher.diagnostic_timeout, 2.0)
        self.assertEqual(self.diagnostic_publisher.diagnostic_history_size, 100)
        
    def test_parameter_loading(self):
        """Test diagnostic parameter loading and validation"""
        # Check hardware monitoring parameters
        self.assertEqual(self.diagnostic_publisher.min_battery_voltage, 10.0)
        self.assertEqual(self.diagnostic_publisher.max_battery_voltage, 16.0)
        self.assertEqual(self.diagnostic_publisher.max_motor_current, 10.0)
        self.assertEqual(self.diagnostic_publisher.max_temperature, 70.0)
        
        # Check communication parameters
        self.assertEqual(self.diagnostic_publisher.max_communication_latency, 50.0)
        self.assertEqual(self.diagnostic_publisher.min_communication_success_rate, 0.95)
        self.assertEqual(self.diagnostic_publisher.communication_timeout_threshold, 5.0)
        
    def test_diagnostic_data_structures(self):
        """Test diagnostic data structure initialization"""
        self.assertIsInstance(self.diagnostic_publisher.hardware_status, HardwareStatus)
        self.assertIsInstance(self.diagnostic_publisher.motion_control_status, MotionControlStatus)
        self.assertIsInstance(self.diagnostic_publisher.communication_status, CommunicationStatus)
        self.assertIsInstance(self.diagnostic_publisher.safety_status, SafetyStatus)
        
        # Check diagnostic history initialization
        self.assertIn('hardware', self.diagnostic_publisher.diagnostic_history)
        self.assertIn('motion', self.diagnostic_publisher.diagnostic_history)
        self.assertIn('communication', self.diagnostic_publisher.diagnostic_history)
        self.assertIn('safety', self.diagnostic_publisher.diagnostic_history)
        
    def test_diagnostic_monitoring_lifecycle(self):
        """Test diagnostic monitoring start/stop lifecycle"""
        # Initially not active
        self.assertFalse(self.diagnostic_publisher.diagnostic_active)
        
        # Start monitoring
        result = self.diagnostic_publisher.start_diagnostic_monitoring()
        self.assertTrue(result)
        self.assertTrue(self.diagnostic_publisher.diagnostic_active)
        
        # Wait for thread to start
        time.sleep(0.1)
        self.assertIsNotNone(self.diagnostic_publisher.diagnostic_thread)
        self.assertTrue(self.diagnostic_publisher.diagnostic_thread.is_alive())
        
        # Stop monitoring
        result = self.diagnostic_publisher.stop_diagnostic_monitoring()
        self.assertTrue(result)
        self.assertFalse(self.diagnostic_publisher.diagnostic_active)
        
    def test_diagnostic_monitoring_already_active(self):
        """Test handling of starting monitoring when already active"""
        # Start monitoring
        result1 = self.diagnostic_publisher.start_diagnostic_monitoring()
        self.assertTrue(result1)
        
        # Try to start again
        result2 = self.diagnostic_publisher.start_diagnostic_monitoring()
        self.assertTrue(result2)  # Should succeed but warn
        self.assertTrue(self.diagnostic_publisher.diagnostic_active)
        
        # Stop monitoring
        self.diagnostic_publisher.stop_diagnostic_monitoring()
        
    def test_diagnostic_monitoring_stop_when_inactive(self):
        """Test handling of stopping monitoring when not active"""
        # Initially not active
        self.assertFalse(self.diagnostic_publisher.diagnostic_active)
        
        # Try to stop when not active
        result = self.diagnostic_publisher.stop_diagnostic_monitoring()
        self.assertTrue(result)  # Should succeed but warn


@unittest.skipUnless(DIAGNOSTIC_PUBLISHER_AVAILABLE, "Diagnostic publisher not available")
class TestHardwareStatusMonitoring(unittest.TestCase):
    """Test hardware status monitoring and diagnostics"""
    
    def setUp(self):
        """Set up test environment"""
        self.mock_hardware = MockHardwareInterface()
        self.diagnostic_publisher = DiagnosticPublisher(hardware_interface=self.mock_hardware)
        
    def test_hardware_diagnostics_collection(self):
        """Test collection of hardware diagnostic data"""
        # Set test values
        self.mock_hardware.voltage_data = (True, 125, 55)  # 12.5V main, 5.5V logic
        self.mock_hardware.current_data = (True, 250, 300)  # 2.5A, 3.0A
        self.mock_hardware.temperature_data = (True, 350, 400)  # 35°C, 40°C
        self.mock_hardware.error_data = (True, False)
        
        # Collect diagnostics
        self.diagnostic_publisher._collect_hardware_diagnostics()
        
        # Verify data collection
        self.assertEqual(self.diagnostic_publisher.hardware_status.main_battery_voltage, 12.5)
        self.assertEqual(self.diagnostic_publisher.hardware_status.logic_battery_voltage, 5.5)
        self.assertEqual(self.diagnostic_publisher.hardware_status.motor1_current, 2.5)
        self.assertEqual(self.diagnostic_publisher.hardware_status.motor2_current, 3.0)
        self.assertEqual(self.diagnostic_publisher.hardware_status.temperature1, 35.0)
        self.assertEqual(self.diagnostic_publisher.hardware_status.temperature2, 40.0)
        self.assertFalse(self.diagnostic_publisher.hardware_status.error_state)
        
    def test_hardware_diagnostics_communication_failure(self):
        """Test hardware diagnostics with communication failures"""
        # Set failure conditions
        self.mock_hardware.voltage_data = (False, 0, 0)
        self.mock_hardware.current_data = (False, 0, 0)
        
        initial_errors = self.diagnostic_publisher.hardware_status.communication_errors
        
        # Collect diagnostics
        self.diagnostic_publisher._collect_hardware_diagnostics()
        
        # Should handle failures gracefully
        self.assertGreater(self.diagnostic_publisher.hardware_status.communication_errors, initial_errors)
        
    def test_voltage_diagnostic_generation(self):
        """Test voltage diagnostic message generation"""
        # Set normal voltage
        self.diagnostic_publisher.hardware_status.main_battery_voltage = 12.0
        self.diagnostic_publisher.hardware_status.logic_battery_voltage = 5.0
        
        diagnostics = self.diagnostic_publisher._generate_hardware_diagnostics()
        voltage_diag = next((d for d in diagnostics if 'voltage' in d.name), None)
        
        self.assertIsNotNone(voltage_diag)
        self.assertEqual(voltage_diag.level, 0)  # DiagnosticStatus.OK
        self.assertIn("normal", voltage_diag.message.lower())
        
        # Set low voltage
        self.diagnostic_publisher.hardware_status.main_battery_voltage = 9.0
        
        diagnostics = self.diagnostic_publisher._generate_hardware_diagnostics()
        voltage_diag = next((d for d in diagnostics if 'voltage' in d.name), None)
        
        self.assertEqual(voltage_diag.level, 2)  # DiagnosticStatus.ERROR
        self.assertIn("out of range", voltage_diag.message.lower())
        
    def test_current_diagnostic_generation(self):
        """Test current diagnostic message generation"""
        # Set normal currents
        self.diagnostic_publisher.hardware_status.motor1_current = 2.0
        self.diagnostic_publisher.hardware_status.motor2_current = 3.0
        
        diagnostics = self.diagnostic_publisher._generate_hardware_diagnostics()
        current_diag = next((d for d in diagnostics if 'current' in d.name), None)
        
        self.assertIsNotNone(current_diag)
        self.assertEqual(current_diag.level, 0)  # DiagnosticStatus.OK
        self.assertIn("normal", current_diag.message.lower())
        
        # Set high current
        self.diagnostic_publisher.hardware_status.motor1_current = 12.0
        
        diagnostics = self.diagnostic_publisher._generate_hardware_diagnostics()
        current_diag = next((d for d in diagnostics if 'current' in d.name), None)
        
        self.assertEqual(current_diag.level, 2)  # DiagnosticStatus.ERROR
        self.assertIn("too high", current_diag.message.lower())
        
    def test_temperature_diagnostic_generation(self):
        """Test temperature diagnostic message generation"""
        # Set normal temperatures
        self.diagnostic_publisher.hardware_status.temperature1 = 35.0
        self.diagnostic_publisher.hardware_status.temperature2 = 40.0
        
        diagnostics = self.diagnostic_publisher._generate_hardware_diagnostics()
        temp_diag = next((d for d in diagnostics if 'temperature' in d.name), None)
        
        self.assertIsNotNone(temp_diag)
        self.assertEqual(temp_diag.level, 0)  # DiagnosticStatus.OK
        self.assertIn("normal", temp_diag.message.lower())
        
        # Set high temperature
        self.diagnostic_publisher.hardware_status.temperature1 = 80.0
        
        diagnostics = self.diagnostic_publisher._generate_hardware_diagnostics()
        temp_diag = next((d for d in diagnostics if 'temperature' in d.name), None)
        
        self.assertEqual(temp_diag.level, 2)  # DiagnosticStatus.ERROR
        self.assertIn("too high", temp_diag.message.lower())


@unittest.skipUnless(DIAGNOSTIC_PUBLISHER_AVAILABLE, "Diagnostic publisher not available")
class TestMotionControlDiagnostics(unittest.TestCase):
    """Test motion control status and performance diagnostics"""
    
    def setUp(self):
        """Set up test environment"""
        self.mock_hardware = MockHardwareInterface()
        self.mock_trajectory_monitor = MockTrajectoryMonitor()
        self.mock_buffer_manager = MockBufferManager()
        
        self.diagnostic_publisher = DiagnosticPublisher(
            hardware_interface=self.mock_hardware,
            trajectory_monitor=self.mock_trajectory_monitor,
            buffer_manager=self.mock_buffer_manager
        )
        
    def test_motion_control_diagnostics_collection(self):
        """Test collection of motion control diagnostic data"""
        # Set test values
        self.mock_hardware.position_error_data = (True, -100, 150)
        self.mock_hardware.speed_error_data = (True, -25, 30)
        self.mock_hardware.speed_data = (True, 1000, 1200)
        self.mock_trajectory_monitor.monitoring_status['trajectory_active'] = True
        
        # Collect diagnostics
        self.diagnostic_publisher._collect_motion_control_diagnostics()
        
        # Verify data collection
        self.assertEqual(self.diagnostic_publisher.motion_control_status.position_error_left, -100)
        self.assertEqual(self.diagnostic_publisher.motion_control_status.position_error_right, 150)
        self.assertEqual(self.diagnostic_publisher.motion_control_status.speed_error_left, -25)
        self.assertEqual(self.diagnostic_publisher.motion_control_status.speed_error_right, 30)
        self.assertEqual(self.diagnostic_publisher.motion_control_status.current_left_speed, 1000)
        self.assertEqual(self.diagnostic_publisher.motion_control_status.current_right_speed, 1200)
        self.assertTrue(self.diagnostic_publisher.motion_control_status.trajectory_active)
        
    def test_buffer_status_integration(self):
        """Test integration with buffer manager for buffer status"""
        self.mock_buffer_manager.buffer_status = {'left_buffer': 8, 'right_buffer': 12}
        
        # Collect diagnostics
        self.diagnostic_publisher._collect_motion_control_diagnostics()
        
        # Verify buffer status collection
        self.assertEqual(self.diagnostic_publisher.motion_control_status.buffer_depth_left, 8)
        self.assertEqual(self.diagnostic_publisher.motion_control_status.buffer_depth_right, 12)
        
    def test_position_error_diagnostic_generation(self):
        """Test position error diagnostic message generation"""
        # Set normal position errors
        self.diagnostic_publisher.motion_control_status.position_error_left = 100
        self.diagnostic_publisher.motion_control_status.position_error_right = -150
        
        diagnostics = self.diagnostic_publisher._generate_motion_control_diagnostics()
        pos_error_diag = next((d for d in diagnostics if 'position_error' in d.name), None)
        
        self.assertIsNotNone(pos_error_diag)
        self.assertEqual(pos_error_diag.level, 0)  # DiagnosticStatus.OK
        self.assertIn("normal", pos_error_diag.message.lower())
        
        # Set high position error
        self.diagnostic_publisher.motion_control_status.position_error_left = 1500
        
        diagnostics = self.diagnostic_publisher._generate_motion_control_diagnostics()
        pos_error_diag = next((d for d in diagnostics if 'position_error' in d.name), None)
        
        self.assertEqual(pos_error_diag.level, 2)  # DiagnosticStatus.ERROR
        self.assertIn("too high", pos_error_diag.message.lower())
        
    def test_speed_error_diagnostic_generation(self):
        """Test speed error diagnostic message generation"""
        # Set normal speed errors
        self.diagnostic_publisher.motion_control_status.speed_error_left = 50
        self.diagnostic_publisher.motion_control_status.speed_error_right = -75
        
        diagnostics = self.diagnostic_publisher._generate_motion_control_diagnostics()
        speed_error_diag = next((d for d in diagnostics if 'speed_error' in d.name), None)
        
        self.assertIsNotNone(speed_error_diag)
        self.assertEqual(speed_error_diag.level, 0)  # DiagnosticStatus.OK
        self.assertIn("normal", speed_error_diag.message.lower())
        
        # Set high speed error
        self.diagnostic_publisher.motion_control_status.speed_error_left = 750
        
        diagnostics = self.diagnostic_publisher._generate_motion_control_diagnostics()
        speed_error_diag = next((d for d in diagnostics if 'speed_error' in d.name), None)
        
        self.assertEqual(speed_error_diag.level, 2)  # DiagnosticStatus.ERROR
        self.assertIn("too high", speed_error_diag.message.lower())


@unittest.skipUnless(DIAGNOSTIC_PUBLISHER_AVAILABLE, "Diagnostic publisher not available")
class TestCommunicationDiagnostics(unittest.TestCase):
    """Test communication status tracking and failure detection"""
    
    def setUp(self):
        """Set up test environment"""
        self.mock_hardware = MockHardwareInterface()
        self.diagnostic_publisher = DiagnosticPublisher(hardware_interface=self.mock_hardware)
        
    def test_communication_diagnostics_collection(self):
        """Test collection of communication diagnostic data"""
        # Set test values
        self.mock_hardware.comm_stats = {
            'total_commands': 2000,
            'successful_commands': 1900,
            'failed_commands': 100,
            'timeout_errors': 10
        }
        self.mock_hardware.latency_stats = {
            'average_ms': 20.0,
            'max_ms': 60.0
        }
        
        # Collect diagnostics
        self.diagnostic_publisher._collect_communication_diagnostics()
        
        # Verify data collection
        self.assertEqual(self.diagnostic_publisher.communication_status.total_commands_sent, 2000)
        self.assertEqual(self.diagnostic_publisher.communication_status.successful_commands, 1900)
        self.assertEqual(self.diagnostic_publisher.communication_status.failed_commands, 100)
        self.assertEqual(self.diagnostic_publisher.communication_status.timeout_errors, 10)
        self.assertEqual(self.diagnostic_publisher.communication_status.average_latency_ms, 20.0)
        self.assertEqual(self.diagnostic_publisher.communication_status.max_latency_ms, 60.0)
        
    def test_success_rate_calculation(self):
        """Test communication success rate calculation"""
        # Set initial values
        self.diagnostic_publisher.communication_status.total_commands_sent = 1000
        self.diagnostic_publisher.communication_status.successful_commands = 950
        
        # Collect diagnostics to update success rate history
        self.diagnostic_publisher._collect_communication_diagnostics()
        
        # Verify success rate calculation
        self.assertEqual(len(self.diagnostic_publisher.success_rate_history), 1)
        self.assertEqual(self.diagnostic_publisher.success_rate_history[0], 0.95)
        
    def test_communication_timeout_detection(self):
        """Test communication timeout detection"""
        # Set last successful communication to old time
        old_time = time.time() - 10.0  # 10 seconds ago
        self.diagnostic_publisher.communication_status.last_successful_communication = old_time
        
        # Collect diagnostics
        self.diagnostic_publisher._collect_communication_diagnostics()
        
        # Should detect communication timeout
        self.assertFalse(self.diagnostic_publisher.communication_status.communication_active)
        
    def test_communication_diagnostic_generation(self):
        """Test communication diagnostic message generation"""
        # Set good communication stats
        self.diagnostic_publisher.communication_status.total_commands_sent = 1000
        self.diagnostic_publisher.communication_status.successful_commands = 980
        self.diagnostic_publisher.communication_status.average_latency_ms = 15.0
        self.diagnostic_publisher.communication_status.communication_active = True
        
        diagnostics = self.diagnostic_publisher._generate_communication_diagnostics()
        comm_diag = next((d for d in diagnostics if 'communication' in d.name), None)
        
        self.assertIsNotNone(comm_diag)
        self.assertEqual(comm_diag.level, 0)  # DiagnosticStatus.OK
        self.assertIn("normal", comm_diag.message.lower())
        
        # Set communication timeout
        self.diagnostic_publisher.communication_status.communication_active = False
        
        diagnostics = self.diagnostic_publisher._generate_communication_diagnostics()
        comm_diag = next((d for d in diagnostics if 'communication' in d.name), None)
        
        self.assertEqual(comm_diag.level, 2)  # DiagnosticStatus.ERROR
        self.assertIn("timeout", comm_diag.message.lower())
        
    def test_low_success_rate_detection(self):
        """Test detection of low communication success rate"""
        # Set low success rate
        self.diagnostic_publisher.communication_status.total_commands_sent = 1000
        self.diagnostic_publisher.communication_status.successful_commands = 800  # 80% success rate
        self.diagnostic_publisher.communication_status.communication_active = True
        
        diagnostics = self.diagnostic_publisher._generate_communication_diagnostics()
        comm_diag = next((d for d in diagnostics if 'communication' in d.name), None)
        
        self.assertEqual(comm_diag.level, 2)  # DiagnosticStatus.ERROR
        self.assertIn("success rate too low", comm_diag.message.lower())
        
    def test_high_latency_detection(self):
        """Test detection of high communication latency"""
        # Set high latency
        self.diagnostic_publisher.communication_status.total_commands_sent = 1000
        self.diagnostic_publisher.communication_status.successful_commands = 980
        self.diagnostic_publisher.communication_status.average_latency_ms = 80.0  # High latency
        self.diagnostic_publisher.communication_status.communication_active = True
        
        diagnostics = self.diagnostic_publisher._generate_communication_diagnostics()
        comm_diag = next((d for d in diagnostics if 'communication' in d.name), None)
        
        self.assertEqual(comm_diag.level, 1)  # DiagnosticStatus.WARN
        self.assertIn("latency high", comm_diag.message.lower())


@unittest.skipUnless(DIAGNOSTIC_PUBLISHER_AVAILABLE, "Diagnostic publisher not available")
class TestSafetyDiagnostics(unittest.TestCase):
    """Test safety status monitoring and alerts"""
    
    def setUp(self):
        """Set up test environment"""
        self.mock_hardware = MockHardwareInterface()
        self.diagnostic_publisher = DiagnosticPublisher(hardware_interface=self.mock_hardware)
        
    def test_safety_diagnostics_collection(self):
        """Test collection of safety diagnostic data"""
        # Set test values
        self.mock_hardware.emergency_stop_active = True
        self.mock_hardware.position_limits_enabled = True
        
        # Collect diagnostics
        self.diagnostic_publisher._collect_safety_diagnostics()
        
        # Verify data collection
        self.assertTrue(self.diagnostic_publisher.safety_status.emergency_stop_active)
        self.assertTrue(self.diagnostic_publisher.safety_status.position_limits_enabled)
        
    def test_hardware_safety_analysis(self):
        """Test analysis of hardware status for safety concerns"""
        # Set hardware values that should trigger safety warnings
        self.diagnostic_publisher.hardware_status.main_battery_voltage = 8.0  # Low voltage
        self.diagnostic_publisher.hardware_status.temperature1 = 80.0  # High temperature
        self.diagnostic_publisher.hardware_status.motor1_current = 12.0  # High current
        self.diagnostic_publisher.motion_control_status.position_error_left = 1500  # High position error
        
        initial_voltage_warnings = self.diagnostic_publisher.safety_status.voltage_warnings
        initial_thermal_warnings = self.diagnostic_publisher.safety_status.thermal_warnings
        initial_current_warnings = self.diagnostic_publisher.safety_status.current_warnings
        initial_position_violations = self.diagnostic_publisher.safety_status.position_limit_violations
        
        # Analyze safety
        self.diagnostic_publisher._analyze_hardware_safety()
        
        # Verify safety warnings were incremented
        self.assertGreater(self.diagnostic_publisher.safety_status.voltage_warnings, initial_voltage_warnings)
        self.assertGreater(self.diagnostic_publisher.safety_status.thermal_warnings, initial_thermal_warnings)
        self.assertGreater(self.diagnostic_publisher.safety_status.current_warnings, initial_current_warnings)
        self.assertGreater(self.diagnostic_publisher.safety_status.position_limit_violations, initial_position_violations)
        
    def test_safety_diagnostic_generation_normal(self):
        """Test safety diagnostic generation under normal conditions"""
        # Set normal safety status
        self.diagnostic_publisher.safety_status.emergency_stop_active = False
        self.diagnostic_publisher.safety_status.thermal_warnings = 0
        self.diagnostic_publisher.safety_status.voltage_warnings = 0
        self.diagnostic_publisher.safety_status.current_warnings = 0
        
        diagnostics = self.diagnostic_publisher._generate_safety_diagnostics()
        safety_diag = next((d for d in diagnostics if 'safety' in d.name), None)
        
        self.assertIsNotNone(safety_diag)
        self.assertEqual(safety_diag.level, 0)  # DiagnosticStatus.OK
        self.assertIn("normal", safety_diag.message.lower())
        
    def test_safety_diagnostic_generation_emergency_stop(self):
        """Test safety diagnostic generation with emergency stop active"""
        # Set emergency stop active
        self.diagnostic_publisher.safety_status.emergency_stop_active = True
        
        diagnostics = self.diagnostic_publisher._generate_safety_diagnostics()
        safety_diag = next((d for d in diagnostics if 'safety' in d.name), None)
        
        self.assertEqual(safety_diag.level, 2)  # DiagnosticStatus.ERROR
        self.assertIn("emergency stop active", safety_diag.message.lower())
        
    def test_safety_diagnostic_generation_warnings(self):
        """Test safety diagnostic generation with multiple warnings"""
        # Set multiple warnings
        self.diagnostic_publisher.safety_status.emergency_stop_active = False
        self.diagnostic_publisher.safety_status.thermal_warnings = 3
        self.diagnostic_publisher.safety_status.voltage_warnings = 2
        self.diagnostic_publisher.safety_status.current_warnings = 1
        
        diagnostics = self.diagnostic_publisher._generate_safety_diagnostics()
        safety_diag = next((d for d in diagnostics if 'safety' in d.name), None)
        
        self.assertEqual(safety_diag.level, 1)  # DiagnosticStatus.WARN
        self.assertIn("warnings present", safety_diag.message.lower())
        
    def test_safety_diagnostic_generation_many_warnings(self):
        """Test safety diagnostic generation with many warnings"""
        # Set many warnings (should escalate to ERROR)
        self.diagnostic_publisher.safety_status.emergency_stop_active = False
        self.diagnostic_publisher.safety_status.thermal_warnings = 5
        self.diagnostic_publisher.safety_status.voltage_warnings = 4
        self.diagnostic_publisher.safety_status.current_warnings = 3
        
        diagnostics = self.diagnostic_publisher._generate_safety_diagnostics()
        safety_diag = next((d for d in diagnostics if 'safety' in d.name), None)
        
        self.assertEqual(safety_diag.level, 2)  # DiagnosticStatus.ERROR
        self.assertIn("multiple safety warnings", safety_diag.message.lower())


@unittest.skipUnless(DIAGNOSTIC_PUBLISHER_AVAILABLE, "Diagnostic publisher not available")
class TestDiagnosticDataManagement(unittest.TestCase):
    """Test diagnostic data management and analysis"""
    
    def setUp(self):
        """Set up test environment"""
        self.mock_hardware = MockHardwareInterface()
        self.diagnostic_publisher = DiagnosticPublisher(hardware_interface=self.mock_hardware)
        
    def test_diagnostic_history_update(self):
        """Test diagnostic history update and management"""
        # Set test data
        self.diagnostic_publisher.hardware_status.main_battery_voltage = 12.0
        self.diagnostic_publisher.motion_control_status.trajectory_active = True
        self.diagnostic_publisher.communication_status.total_commands_sent = 1000
        self.diagnostic_publisher.communication_status.successful_commands = 950
        self.diagnostic_publisher.safety_status.emergency_stop_active = False
        
        # Update history
        self.diagnostic_publisher._update_diagnostic_history()
        
        # Verify history was updated
        self.assertEqual(len(self.diagnostic_publisher.diagnostic_history['hardware']), 1)
        self.assertEqual(len(self.diagnostic_publisher.diagnostic_history['motion']), 1)
        self.assertEqual(len(self.diagnostic_publisher.diagnostic_history['communication']), 1)
        self.assertEqual(len(self.diagnostic_publisher.diagnostic_history['safety']), 1)
        
        # Check data content
        hardware_entry = self.diagnostic_publisher.diagnostic_history['hardware'][0]
        self.assertEqual(hardware_entry['battery_voltage'], 12.0)
        
        motion_entry = self.diagnostic_publisher.diagnostic_history['motion'][0]
        self.assertTrue(motion_entry['trajectory_active'])
        
    def test_diagnostic_summary_generation(self):
        """Test comprehensive diagnostic summary generation"""
        # Set test data
        self.diagnostic_publisher.hardware_status.main_battery_voltage = 12.5
        self.diagnostic_publisher.motion_control_status.trajectory_active = True
        self.diagnostic_publisher.communication_status.total_commands_sent = 1000
        self.diagnostic_publisher.communication_status.successful_commands = 980
        self.diagnostic_publisher.safety_status.emergency_stop_active = False
        
        # Generate summary
        summary = self.diagnostic_publisher.get_diagnostic_summary()
        
        # Verify summary structure
        self.assertIn('hardware', summary)
        self.assertIn('motion_control', summary)
        self.assertIn('communication', summary)
        self.assertIn('safety', summary)
        self.assertIn('monitoring', summary)
        
        # Verify data content
        self.assertEqual(summary['hardware']['battery_voltage'], 12.5)
        self.assertTrue(summary['motion_control']['trajectory_active'])
        self.assertEqual(summary['communication']['total_commands'], 1000)
        self.assertFalse(summary['safety']['emergency_stop_active'])
        
    def test_diagnostic_trends_analysis(self):
        """Test diagnostic trends analysis functionality"""
        # Add multiple data points to history
        for i in range(5):
            voltage = 12.0 + i * 0.1  # Increasing voltage trend
            self.diagnostic_publisher.hardware_status.main_battery_voltage = voltage
            self.diagnostic_publisher.communication_status.total_commands_sent = (i + 1) * 100
            self.diagnostic_publisher.communication_status.successful_commands = (i + 1) * 95
            self.diagnostic_publisher._update_diagnostic_history()
            time.sleep(0.01)  # Small delay for different timestamps
            
        # Get trends
        hardware_trends = self.diagnostic_publisher.get_diagnostic_trends('hardware', 1.0)
        communication_trends = self.diagnostic_publisher.get_diagnostic_trends('communication', 1.0)
        
        # Verify trends
        self.assertIn('sample_count', hardware_trends)
        self.assertIn('voltage_trend', hardware_trends)
        self.assertEqual(hardware_trends['sample_count'], 5)
        
        self.assertIn('performance_trend', communication_trends)
        self.assertEqual(communication_trends['sample_count'], 5)
        
    def test_diagnostic_counter_reset(self):
        """Test diagnostic counter reset functionality"""
        # Set some counter values
        self.diagnostic_publisher.hardware_status.communication_errors = 5
        self.diagnostic_publisher.communication_status.total_commands_sent = 1000
        self.diagnostic_publisher.communication_status.failed_commands = 50
        self.diagnostic_publisher.safety_status.thermal_warnings = 3
        
        # Add some history
        self.diagnostic_publisher._update_diagnostic_history()
        
        # Reset counters
        self.diagnostic_publisher.reset_diagnostic_counters()
        
        # Verify reset
        self.assertEqual(self.diagnostic_publisher.hardware_status.communication_errors, 0)
        self.assertEqual(self.diagnostic_publisher.communication_status.total_commands_sent, 0)
        self.assertEqual(self.diagnostic_publisher.communication_status.failed_commands, 0)
        self.assertEqual(self.diagnostic_publisher.safety_status.thermal_warnings, 0)
        
        # Verify history was cleared
        for category in self.diagnostic_publisher.diagnostic_history:
            self.assertEqual(len(self.diagnostic_publisher.diagnostic_history[category]), 0)


@unittest.skipUnless(DIAGNOSTIC_PUBLISHER_AVAILABLE, "Diagnostic publisher not available")
class TestDiagnosticPublisherIntegration(unittest.TestCase):
    """Test diagnostic publisher integration with other components"""
    
    def setUp(self):
        """Set up test environment"""
        self.mock_hardware = MockHardwareInterface()
        self.mock_trajectory_monitor = MockTrajectoryMonitor()
        self.mock_buffer_manager = MockBufferManager()
        
        self.diagnostic_publisher = DiagnosticPublisher(
            hardware_interface=self.mock_hardware,
            trajectory_monitor=self.mock_trajectory_monitor,
            buffer_manager=self.mock_buffer_manager
        )
        
    def test_integration_with_trajectory_monitor(self):
        """Test integration with trajectory monitoring system"""
        # Set trajectory monitor status
        self.mock_trajectory_monitor.monitoring_status = {
            'trajectory_active': True,
            'monitoring_active': True,
            'current_errors': {'position': 100, 'speed': 50}
        }
        
        # Collect diagnostics
        self.diagnostic_publisher._collect_motion_control_diagnostics()
        
        # Verify integration
        self.assertTrue(self.diagnostic_publisher.motion_control_status.trajectory_active)
        
    def test_integration_with_buffer_manager(self):
        """Test integration with buffer management system"""
        # Set buffer status
        self.mock_buffer_manager.buffer_status = {
            'left_buffer': 10,
            'right_buffer': 15
        }
        
        # Collect diagnostics
        self.diagnostic_publisher._collect_motion_control_diagnostics()
        
        # Verify integration
        self.assertEqual(self.diagnostic_publisher.motion_control_status.buffer_depth_left, 10)
        self.assertEqual(self.diagnostic_publisher.motion_control_status.buffer_depth_right, 15)
        
    def test_comprehensive_diagnostic_monitoring_cycle(self):
        """Test complete diagnostic monitoring cycle"""
        # Start monitoring
        result = self.diagnostic_publisher.start_diagnostic_monitoring()
        self.assertTrue(result)
        
        # Let monitoring run for a brief period
        time.sleep(0.2)
        
        # Check that diagnostic data is being collected
        self.assertGreater(self.diagnostic_publisher.hardware_status.last_update, 0)
        self.assertGreater(self.diagnostic_publisher.motion_control_status.last_update, 0)
        
        # Stop monitoring
        result = self.diagnostic_publisher.stop_diagnostic_monitoring()
        self.assertTrue(result)
        
    def test_diagnostic_callback_registration(self):
        """Test diagnostic callback registration and execution"""
        callback_executed = False
        callback_data = None
        
        def test_callback(data):
            nonlocal callback_executed, callback_data
            callback_executed = True
            callback_data = data
            
        # Register callback
        self.diagnostic_publisher.register_diagnostic_callback(DiagnosticCategory.HARDWARE, test_callback)
        
        # Verify callback was registered
        self.assertIn(test_callback, self.diagnostic_publisher.diagnostic_callbacks[DiagnosticCategory.HARDWARE])


# Test runner
if __name__ == '__main__':
    # Create test suite
    suite = unittest.TestSuite()
    
    # Add test classes
    if DIAGNOSTIC_PUBLISHER_AVAILABLE:
        suite.addTest(unittest.makeSuite(TestDiagnosticPublisherCore))
        suite.addTest(unittest.makeSuite(TestHardwareStatusMonitoring))
        suite.addTest(unittest.makeSuite(TestMotionControlDiagnostics))
        suite.addTest(unittest.makeSuite(TestCommunicationDiagnostics))
        suite.addTest(unittest.makeSuite(TestSafetyDiagnostics))
        suite.addTest(unittest.makeSuite(TestDiagnosticDataManagement))
        suite.addTest(unittest.makeSuite(TestDiagnosticPublisherIntegration))
        
        print("Running Diagnostic Publisher tests...")
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        # Print summary
        if result.wasSuccessful():
            print(f"\n✅ All {result.testsRun} diagnostic publisher tests passed!")
        else:
            print(f"\n❌ {len(result.failures)} test(s) failed, {len(result.errors)} error(s)")
            for test, error in result.failures + result.errors:
                print(f"   - {test}: {error.split(':', 1)[0]}")
    else:
        print("❌ Diagnostic publisher not available - skipping tests")
        print("   Make sure the diagnostic_publisher module can be imported")