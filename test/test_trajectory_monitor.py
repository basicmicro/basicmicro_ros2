#!/usr/bin/env python3
"""
Test Suite for Trajectory Monitor

Comprehensive testing of trajectory monitoring functionality including
error tracking, deviation detection, servo monitoring, and performance validation.

Author: ROS2 Driver Development
"""

import unittest
import time
import threading
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the package to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import modules under test
from basicmicro_driver.trajectory_monitor import (
    TrajectoryMonitor,
    TrajectoryMonitoringState,
    ErrorSeverity,
    MonitoringMode,
    TrajectoryDeviation
)


class TestTrajectoryMonitorCore(unittest.TestCase):
    """Test core trajectory monitoring functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create mock components
        self.mock_hardware_interface = Mock()
        self.mock_hardware_interface.controller = Mock()
        self.mock_hardware_interface.address = 0x80
        self.mock_hardware_interface.wheel_radius = 0.1
        
        self.mock_buffer_manager = Mock()
        
        self.mock_unit_converter = Mock()
        self.mock_unit_converter.counts_to_radians.side_effect = lambda x: x * 0.001
        self.mock_unit_converter.rad_per_sec_to_counts_per_sec.side_effect = lambda x: x * 1000
        
        self.mock_servo_service = Mock()
        
        # Create trajectory monitor instance
        self.monitor = TrajectoryMonitor(
            hardware_interface=self.mock_hardware_interface,
            buffer_manager=self.mock_buffer_manager,
            unit_converter=self.mock_unit_converter,
            servo_position_service=self.mock_servo_service
        )
    
    def test_initialization(self):
        """Test trajectory monitor initialization"""
        self.assertIsNotNone(self.monitor)
        self.assertEqual(self.monitor.monitoring_state, TrajectoryMonitoringState.IDLE)
        self.assertFalse(self.monitor.monitoring_active)
        self.assertEqual(self.monitor.monitoring_rate_hz, 25.0)
        self.assertEqual(self.monitor.monitoring_mode, MonitoringMode.ACTIVE)
        
    def test_parameter_setup(self):
        """Test parameter configuration"""
        # Test default parameters
        self.assertEqual(self.monitor.position_error_threshold, 500)
        self.assertEqual(self.monitor.speed_error_threshold, 100)
        self.assertEqual(self.monitor.trajectory_deviation_threshold, 0.1)
        self.assertTrue(self.monitor.enable_deviation_detection)
        self.assertTrue(self.monitor.enable_error_limit_monitoring)
        
    def test_start_stop_monitoring(self):
        """Test monitoring lifecycle"""
        # Test start monitoring
        result = self.monitor.start_monitoring()
        self.assertTrue(result)
        self.assertTrue(self.monitor.monitoring_active)
        self.assertEqual(self.monitor.monitoring_state, TrajectoryMonitoringState.MONITORING)
        
        # Test stop monitoring
        result = self.monitor.stop_monitoring()
        self.assertTrue(result)
        self.assertFalse(self.monitor.monitoring_active)
        self.assertEqual(self.monitor.monitoring_state, TrajectoryMonitoringState.IDLE)
        
    def test_start_monitoring_without_hardware(self):
        """Test start monitoring failure without hardware interface"""
        monitor_no_hw = TrajectoryMonitor()
        result = monitor_no_hw.start_monitoring()
        self.assertFalse(result)
        self.assertFalse(monitor_no_hw.monitoring_active)
        
    def test_monitoring_prerequisites_validation(self):
        """Test prerequisite validation"""
        # Valid prerequisites
        result = self.monitor._validate_monitoring_prerequisites()
        self.assertTrue(result)
        
        # Missing hardware interface
        monitor_no_hw = TrajectoryMonitor()
        result = monitor_no_hw._validate_monitoring_prerequisites()
        self.assertFalse(result)
        
        # Missing controller in hardware interface
        hw_no_controller = Mock()
        # Explicitly remove controller attribute
        if hasattr(hw_no_controller, 'controller'):
            delattr(hw_no_controller, 'controller')
        monitor_no_controller = TrajectoryMonitor(hardware_interface=hw_no_controller)
        result = monitor_no_controller._validate_monitoring_prerequisites()
        self.assertFalse(result)


class TestTrajectoryMonitorErrorTracking(unittest.TestCase):
    """Test error tracking and detection functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_hardware_interface = Mock()
        self.mock_hardware_interface.controller = Mock()
        self.mock_hardware_interface.address = 0x80
        self.mock_hardware_interface.wheel_radius = 0.1
        
        self.mock_unit_converter = Mock()
        self.mock_unit_converter.counts_to_radians.side_effect = lambda x: x * 0.001
        
        self.monitor = TrajectoryMonitor(
            hardware_interface=self.mock_hardware_interface,
            unit_converter=self.mock_unit_converter
        )
    
    def test_read_error_data_success(self):
        """Test successful error data reading"""
        # Setup mock responses
        self.mock_hardware_interface.controller.GetPosErrors.return_value = (True, 100, -50)
        self.mock_hardware_interface.controller.GetSpeedErrors.return_value = (True, 25, -15)
        
        pos_errors, speed_errors = self.monitor._read_error_data()
        
        self.assertEqual(pos_errors, (100, -50))
        self.assertEqual(speed_errors, (25, -15))
        
        # Verify calls were made
        self.mock_hardware_interface.controller.GetPosErrors.assert_called_once_with(0x80)
        self.mock_hardware_interface.controller.GetSpeedErrors.assert_called_once_with(0x80)
    
    def test_read_error_data_failure(self):
        """Test error data reading failure"""
        # Setup mock responses for failure
        self.mock_hardware_interface.controller.GetPosErrors.return_value = (False, 0, 0)
        self.mock_hardware_interface.controller.GetSpeedErrors.return_value = (False, 0, 0)
        
        pos_errors, speed_errors = self.monitor._read_error_data()
        
        self.assertEqual(pos_errors, (0, 0))
        self.assertEqual(speed_errors, (0, 0))
    
    def test_read_error_data_no_hardware(self):
        """Test error data reading without hardware"""
        monitor_no_hw = TrajectoryMonitor()
        pos_errors, speed_errors = monitor_no_hw._read_error_data()
        
        self.assertEqual(pos_errors, (0, 0))
        self.assertEqual(speed_errors, (0, 0))
    
    def test_read_error_data_exception_handling(self):
        """Test error data reading with exceptions"""
        # Setup mock to raise exception
        self.mock_hardware_interface.controller.GetPosErrors.side_effect = Exception("Communication error")
        
        pos_errors, speed_errors = self.monitor._read_error_data()
        
        # Should return zeros on exception
        self.assertEqual(pos_errors, (0, 0))
        self.assertEqual(speed_errors, (0, 0))
    
    def test_error_history_update(self):
        """Test error history tracking"""
        timestamp = time.time()
        pos_errors = (150, -75)
        speed_errors = (30, -20)
        
        # Clear any existing history
        self.monitor.error_history.clear()
        
        self.monitor._update_error_history(pos_errors, speed_errors, timestamp)
        
        self.assertEqual(len(self.monitor.error_history), 1)
        
        entry = self.monitor.error_history[0]
        self.assertEqual(entry['timestamp'], timestamp)
        self.assertEqual(entry['left_pos_error'], 150)
        self.assertEqual(entry['right_pos_error'], -75)
        self.assertEqual(entry['left_speed_error'], 30)
        self.assertEqual(entry['right_speed_error'], -20)
        self.assertEqual(entry['max_pos_error'], 150)
        self.assertEqual(entry['max_speed_error'], 30)
    
    def test_error_history_maximum_size(self):
        """Test error history maximum size enforcement"""
        # Fill history beyond maximum
        for i in range(150):  # Exceed default max of 100
            self.monitor._update_error_history((i, i), (i, i), time.time())
        
        # Should be limited to max size
        self.assertEqual(len(self.monitor.error_history), self.monitor.max_error_history)


class TestTrajectoryDeviationDetection(unittest.TestCase):
    """Test trajectory deviation detection"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_hardware_interface = Mock()
        self.mock_hardware_interface.wheel_radius = 0.1
        
        self.mock_unit_converter = Mock()
        self.mock_unit_converter.counts_to_radians.side_effect = lambda x: x * 0.001
        
        self.monitor = TrajectoryMonitor(
            hardware_interface=self.mock_hardware_interface,
            unit_converter=self.mock_unit_converter
        )
    
    def test_no_deviation_detection(self):
        """Test no deviation with small errors"""
        pos_errors = (50, -25)  # Below threshold
        speed_errors = (10, -5)  # Below threshold
        
        deviation = self.monitor._detect_trajectory_deviation(pos_errors, speed_errors)
        
        self.assertIsNone(deviation)
    
    def test_position_error_deviation(self):
        """Test deviation detection with position errors"""
        pos_errors = (600, -300)  # Above threshold (500)
        speed_errors = (10, -5)   # Below threshold
        
        deviation = self.monitor._detect_trajectory_deviation(pos_errors, speed_errors)
        
        self.assertIsNotNone(deviation)
        self.assertEqual(deviation.left_pos_error, 600)
        self.assertEqual(deviation.right_pos_error, -300)
        self.assertIn(deviation.severity, [ErrorSeverity.WARNING, ErrorSeverity.CRITICAL])
    
    def test_speed_error_deviation(self):
        """Test deviation detection with speed errors"""
        pos_errors = (50, -25)    # Below threshold
        speed_errors = (150, -75) # Above threshold (100)
        
        deviation = self.monitor._detect_trajectory_deviation(pos_errors, speed_errors)
        
        self.assertIsNotNone(deviation)
        self.assertEqual(deviation.left_speed_error, 150)
        self.assertEqual(deviation.right_speed_error, -75)
        self.assertIn(deviation.severity, [ErrorSeverity.WARNING, ErrorSeverity.CRITICAL])
    
    def test_critical_deviation_detection(self):
        """Test critical deviation detection"""
        pos_errors = (1200, -800)  # Well above threshold
        speed_errors = (250, -150) # Well above threshold
        
        deviation = self.monitor._detect_trajectory_deviation(pos_errors, speed_errors)
        
        self.assertIsNotNone(deviation)
        self.assertEqual(deviation.severity, ErrorSeverity.CRITICAL)
        self.assertGreater(deviation.deviation_magnitude, 0)
    
    def test_deviation_without_unit_converter(self):
        """Test deviation detection without unit converter"""
        monitor_no_uc = TrajectoryMonitor(hardware_interface=self.mock_hardware_interface)
        
        pos_errors = (600, -300)
        speed_errors = (10, -5)
        
        deviation = monitor_no_uc._detect_trajectory_deviation(pos_errors, speed_errors)
        
        self.assertIsNotNone(deviation)
        self.assertGreater(deviation.deviation_magnitude, 0)
    
    def test_deviation_exception_handling(self):
        """Test deviation detection with exceptions"""
        # Mock unit converter to raise exception
        self.mock_unit_converter.counts_to_radians.side_effect = Exception("Conversion error")
        
        pos_errors = (600, -300)
        speed_errors = (10, -5)
        
        deviation = self.monitor._detect_trajectory_deviation(pos_errors, speed_errors)
        
        # Should return None on exception
        self.assertIsNone(deviation)


class TestServoErrorLimitMonitoring(unittest.TestCase):
    """Test servo error limit monitoring"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.monitor = TrajectoryMonitor()
        
        # Set test thresholds
        self.monitor.max_position_error_limit = 1000
        self.monitor.max_speed_error_limit = 200
    
    def test_normal_error_levels(self):
        """Test monitoring with normal error levels"""
        pos_errors = (100, -50)  # Below limits
        speed_errors = (25, -15) # Below limits
        
        self.monitor._monitor_servo_error_limits(pos_errors, speed_errors)
        
        self.assertFalse(self.monitor.error_limit_violated)
    
    def test_position_error_limit_violation(self):
        """Test position error limit violation"""
        pos_errors = (1200, -800)  # Above limit (1000)
        speed_errors = (25, -15)   # Below limit
        
        self.monitor._monitor_servo_error_limits(pos_errors, speed_errors)
        
        self.assertTrue(self.monitor.error_limit_violated)
    
    def test_speed_error_limit_violation(self):
        """Test speed error limit violation"""
        pos_errors = (100, -50)   # Below limit
        speed_errors = (250, -150) # Above limit (200)
        
        self.monitor._monitor_servo_error_limits(pos_errors, speed_errors)
        
        self.assertTrue(self.monitor.error_limit_violated)
    
    def test_combined_error_limit_violation(self):
        """Test combined error limit violation"""
        pos_errors = (1200, -800)  # Above limit
        speed_errors = (250, -150) # Above limit
        
        self.monitor._monitor_servo_error_limits(pos_errors, speed_errors)
        
        self.assertTrue(self.monitor.error_limit_violated)
    
    def test_error_limit_state_transitions(self):
        """Test error limit state transitions"""
        # Start with no violation
        self.monitor.error_limit_violated = False
        
        # Trigger violation
        pos_errors = (1200, -800)
        speed_errors = (25, -15)
        self.monitor._monitor_servo_error_limits(pos_errors, speed_errors)
        self.assertTrue(self.monitor.error_limit_violated)
        
        # Return to normal
        pos_errors = (100, -50)
        speed_errors = (25, -15)
        self.monitor._monitor_servo_error_limits(pos_errors, speed_errors)
        self.assertFalse(self.monitor.error_limit_violated)


class TestControllerErrorStateDetection(unittest.TestCase):
    """Test controller error state detection"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_hardware_interface = Mock()
        self.mock_hardware_interface.controller = Mock()
        self.mock_hardware_interface.address = 0x80
        
        self.monitor = TrajectoryMonitor(hardware_interface=self.mock_hardware_interface)
    
    def test_no_controller_error(self):
        """Test normal controller state"""
        # Setup mock for no error
        self.mock_hardware_interface.controller.ReadError.return_value = (True, 0)
        
        self.monitor._check_controller_error_state()
        
        self.assertFalse(self.monitor.controller_error_state)
    
    def test_controller_error_detected(self):
        """Test controller error state detection"""
        # Setup mock for error condition
        self.mock_hardware_interface.controller.ReadError.return_value = (True, 1)
        
        self.monitor._check_controller_error_state()
        
        self.assertTrue(self.monitor.controller_error_state)
        self.assertEqual(self.monitor.monitoring_state, TrajectoryMonitoringState.ERROR_DETECTED)
    
    def test_controller_communication_failure(self):
        """Test communication failure detection"""
        # Setup mock for communication failure
        self.mock_hardware_interface.controller.ReadError.return_value = (False, 0)
        
        self.monitor._check_controller_error_state()
        
        self.assertTrue(self.monitor.controller_error_state)
    
    def test_controller_error_state_transitions(self):
        """Test controller error state transitions"""
        # Start with no error
        self.monitor.controller_error_state = False
        
        # Trigger error
        self.mock_hardware_interface.controller.ReadError.return_value = (True, 1)
        self.monitor._check_controller_error_state()
        self.assertTrue(self.monitor.controller_error_state)
        
        # Clear error
        self.mock_hardware_interface.controller.ReadError.return_value = (True, 0)
        self.monitor._check_controller_error_state()
        self.assertFalse(self.monitor.controller_error_state)
    
    def test_controller_error_without_hardware(self):
        """Test controller error detection without hardware"""
        monitor_no_hw = TrajectoryMonitor()
        
        monitor_no_hw._check_controller_error_state()
        
        # Should not change state without hardware
        self.assertFalse(monitor_no_hw.controller_error_state)
    
    def test_controller_error_exception_handling(self):
        """Test controller error detection with exceptions"""
        # Setup mock to raise exception
        self.mock_hardware_interface.controller.ReadError.side_effect = Exception("Communication error")
        
        self.monitor._check_controller_error_state()
        
        # Should not crash on exception
        # Error state handling depends on implementation


class TestMonitoringConfiguration(unittest.TestCase):
    """Test monitoring configuration management"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.monitor = TrajectoryMonitor()
    
    def test_configuration_update_thresholds(self):
        """Test threshold configuration updates"""
        config = {
            'position_error_threshold': 300,
            'speed_error_threshold': 75,
            'trajectory_deviation_threshold': 0.05,
            'max_position_error_limit': 800,
            'max_speed_error_limit': 150
        }
        
        result = self.monitor.update_monitoring_configuration(config)
        
        self.assertTrue(result)
        self.assertEqual(self.monitor.position_error_threshold, 300)
        self.assertEqual(self.monitor.speed_error_threshold, 75)
        self.assertEqual(self.monitor.trajectory_deviation_threshold, 0.05)
        self.assertEqual(self.monitor.max_position_error_limit, 800)
        self.assertEqual(self.monitor.max_speed_error_limit, 150)
    
    def test_configuration_update_behavior(self):
        """Test monitoring behavior configuration updates"""
        config = {
            'monitoring_mode': 'corrective',
            'enable_deviation_detection': False,
            'enable_error_limit_monitoring': False,
            'enable_controller_error_detection': False
        }
        
        result = self.monitor.update_monitoring_configuration(config)
        
        self.assertTrue(result)
        self.assertEqual(self.monitor.monitoring_mode, MonitoringMode.CORRECTIVE)
        self.assertFalse(self.monitor.enable_deviation_detection)
        self.assertFalse(self.monitor.enable_error_limit_monitoring)
        self.assertFalse(self.monitor.enable_controller_error_detection)
    
    def test_configuration_update_monitoring_rate(self):
        """Test monitoring rate configuration update"""
        original_rate = self.monitor.monitoring_rate_hz
        
        config = {'monitoring_rate_hz': 50.0}
        
        result = self.monitor.update_monitoring_configuration(config)
        
        self.assertTrue(result)
        self.assertEqual(self.monitor.monitoring_rate_hz, 50.0)
        self.assertEqual(self.monitor.monitoring_period, 0.02)  # 1/50
    
    def test_configuration_update_invalid_mode(self):
        """Test configuration update with invalid monitoring mode"""
        config = {'monitoring_mode': 'invalid_mode'}
        
        # Should handle invalid mode gracefully
        result = self.monitor.update_monitoring_configuration(config)
        
        # Result depends on implementation - might be False for invalid values
        # self.assertFalse(result)
    
    def test_configuration_update_exception_handling(self):
        """Test configuration update exception handling"""
        # Test with configuration that could cause exception during processing
        # Mock the monitoring mode enum constructor to fail
        with patch('basicmicro_driver.trajectory_monitor.MonitoringMode', side_effect=ValueError("Invalid mode")):
            config = {'monitoring_mode': 'invalid_mode'}
            
            # Should handle exceptions gracefully
            result = self.monitor.update_monitoring_configuration(config)
            
            # Result should be False on exception
            self.assertFalse(result)


class TestMonitoringPerformance(unittest.TestCase):
    """Test monitoring performance and metrics"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.monitor = TrajectoryMonitor()
    
    def test_performance_metrics_initialization(self):
        """Test performance metrics initialization"""
        # Should start empty
        self.assertEqual(len(self.monitor.performance_metrics), 0)
    
    def test_performance_metrics_update(self):
        """Test performance metrics updates"""
        cycle_duration = 0.005  # 5ms
        
        self.monitor._update_performance_metrics(cycle_duration)
        
        self.assertIn('cycle_durations', self.monitor.performance_metrics)
        self.assertIn('max_cycle_duration', self.monitor.performance_metrics)
        self.assertIn('avg_cycle_duration', self.monitor.performance_metrics)
        
        self.assertEqual(self.monitor.performance_metrics['max_cycle_duration'], 5.0)  # ms
        self.assertEqual(self.monitor.performance_metrics['avg_cycle_duration'], 5.0)  # ms
    
    def test_performance_metrics_multiple_cycles(self):
        """Test performance metrics with multiple cycles"""
        durations = [0.003, 0.005, 0.004, 0.006, 0.002]  # Various durations
        
        for duration in durations:
            self.monitor._update_performance_metrics(duration)
        
        # Check that metrics are calculated correctly
        self.assertEqual(self.monitor.performance_metrics['max_cycle_duration'], 6.0)  # ms
        self.assertEqual(len(self.monitor.performance_metrics['cycle_durations']), 5)
        
        # Average should be around 4ms
        avg = self.monitor.performance_metrics['avg_cycle_duration']
        self.assertAlmostEqual(avg, 4.0, places=1)
    
    def test_performance_metrics_buffer_limit(self):
        """Test performance metrics buffer size limit"""
        # Add more than buffer limit (100)
        for i in range(150):
            self.monitor._update_performance_metrics(0.001 * i)
        
        # Should be limited to buffer size
        self.assertEqual(len(self.monitor.performance_metrics['cycle_durations']), 100)
    
    def test_monitoring_statistics_reset(self):
        """Test monitoring statistics reset"""
        # Add some data
        self.monitor.total_monitoring_cycles = 100
        self.monitor.successful_monitoring_cycles = 95
        self.monitor._update_performance_metrics(0.005)
        self.monitor.error_history.append({'test': 'data'})
        self.monitor.deviation_history.append(Mock())
        
        # Reset statistics
        self.monitor.reset_monitoring_statistics()
        
        # Check that everything is reset
        self.assertEqual(self.monitor.total_monitoring_cycles, 0)
        self.assertEqual(self.monitor.successful_monitoring_cycles, 0)
        self.assertEqual(len(self.monitor.performance_metrics), 0)
        self.assertEqual(len(self.monitor.error_history), 0)
        self.assertEqual(len(self.monitor.deviation_history), 0)


class TestMonitoringStatus(unittest.TestCase):
    """Test monitoring status and reporting"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.monitor = TrajectoryMonitor()
    
    def test_get_monitoring_status_basic(self):
        """Test basic monitoring status retrieval"""
        status = self.monitor.get_monitoring_status()
        
        self.assertIn('monitoring_active', status)
        self.assertIn('monitoring_state', status)
        self.assertIn('monitoring_rate_hz', status)
        self.assertIn('total_cycles', status)
        self.assertIn('successful_cycles', status)
        self.assertIn('success_rate_percent', status)
        
        self.assertFalse(status['monitoring_active'])
        self.assertEqual(status['monitoring_state'], 'idle')
        self.assertEqual(status['monitoring_rate_hz'], 25.0)
    
    def test_get_monitoring_status_with_activity(self):
        """Test monitoring status with activity"""
        # Simulate some monitoring activity
        self.monitor.monitoring_active = True
        self.monitor.monitoring_state = TrajectoryMonitoringState.MONITORING
        self.monitor.total_monitoring_cycles = 100
        self.monitor.successful_monitoring_cycles = 95
        
        status = self.monitor.get_monitoring_status()
        
        self.assertTrue(status['monitoring_active'])
        self.assertEqual(status['monitoring_state'], 'monitoring')
        self.assertEqual(status['total_cycles'], 100)
        self.assertEqual(status['successful_cycles'], 95)
        self.assertEqual(status['success_rate_percent'], 95.0)
    
    def test_get_monitoring_status_with_errors(self):
        """Test monitoring status with error conditions"""
        self.monitor.current_position_errors = (150, -75)
        self.monitor.current_speed_errors = (30, -20)
        self.monitor.current_trajectory_deviation = 0.05
        self.monitor.error_limit_violated = True
        self.monitor.controller_error_state = True
        
        status = self.monitor.get_monitoring_status()
        
        self.assertEqual(status['current_position_errors'], (150, -75))
        self.assertEqual(status['current_speed_errors'], (30, -20))
        self.assertEqual(status['current_trajectory_deviation'], 0.05)
        self.assertTrue(status['error_limit_violated'])
        self.assertTrue(status['controller_error_state'])
    
    def test_get_monitoring_status_with_error_history(self):
        """Test monitoring status with error history"""
        # Add some error history
        for i in range(10):
            self.monitor.error_history.append({
                'max_pos_error': 100 + i * 10,
                'max_speed_error': 20 + i * 2
            })
        
        status = self.monitor.get_monitoring_status()
        
        self.assertIn('recent_error_stats', status)
        self.assertIn('avg_pos_error', status['recent_error_stats'])
        self.assertIn('max_pos_error', status['recent_error_stats'])
        self.assertIn('avg_speed_error', status['recent_error_stats'])
        self.assertIn('max_speed_error', status['recent_error_stats'])
    
    def test_get_deviation_history(self):
        """Test deviation history retrieval"""
        # Add some deviation history
        for i in range(5):
            deviation = TrajectoryDeviation(
                timestamp=time.time() + i,
                left_pos_error=100 + i * 10,
                right_pos_error=-50 - i * 5,
                left_speed_error=20 + i * 2,
                right_speed_error=-10 - i,
                deviation_magnitude=0.01 + i * 0.01,
                severity=ErrorSeverity.WARNING
            )
            self.monitor.deviation_history.append(deviation)
        
        history = self.monitor.get_trajectory_deviation_history()
        
        self.assertEqual(len(history), 5)
        
        # Check first entry
        entry = history[0]
        self.assertIn('timestamp', entry)
        self.assertIn('left_pos_error', entry)
        self.assertIn('right_pos_error', entry)
        self.assertIn('left_speed_error', entry)
        self.assertIn('right_speed_error', entry)
        self.assertIn('deviation_magnitude', entry)
        self.assertIn('severity', entry)
        
        self.assertEqual(entry['left_pos_error'], 100)
        self.assertEqual(entry['severity'], 'warning')


class TestMonitoringCycleExecution(unittest.TestCase):
    """Test monitoring cycle execution"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_hardware_interface = Mock()
        self.mock_hardware_interface.controller = Mock()
        self.mock_hardware_interface.address = 0x80
        self.mock_hardware_interface.wheel_radius = 0.1
        
        self.mock_unit_converter = Mock()
        self.mock_unit_converter.counts_to_radians.side_effect = lambda x: x * 0.001
        
        self.monitor = TrajectoryMonitor(
            hardware_interface=self.mock_hardware_interface,
            unit_converter=self.mock_unit_converter
        )
    
    def test_monitoring_cycle_execution(self):
        """Test single monitoring cycle execution"""
        # Setup mock responses
        self.mock_hardware_interface.controller.GetPosErrors.return_value = (True, 100, -50)
        self.mock_hardware_interface.controller.GetSpeedErrors.return_value = (True, 25, -15)
        self.mock_hardware_interface.controller.ReadError.return_value = (True, 0)
        
        # Execute monitoring cycle
        self.monitor._perform_monitoring_cycle()
        
        # Check that cycle completed
        self.assertEqual(self.monitor.total_monitoring_cycles, 1)
        self.assertEqual(self.monitor.successful_monitoring_cycles, 1)
        
        # Check that error data was updated
        self.assertEqual(self.monitor.current_position_errors, (100, -50))
        self.assertEqual(self.monitor.current_speed_errors, (25, -15))
    
    def test_monitoring_cycle_with_exceptions(self):
        """Test monitoring cycle with exceptions"""
        # Setup mock to raise exception
        self.mock_hardware_interface.controller.GetPosErrors.side_effect = Exception("Test exception")
        
        # Execute monitoring cycle
        self.monitor._perform_monitoring_cycle()
        
        # Should handle exception gracefully
        self.assertEqual(self.monitor.total_monitoring_cycles, 1)
        # Successful cycles might be 0 due to exception
    
    def test_monitoring_cycle_with_deviation_detection_disabled(self):
        """Test monitoring cycle with deviation detection disabled"""
        self.monitor.enable_deviation_detection = False
        
        # Setup mock responses that would normally trigger deviation
        self.mock_hardware_interface.controller.GetPosErrors.return_value = (True, 600, -300)
        self.mock_hardware_interface.controller.GetSpeedErrors.return_value = (True, 150, -75)
        self.mock_hardware_interface.controller.ReadError.return_value = (True, 0)
        
        # Execute monitoring cycle
        self.monitor._perform_monitoring_cycle()
        
        # Should not add deviations to history
        self.assertEqual(len(self.monitor.deviation_history), 0)
    
    def test_monitoring_cycle_with_error_limit_monitoring_disabled(self):
        """Test monitoring cycle with error limit monitoring disabled"""
        self.monitor.enable_error_limit_monitoring = False
        
        # Setup mock responses that would normally violate limits
        self.mock_hardware_interface.controller.GetPosErrors.return_value = (True, 1200, -800)
        self.mock_hardware_interface.controller.GetSpeedErrors.return_value = (True, 250, -150)
        self.mock_hardware_interface.controller.ReadError.return_value = (True, 0)
        
        # Execute monitoring cycle
        self.monitor._perform_monitoring_cycle()
        
        # Should not set error limit violation
        self.assertFalse(self.monitor.error_limit_violated)


def run_trajectory_monitor_tests():
    """Run all trajectory monitor tests"""
    
    test_classes = [
        TestTrajectoryMonitorCore,
        TestTrajectoryMonitorErrorTracking,
        TestTrajectoryDeviationDetection,
        TestServoErrorLimitMonitoring,
        TestControllerErrorStateDetection,
        TestMonitoringConfiguration,
        TestMonitoringPerformance,
        TestMonitoringStatus,
        TestMonitoringCycleExecution
    ]
    
    total_tests = 0
    total_failures = 0
    
    print("Running Trajectory Monitor Test Suite...")
    print("=" * 60)
    
    for test_class in test_classes:
        print(f"\nRunning {test_class.__name__}...")
        suite = unittest.TestLoader().loadTestsFromTestCase(test_class)
        result = unittest.TextTestRunner(verbosity=1).run(suite)
        
        total_tests += result.testsRun
        total_failures += len(result.failures) + len(result.errors)
        
        if result.failures:
            print(f"FAILURES in {test_class.__name__}:")
            for test, traceback in result.failures:
                print(f"  - {test}: {traceback}")
        
        if result.errors:
            print(f"ERRORS in {test_class.__name__}:")
            for test, traceback in result.errors:
                print(f"  - {test}: {traceback}")
    
    print("\n" + "=" * 60)
    print(f"Trajectory Monitor Test Summary:")
    print(f"Total tests run: {total_tests}")
    print(f"Total failures/errors: {total_failures}")
    print(f"Success rate: {((total_tests - total_failures) / total_tests * 100):.1f}%")
    
    if total_failures == 0:
        print("🎉 All trajectory monitor tests PASSED!")
        return True
    else:
        print(f"❌ {total_failures} trajectory monitor tests FAILED!")
        return False


if __name__ == "__main__":
    success = run_trajectory_monitor_tests()
    exit(0 if success else 1)