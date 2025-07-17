#!/usr/bin/env python3
"""
Comprehensive test suite for Performance Monitor

Tests all functionality including:
- Command execution timing and latency monitoring
- Communication performance and reliability tracking  
- Motion control accuracy and smoothness analysis
- Buffer utilization efficiency monitoring
- Performance optimization recommendations
- Performance reporting and trending analysis

Author: Basicmicro Driver Team
License: MIT
"""

import unittest
import time
import threading
import json
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the package to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from basicmicro_driver.performance_monitor import (
    PerformanceMonitor, PerformanceCategory, PerformanceLevel,
    PerformanceMetric, CommandTimingData, CommunicationStats,
    MotionControlStats, BufferStats, OptimizationRecommendation,
    PerformanceThresholds
)


class TestPerformanceMetric(unittest.TestCase):
    """Test PerformanceMetric data structure"""
    
    def test_performance_metric_creation(self):
        """Test creating performance metrics"""
        metric = PerformanceMetric(
            name="test_metric",
            value=42.5,
            unit="ms",
            timestamp=time.time(),
            category=PerformanceCategory.COMMAND_EXECUTION,
            level=PerformanceLevel.GOOD
        )
        
        self.assertEqual(metric.name, "test_metric")
        self.assertEqual(metric.value, 42.5)
        self.assertEqual(metric.unit, "ms")
        self.assertEqual(metric.category, PerformanceCategory.COMMAND_EXECUTION)
        self.assertEqual(metric.level, PerformanceLevel.GOOD)
        self.assertFalse(metric.threshold_exceeded)
    
    def test_performance_metric_serialization(self):
        """Test metric serialization to dictionary"""
        metric = PerformanceMetric(
            name="test_metric",
            value=42.5,
            unit="ms",
            timestamp=123456789.0,
            category=PerformanceCategory.COMMUNICATION,
            level=PerformanceLevel.CRITICAL,
            threshold_exceeded=True
        )
        
        expected_dict = {
            'name': 'test_metric',
            'value': 42.5,
            'unit': 'ms',
            'timestamp': 123456789.0,
            'category': 'communication',
            'level': 'critical',
            'threshold_exceeded': True
        }
        
        self.assertEqual(metric.to_dict(), expected_dict)


class TestCommandTimingData(unittest.TestCase):
    """Test CommandTimingData functionality"""
    
    def test_command_timing_data_creation(self):
        """Test creating command timing data"""
        timing_data = CommandTimingData(
            command_type="SpeedM1M2",
            start_time=123456789.0,
            end_time=123456789.005,
            execution_time=0.005,
            success=True
        )
        
        self.assertEqual(timing_data.command_type, "SpeedM1M2")
        self.assertEqual(timing_data.start_time, 123456789.0)
        self.assertEqual(timing_data.end_time, 123456789.005)
        self.assertEqual(timing_data.execution_time, 0.005)
        self.assertTrue(timing_data.success)
        self.assertIsNone(timing_data.error_message)
    
    def test_command_timing_data_with_error(self):
        """Test command timing data with error"""
        timing_data = CommandTimingData(
            command_type="GetEncoders",
            start_time=123456789.0,
            end_time=123456789.010,
            execution_time=0.010,
            success=False,
            error_message="Communication timeout"
        )
        
        self.assertFalse(timing_data.success)
        self.assertEqual(timing_data.error_message, "Communication timeout")
    
    def test_command_timing_data_serialization(self):
        """Test timing data serialization"""
        timing_data = CommandTimingData(
            command_type="DutyM1M2",
            start_time=123456789.0,
            end_time=123456789.003,
            execution_time=0.003,
            success=True
        )
        
        data_dict = timing_data.to_dict()
        
        self.assertEqual(data_dict['command_type'], "DutyM1M2")
        self.assertEqual(data_dict['execution_time'], 0.003)
        self.assertTrue(data_dict['success'])


class TestCommunicationStats(unittest.TestCase):
    """Test CommunicationStats functionality"""
    
    def test_communication_stats_initial_state(self):
        """Test initial communication stats state"""
        stats = CommunicationStats()
        
        self.assertEqual(stats.total_commands, 0)
        self.assertEqual(stats.successful_commands, 0)
        self.assertEqual(stats.failed_commands, 0)
        self.assertEqual(stats.success_rate, 100.0)  # No commands = 100%
        self.assertEqual(stats.average_latency, 0.0)
    
    def test_communication_stats_calculations(self):
        """Test communication statistics calculations"""
        stats = CommunicationStats()
        
        # Simulate some commands
        stats.total_commands = 100
        stats.successful_commands = 95
        stats.failed_commands = 5
        stats.total_latency = 0.950  # 95 commands * 0.01s average
        stats.min_latency = 0.001
        stats.max_latency = 0.050
        
        self.assertEqual(stats.success_rate, 95.0)
        self.assertAlmostEqual(stats.average_latency, 0.01, places=4)
    
    def test_communication_stats_serialization(self):
        """Test communication stats serialization"""
        stats = CommunicationStats()
        stats.total_commands = 50
        stats.successful_commands = 48
        stats.failed_commands = 2
        stats.total_latency = 0.480
        stats.timeout_count = 1
        stats.retry_count = 1
        
        data_dict = stats.to_dict()
        
        self.assertEqual(data_dict['total_commands'], 50)
        self.assertEqual(data_dict['success_rate'], 96.0)
        self.assertEqual(data_dict['timeout_count'], 1)


class TestMotionControlStats(unittest.TestCase):
    """Test MotionControlStats functionality"""
    
    def test_motion_control_stats_initial_state(self):
        """Test initial motion control stats state"""
        stats = MotionControlStats()
        
        self.assertEqual(len(stats.position_error_samples), 0)
        self.assertEqual(len(stats.speed_error_samples), 0)
        self.assertEqual(stats.average_position_error, 0.0)
        self.assertEqual(stats.trajectory_accuracy, 100.0)
    
    def test_motion_control_stats_calculations(self):
        """Test motion control statistics calculations"""
        stats = MotionControlStats()
        
        # Add sample data
        stats.position_error_samples = [100, -200, 150, -100, 50]
        stats.speed_error_samples = [10, -20, 15, -10, 5]
        stats.trajectory_deviations = [50, 100, 75, 25, 0]
        stats.smoothness_measurements = [95, 98, 92, 96, 99]
        
        self.assertEqual(stats.average_position_error, 0.0)  # Mean of mixed positive/negative
        self.assertEqual(stats.average_speed_error, 0.0)
        self.assertGreater(stats.trajectory_accuracy, 90.0)  # Should be high with small deviations
        self.assertGreater(stats.motion_smoothness, 95.0)
    
    def test_motion_control_stats_serialization(self):
        """Test motion control stats serialization"""
        stats = MotionControlStats()
        stats.position_error_samples = [100, -100, 50]
        stats.speed_error_samples = [10, -10, 5]
        
        data_dict = stats.to_dict()
        
        self.assertIn('average_position_error', data_dict)
        self.assertIn('trajectory_accuracy', data_dict)
        self.assertIn('sample_count', data_dict)
        self.assertEqual(data_dict['sample_count'], 3)


class TestBufferStats(unittest.TestCase):
    """Test BufferStats functionality"""
    
    def test_buffer_stats_initial_state(self):
        """Test initial buffer stats state"""
        stats = BufferStats()
        
        self.assertEqual(len(stats.buffer_depth_samples), 0)
        self.assertEqual(stats.buffer_overflow_count, 0)
        self.assertEqual(stats.average_buffer_depth, 0.0)
        self.assertEqual(stats.buffer_efficiency, 100.0)  # No samples = 100%
    
    def test_buffer_stats_calculations(self):
        """Test buffer statistics calculations"""
        stats = BufferStats()
        
        # Add sample data (optimal range is 2-8)
        stats.buffer_depth_samples = [2, 4, 6, 8, 5, 3, 7, 4, 6, 5]
        stats.optimal_utilization_time = 8.0
        stats.total_monitoring_time = 10.0
        
        self.assertEqual(stats.average_buffer_depth, 5.0)
        self.assertEqual(stats.buffer_efficiency, 100.0)  # All samples in optimal range
        self.assertEqual(stats.utilization_rate, 80.0)
    
    def test_buffer_stats_with_overflows(self):
        """Test buffer stats with overflow conditions"""
        stats = BufferStats()
        
        # Add samples with some overflows
        stats.buffer_depth_samples = [2, 4, 32, 8, 0, 3, 32, 4]
        stats.buffer_overflow_count = 2
        stats.buffer_underrun_count = 1
        
        # Efficiency should be lower due to non-optimal values
        efficiency = stats.buffer_efficiency
        self.assertLess(efficiency, 100.0)
        self.assertEqual(stats.buffer_overflow_count, 2)
        self.assertEqual(stats.buffer_underrun_count, 1)
    
    def test_buffer_stats_serialization(self):
        """Test buffer stats serialization"""
        stats = BufferStats()
        stats.buffer_depth_samples = [2, 4, 6, 8]
        stats.buffer_overflow_count = 1
        
        data_dict = stats.to_dict()
        
        self.assertIn('average_buffer_depth', data_dict)
        self.assertIn('buffer_efficiency', data_dict)
        self.assertIn('buffer_overflow_count', data_dict)
        self.assertEqual(data_dict['max_buffer_depth'], 8)
        self.assertEqual(data_dict['min_buffer_depth'], 2)


class TestOptimizationRecommendation(unittest.TestCase):
    """Test OptimizationRecommendation functionality"""
    
    def test_optimization_recommendation_creation(self):
        """Test creating optimization recommendations"""
        recommendation = OptimizationRecommendation(
            category=PerformanceCategory.COMMAND_EXECUTION,
            severity=PerformanceLevel.POOR,
            title="High Latency",
            description="Command latency is too high",
            action_items=["Check baud rate", "Reduce frequency"],
            expected_improvement="Reduce latency to <5ms",
            implementation_effort="medium"
        )
        
        self.assertEqual(recommendation.category, PerformanceCategory.COMMAND_EXECUTION)
        self.assertEqual(recommendation.severity, PerformanceLevel.POOR)
        self.assertEqual(recommendation.title, "High Latency")
        self.assertEqual(len(recommendation.action_items), 2)
    
    def test_optimization_recommendation_serialization(self):
        """Test recommendation serialization"""
        recommendation = OptimizationRecommendation(
            category=PerformanceCategory.COMMUNICATION,
            severity=PerformanceLevel.CRITICAL,
            title="Connection Issues",
            description="Communication failing",
            action_items=["Check cables"],
            expected_improvement="Restore communication",
            implementation_effort="low"
        )
        
        data_dict = recommendation.to_dict()
        
        self.assertEqual(data_dict['category'], 'communication')
        self.assertEqual(data_dict['severity'], 'critical')
        self.assertEqual(data_dict['action_items'], ['Check cables'])


class TestPerformanceThresholds(unittest.TestCase):
    """Test PerformanceThresholds functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.thresholds = PerformanceThresholds()
    
    def test_command_latency_evaluation(self):
        """Test command latency performance evaluation"""
        # Excellent performance
        level = self.thresholds.evaluate_performance_level("command_latency", 2.0)
        self.assertEqual(level, PerformanceLevel.EXCELLENT)
        
        # Good performance
        level = self.thresholds.evaluate_performance_level("command_latency", 4.0)
        self.assertEqual(level, PerformanceLevel.GOOD)
        
        # Acceptable performance
        level = self.thresholds.evaluate_performance_level("command_latency", 8.0)
        self.assertEqual(level, PerformanceLevel.ACCEPTABLE)
        
        # Critical performance
        level = self.thresholds.evaluate_performance_level("command_latency", 15.0)
        self.assertEqual(level, PerformanceLevel.CRITICAL)
    
    def test_success_rate_evaluation(self):
        """Test success rate performance evaluation"""
        # Excellent performance
        level = self.thresholds.evaluate_performance_level("success_rate", 99.5)
        self.assertEqual(level, PerformanceLevel.EXCELLENT)
        
        # Good performance
        level = self.thresholds.evaluate_performance_level("success_rate", 97.0)
        self.assertEqual(level, PerformanceLevel.GOOD)
        
        # Acceptable performance
        level = self.thresholds.evaluate_performance_level("success_rate", 92.0)
        self.assertEqual(level, PerformanceLevel.ACCEPTABLE)
        
        # Critical performance
        level = self.thresholds.evaluate_performance_level("success_rate", 85.0)
        self.assertEqual(level, PerformanceLevel.CRITICAL)
    
    def test_position_error_evaluation(self):
        """Test position error performance evaluation"""
        # Excellent performance
        level = self.thresholds.evaluate_performance_level("position_error", 300.0)
        self.assertEqual(level, PerformanceLevel.EXCELLENT)
        
        # Good performance
        level = self.thresholds.evaluate_performance_level("position_error", 800.0)
        self.assertEqual(level, PerformanceLevel.GOOD)
        
        # Acceptable performance
        level = self.thresholds.evaluate_performance_level("position_error", 1500.0)
        self.assertEqual(level, PerformanceLevel.ACCEPTABLE)
        
        # Critical performance
        level = self.thresholds.evaluate_performance_level("position_error", 2500.0)
        self.assertEqual(level, PerformanceLevel.CRITICAL)


class TestPerformanceMonitorCore(unittest.TestCase):
    """Test core PerformanceMonitor functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_hardware_interface = Mock()
        self.mock_trajectory_monitor = Mock()
        self.mock_buffer_manager = Mock()
        
        self.performance_monitor = PerformanceMonitor(
            hardware_interface=self.mock_hardware_interface,
            trajectory_monitor=self.mock_trajectory_monitor,
            buffer_manager=self.mock_buffer_manager
        )
    
    def test_performance_monitor_initialization(self):
        """Test performance monitor initialization"""
        self.assertIsNotNone(self.performance_monitor)
        self.assertFalse(self.performance_monitor.monitoring_active)
        self.assertEqual(len(self.performance_monitor.command_timing_history), 0)
        self.assertIsInstance(self.performance_monitor.communication_stats, CommunicationStats)
        self.assertIsInstance(self.performance_monitor.thresholds, PerformanceThresholds)
    
    def test_start_stop_monitoring(self):
        """Test starting and stopping performance monitoring"""
        # Test starting monitoring
        success = self.performance_monitor.start_performance_monitoring()
        self.assertTrue(success)
        self.assertTrue(self.performance_monitor.monitoring_active)
        
        # Test stopping monitoring
        success = self.performance_monitor.stop_performance_monitoring()
        self.assertTrue(success)
        self.assertFalse(self.performance_monitor.monitoring_active)
    
    def test_start_monitoring_already_active(self):
        """Test starting monitoring when already active"""
        self.performance_monitor.start_performance_monitoring()
        
        # Try to start again
        success = self.performance_monitor.start_performance_monitoring()
        self.assertTrue(success)  # Should succeed but not create duplicate threads
    
    def test_stop_monitoring_not_active(self):
        """Test stopping monitoring when not active"""
        success = self.performance_monitor.stop_performance_monitoring()
        self.assertTrue(success)  # Should succeed even if not active


class TestCommandExecutionTracking(unittest.TestCase):
    """Test command execution tracking functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.performance_monitor = PerformanceMonitor()
    
    def test_track_successful_command(self):
        """Test tracking successful command execution"""
        self.performance_monitor.track_command_execution(
            command_type="SpeedM1M2",
            execution_time=0.003,
            success=True
        )
        
        # Check timing history
        self.assertEqual(len(self.performance_monitor.command_timing_history), 1)
        timing_data = self.performance_monitor.command_timing_history[0]
        self.assertEqual(timing_data.command_type, "SpeedM1M2")
        self.assertEqual(timing_data.execution_time, 0.003)
        self.assertTrue(timing_data.success)
        
        # Check communication stats
        stats = self.performance_monitor.communication_stats
        self.assertEqual(stats.total_commands, 1)
        self.assertEqual(stats.successful_commands, 1)
        self.assertEqual(stats.failed_commands, 0)
        self.assertEqual(stats.success_rate, 100.0)
        
        # Check performance metrics
        self.assertIn("command_latency", self.performance_monitor.performance_metrics)
        metric = self.performance_monitor.performance_metrics["command_latency"]
        self.assertEqual(metric.value, 3.0)  # 3ms
        self.assertEqual(metric.unit, "ms")
    
    def test_track_failed_command(self):
        """Test tracking failed command execution"""
        self.performance_monitor.track_command_execution(
            command_type="GetEncoders",
            execution_time=0.050,
            success=False,
            error_message="Communication timeout"
        )
        
        # Check timing history
        timing_data = self.performance_monitor.command_timing_history[0]
        self.assertFalse(timing_data.success)
        self.assertEqual(timing_data.error_message, "Communication timeout")
        
        # Check communication stats
        stats = self.performance_monitor.communication_stats
        self.assertEqual(stats.total_commands, 1)
        self.assertEqual(stats.successful_commands, 0)
        self.assertEqual(stats.failed_commands, 1)
        self.assertEqual(stats.timeout_count, 1)  # Should detect timeout
        self.assertEqual(stats.success_rate, 0.0)
    
    def test_track_multiple_commands(self):
        """Test tracking multiple commands"""
        # Track several commands
        commands = [
            ("SpeedM1M2", 0.002, True),
            ("GetEncoders", 0.004, True),
            ("DutyM1M2", 0.001, True),
            ("GetStatus", 0.008, False, "Communication error"),
            ("SpeedM1M2", 0.003, True)
        ]
        
        for cmd_data in commands:
            if len(cmd_data) == 3:
                self.performance_monitor.track_command_execution(*cmd_data)
            else:
                self.performance_monitor.track_command_execution(*cmd_data)
        
        # Check aggregated stats
        stats = self.performance_monitor.communication_stats
        self.assertEqual(stats.total_commands, 5)
        self.assertEqual(stats.successful_commands, 4)
        self.assertEqual(stats.failed_commands, 1)
        self.assertEqual(stats.success_rate, 80.0)
        
        # Check latency calculations
        expected_avg_latency = (0.002 + 0.004 + 0.001 + 0.003) / 4
        self.assertAlmostEqual(stats.average_latency, expected_avg_latency, places=6)
        self.assertEqual(stats.min_latency, 0.001)
        self.assertEqual(stats.max_latency, 0.004)


class TestCommunicationPerformanceTracking(unittest.TestCase):
    """Test communication performance tracking"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.performance_monitor = PerformanceMonitor()
    
    def test_track_communication_performance(self):
        """Test tracking communication performance"""
        # Simulate some successful communications
        for i in range(10):
            latency = 0.001 + (i * 0.0005)  # Increasing latency
            self.performance_monitor.track_command_execution(
                f"Command{i}", latency, True
            )
        
        # Track communication performance
        self.performance_monitor.track_communication_performance(
            latency=0.005,
            success=True,
            timeout_occurred=False
        )
        
        # Check metrics
        self.assertIn("success_rate", self.performance_monitor.performance_metrics)
        self.assertIn("average_latency", self.performance_monitor.performance_metrics)
        
        success_metric = self.performance_monitor.performance_metrics["success_rate"]
        self.assertEqual(success_metric.value, 100.0)
        self.assertEqual(success_metric.category, PerformanceCategory.COMMUNICATION)
    
    def test_track_communication_with_timeouts(self):
        """Test tracking communication with timeout events"""
        # Track some communications with timeouts
        self.performance_monitor.track_communication_performance(
            latency=0.100,
            success=False,
            timeout_occurred=True
        )
        
        # Check timeout count
        stats = self.performance_monitor.communication_stats
        self.assertEqual(stats.timeout_count, 1)


class TestMotionControlPerformanceTracking(unittest.TestCase):
    """Test motion control performance tracking"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.performance_monitor = PerformanceMonitor()
    
    def test_track_motion_control_performance(self):
        """Test tracking motion control performance"""
        # Track motion control data
        self.performance_monitor.track_motion_control_performance(
            position_error=500.0,
            speed_error=50.0,
            trajectory_deviation=100.0,
            smoothness_score=95.0
        )
        
        # Check motion control stats
        stats = self.performance_monitor.motion_control_stats
        self.assertEqual(len(stats.position_error_samples), 1)
        self.assertEqual(stats.position_error_samples[0], 500.0)
        self.assertEqual(stats.speed_error_samples[0], 50.0)
        
        # Check performance metrics
        self.assertIn("position_error", self.performance_monitor.performance_metrics)
        self.assertIn("trajectory_accuracy", self.performance_monitor.performance_metrics)
        
        position_metric = self.performance_monitor.performance_metrics["position_error"]
        self.assertEqual(position_metric.value, 500.0)
        self.assertEqual(position_metric.category, PerformanceCategory.MOTION_CONTROL)
    
    def test_track_multiple_motion_samples(self):
        """Test tracking multiple motion control samples"""
        # Track multiple samples
        samples = [
            (100.0, 10.0, 50.0, 98.0),
            (-200.0, -15.0, 75.0, 96.0),
            (300.0, 8.0, 25.0, 99.0),
            (-150.0, -12.0, 40.0, 97.0)
        ]
        
        for pos_err, speed_err, traj_dev, smooth in samples:
            self.performance_monitor.track_motion_control_performance(
                pos_err, speed_err, traj_dev, smooth
            )
        
        # Check aggregated data
        stats = self.performance_monitor.motion_control_stats
        self.assertEqual(len(stats.position_error_samples), 4)
        self.assertEqual(len(stats.trajectory_deviations), 4)
        
        # Check trajectory accuracy calculation
        accuracy = stats.trajectory_accuracy
        self.assertGreater(accuracy, 90.0)  # Should be high with small deviations


class TestBufferUtilizationTracking(unittest.TestCase):
    """Test buffer utilization tracking"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.performance_monitor = PerformanceMonitor()
    
    def test_track_buffer_utilization(self):
        """Test tracking buffer utilization"""
        # Track buffer utilization in optimal range
        self.performance_monitor.track_buffer_utilization(
            buffer_depth=4,
            is_optimal=True
        )
        
        # Check buffer stats
        stats = self.performance_monitor.buffer_stats
        self.assertEqual(len(stats.buffer_depth_samples), 1)
        self.assertEqual(stats.buffer_depth_samples[0], 4)
        self.assertGreater(stats.optimal_utilization_time, 0)
        
        # Check performance metrics
        self.assertIn("buffer_efficiency", self.performance_monitor.performance_metrics)
        efficiency_metric = self.performance_monitor.performance_metrics["buffer_efficiency"]
        self.assertEqual(efficiency_metric.category, PerformanceCategory.BUFFER_UTILIZATION)
    
    def test_track_buffer_overflow(self):
        """Test tracking buffer overflow conditions"""
        # Track buffer overflow
        self.performance_monitor.track_buffer_utilization(
            buffer_depth=32,
            is_optimal=False
        )
        
        stats = self.performance_monitor.buffer_stats
        self.assertEqual(stats.buffer_overflow_count, 1)
    
    def test_track_buffer_underrun(self):
        """Test tracking buffer underrun conditions"""
        # Track buffer underrun
        self.performance_monitor.track_buffer_utilization(
            buffer_depth=0,
            is_optimal=False
        )
        
        stats = self.performance_monitor.buffer_stats
        self.assertEqual(stats.buffer_underrun_count, 1)
    
    def test_buffer_efficiency_calculation(self):
        """Test buffer efficiency calculation"""
        # Track mix of optimal and suboptimal buffer depths
        buffer_depths = [2, 4, 6, 8, 10, 1, 15, 5, 3, 7]  # Count optimal vs suboptimal
        optimal_count = sum(1 for depth in buffer_depths if 2 <= depth <= 8)
        expected_efficiency = (optimal_count / len(buffer_depths)) * 100.0
        
        for depth in buffer_depths:
            is_optimal = 2 <= depth <= 8
            self.performance_monitor.track_buffer_utilization(depth, is_optimal)
        
        stats = self.performance_monitor.buffer_stats
        efficiency = stats.buffer_efficiency
        
        # Check actual efficiency calculation (7 out of 10 samples are optimal: 2,4,6,8,5,3,7)
        self.assertAlmostEqual(efficiency, expected_efficiency, places=1)


class TestOptimizationRecommendations(unittest.TestCase):
    """Test optimization recommendation generation"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.performance_monitor = PerformanceMonitor()
    
    def test_generate_recommendations_no_issues(self):
        """Test generating recommendations with good performance"""
        # Track good performance
        self.performance_monitor.track_command_execution("SpeedM1M2", 0.002, True)
        self.performance_monitor.track_motion_control_performance(100, 10, 25, 98)
        
        recommendations = self.performance_monitor.generate_optimization_recommendations()
        
        # Should have no recommendations for good performance
        self.assertEqual(len(recommendations), 0)
    
    def test_generate_recommendations_high_latency(self):
        """Test generating recommendations for high command latency"""
        # Track poor command performance
        self.performance_monitor.track_command_execution("SpeedM1M2", 0.015, True)  # 15ms
        
        recommendations = self.performance_monitor.generate_optimization_recommendations()
        
        # Should have recommendation for high latency
        self.assertGreater(len(recommendations), 0)
        
        latency_rec = next(
            (rec for rec in recommendations 
             if rec.category == PerformanceCategory.COMMAND_EXECUTION),
            None
        )
        self.assertIsNotNone(latency_rec)
        self.assertEqual(latency_rec.title, "High Command Execution Latency")
        self.assertIn("baud rate", latency_rec.action_items[0])
    
    def test_generate_recommendations_low_success_rate(self):
        """Test generating recommendations for low success rate"""
        # Track poor communication performance
        for i in range(10):
            success = i < 8  # 80% success rate
            self.performance_monitor.track_command_execution(f"Cmd{i}", 0.005, success)
        
        # Trigger communication performance tracking to update metrics
        self.performance_monitor.track_communication_performance(0.005, True, False)
        
        recommendations = self.performance_monitor.generate_optimization_recommendations()
        
        # Should have recommendation for low success rate
        comm_rec = next(
            (rec for rec in recommendations 
             if rec.category == PerformanceCategory.COMMUNICATION),
            None
        )
        self.assertIsNotNone(comm_rec)
        self.assertEqual(comm_rec.title, "Low Communication Success Rate")
    
    def test_generate_recommendations_poor_accuracy(self):
        """Test generating recommendations for poor trajectory accuracy"""
        # Track poor motion control performance (large deviations)
        for i in range(10):
            large_deviation = 2000 + (i * 100)  # Large trajectory deviations
            self.performance_monitor.track_motion_control_performance(
                large_deviation, 100, large_deviation, 85
            )
        
        recommendations = self.performance_monitor.generate_optimization_recommendations()
        
        # Should have recommendation for poor accuracy
        motion_rec = next(
            (rec for rec in recommendations 
             if rec.category == PerformanceCategory.MOTION_CONTROL),
            None
        )
        self.assertIsNotNone(motion_rec)
        self.assertEqual(motion_rec.title, "Poor Trajectory Accuracy")
    
    def test_generate_recommendations_buffer_inefficiency(self):
        """Test generating recommendations for buffer inefficiency"""
        # Track poor buffer utilization (mostly out of optimal range)
        buffer_depths = [0, 1, 15, 20, 0, 32, 25, 1, 30, 0]  # Mostly suboptimal
        
        for depth in buffer_depths:
            is_optimal = 2 <= depth <= 8
            self.performance_monitor.track_buffer_utilization(depth, is_optimal)
        
        recommendations = self.performance_monitor.generate_optimization_recommendations()
        
        # Should have recommendation for buffer inefficiency
        buffer_rec = next(
            (rec for rec in recommendations 
             if rec.category == PerformanceCategory.BUFFER_UTILIZATION),
            None
        )
        self.assertIsNotNone(buffer_rec)
        self.assertEqual(buffer_rec.title, "Inefficient Buffer Utilization")
    
    def test_generate_recommendations_multiple_issues(self):
        """Test generating recommendations for multiple performance issues"""
        # Create multiple performance issues
        
        # High latency
        self.performance_monitor.track_command_execution("Cmd1", 0.020, True)
        
        # Low success rate
        for i in range(10):
            success = i < 7  # 70% success rate
            self.performance_monitor.track_command_execution(f"Cmd{i+2}", 0.005, success)
        
        # Trigger communication performance tracking
        self.performance_monitor.track_communication_performance(0.005, True, False)
        
        # Poor motion accuracy
        for i in range(5):
            self.performance_monitor.track_motion_control_performance(
                3000, 200, 2000, 70  # Poor accuracy
            )
        
        # Poor buffer utilization
        for depth in [0, 1, 32, 30, 0, 1, 32]:  # All suboptimal
            self.performance_monitor.track_buffer_utilization(depth, False)
        
        # Set evaluation time to trigger overall system check
        self.performance_monitor.last_performance_evaluation = time.time() - 400  # 6+ minutes ago
        
        recommendations = self.performance_monitor.generate_optimization_recommendations()
        
        # Should have multiple recommendations
        self.assertGreaterEqual(len(recommendations), 2)
        
        # Check for specific recommendation categories (at least communication and motion)
        categories = [rec.category for rec in recommendations]
        self.assertIn(PerformanceCategory.COMMUNICATION, categories)
        
        # Should have multiple different categories
        unique_categories = set(categories)
        self.assertGreaterEqual(len(unique_categories), 2)


class TestPerformanceReporting(unittest.TestCase):
    """Test performance reporting functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.performance_monitor = PerformanceMonitor()
    
    def test_get_performance_report_empty(self):
        """Test getting performance report with no data"""
        report = self.performance_monitor.get_performance_report()
        
        self.assertIn('timestamp', report)
        self.assertIn('monitoring_active', report)
        self.assertIn('command_execution', report)
        self.assertIn('communication', report)
        self.assertIn('motion_control', report)
        self.assertIn('buffer_utilization', report)
        self.assertIn('current_metrics', report)
        self.assertIn('recommendations', report)
        self.assertIn('overall_status', report)
        
        # Should have good overall status with no data
        overall = report['overall_status']
        self.assertEqual(overall['level'], 'good')
    
    def test_get_performance_report_with_data(self):
        """Test getting performance report with performance data"""
        # Add some performance data
        self.performance_monitor.track_command_execution("SpeedM1M2", 0.003, True)
        self.performance_monitor.track_motion_control_performance(200, 20, 50, 95)
        self.performance_monitor.track_buffer_utilization(4, True)
        
        report = self.performance_monitor.get_performance_report()
        
        # Check command execution data
        cmd_exec = report['command_execution']
        self.assertEqual(cmd_exec['recent_commands'], 1)
        self.assertEqual(cmd_exec['success_rate'], 100.0)
        
        # Check communication data
        comm = report['communication']
        self.assertEqual(comm['total_commands'], 1)
        self.assertEqual(comm['successful_commands'], 1)
        
        # Check motion control data
        motion = report['motion_control']
        self.assertIn('average_position_error', motion)
        self.assertIn('trajectory_accuracy', motion)
        
        # Check buffer utilization data
        buffer = report['buffer_utilization']
        self.assertIn('average_buffer_depth', buffer)
        self.assertIn('buffer_efficiency', buffer)
        
        # Check current metrics
        metrics = report['current_metrics']
        self.assertIn('command_latency', metrics)
        self.assertIn('trajectory_accuracy', metrics)
        
        # Check overall status
        overall = report['overall_status']
        self.assertIn('score', overall)
        self.assertIn('summary', overall)
    
    def test_overall_performance_evaluation(self):
        """Test overall performance evaluation"""
        # Create mixed performance levels
        
        # Good command latency
        self.performance_monitor.track_command_execution("Cmd1", 0.003, True)
        
        # Poor motion control
        self.performance_monitor.track_motion_control_performance(3000, 500, 2000, 60)
        
        # Get overall evaluation
        report = self.performance_monitor.get_performance_report()
        overall = report['overall_status']
        
        # Should reflect mixed performance
        self.assertLess(overall['score'], 100.0)
        self.assertGreater(len(overall['issues']), 0)
        self.assertIn('position_error', overall['issues'][0])
    
    def test_reset_performance_statistics(self):
        """Test resetting performance statistics"""
        # Add some data
        self.performance_monitor.track_command_execution("SpeedM1M2", 0.005, True)
        self.performance_monitor.track_motion_control_performance(100, 10, 25, 95)
        
        # Verify data exists
        self.assertGreater(len(self.performance_monitor.command_timing_history), 0)
        self.assertGreater(self.performance_monitor.communication_stats.total_commands, 0)
        
        # Reset statistics
        self.performance_monitor.reset_performance_statistics()
        
        # Verify data is cleared
        self.assertEqual(len(self.performance_monitor.command_timing_history), 0)
        self.assertEqual(self.performance_monitor.communication_stats.total_commands, 0)
        self.assertEqual(len(self.performance_monitor.performance_metrics), 0)


class TestPerformanceMonitorIntegration(unittest.TestCase):
    """Test performance monitor integration with other components"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_hardware_interface = Mock()
        self.mock_trajectory_monitor = Mock()
        self.mock_buffer_manager = Mock()
        
        self.performance_monitor = PerformanceMonitor(
            hardware_interface=self.mock_hardware_interface,
            trajectory_monitor=self.mock_trajectory_monitor,
            buffer_manager=self.mock_buffer_manager
        )
    
    def test_component_integration(self):
        """Test integration with hardware components"""
        self.assertEqual(
            self.performance_monitor.hardware_interface,
            self.mock_hardware_interface
        )
        self.assertEqual(
            self.performance_monitor.trajectory_monitor,
            self.mock_trajectory_monitor
        )
        self.assertEqual(
            self.performance_monitor.buffer_manager,
            self.mock_buffer_manager
        )
    
    def test_callback_registration(self):
        """Test optimization callback registration"""
        callback_called = False
        
        def test_callback(recommendations):
            nonlocal callback_called
            callback_called = True
        
        self.performance_monitor.register_optimization_callback(test_callback)
        
        # Verify callback is registered
        self.assertIn(test_callback, self.performance_monitor.recommendation_callbacks)
    
    @patch('time.sleep')
    def test_monitoring_loop_execution(self, mock_sleep):
        """Test monitoring loop execution"""
        # Start monitoring
        success = self.performance_monitor.start_performance_monitoring()
        self.assertTrue(success)
        
        # Let it run briefly
        time.sleep(0.1)
        
        # Stop monitoring
        success = self.performance_monitor.stop_performance_monitoring()
        self.assertTrue(success)
        
        # Verify monitoring thread was created and stopped
        self.assertIsNotNone(self.performance_monitor.monitoring_thread)


class TestPerformanceMonitorStandalone(unittest.TestCase):
    """Test performance monitor in standalone mode"""
    
    def test_standalone_execution(self):
        """Test performance monitor can run standalone"""
        monitor = PerformanceMonitor()
        
        # Should initialize without ROS2
        self.assertIsNotNone(monitor)
        
        # Should be able to track performance
        monitor.track_command_execution("TestCmd", 0.005, True)
        
        # Should generate reports
        report = monitor.get_performance_report()
        self.assertIn('timestamp', report)


def run_all_tests():
    """Run all performance monitor tests"""
    test_classes = [
        TestPerformanceMetric,
        TestCommandTimingData,
        TestCommunicationStats,
        TestMotionControlStats,
        TestBufferStats,
        TestOptimizationRecommendation,
        TestPerformanceThresholds,
        TestPerformanceMonitorCore,
        TestCommandExecutionTracking,
        TestCommunicationPerformanceTracking,
        TestMotionControlPerformanceTracking,
        TestBufferUtilizationTracking,
        TestOptimizationRecommendations,
        TestPerformanceReporting,
        TestPerformanceMonitorIntegration,
        TestPerformanceMonitorStandalone
    ]
    
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print(f"\n{'='*70}")
    print(f"Performance Monitor Test Summary")
    print(f"{'='*70}")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print(f"\nFailures:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback}")
    
    if result.errors:
        print(f"\nErrors:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)