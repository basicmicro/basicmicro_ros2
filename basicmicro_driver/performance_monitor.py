#!/usr/bin/env python3
"""
Performance Monitoring and Optimization System for Basicmicro Driver

This module implements comprehensive performance monitoring and optimization 
recommendations for motion control as specified in Task 3.3.

Features:
- Command execution timing and latency monitoring
- Communication performance and reliability tracking
- Motion control accuracy and smoothness analysis
- Buffer utilization efficiency monitoring
- Performance optimization recommendations
- Performance reporting and trending analysis

Author: Basicmicro Driver Team
License: MIT
"""

import time
import threading
import statistics
from collections import deque, defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Callable
from enum import Enum
import json

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    
    # Mock classes for non-ROS2 environments
    class Node:
        def __init__(self, name):
            self.name = name
            
        def get_logger(self):
            return MockLogger()
            
        def create_timer(self, period, callback):
            return MockTimer()
            
        def create_publisher(self, msg_type, topic, qos):
            return MockPublisher()
            
        def declare_parameter(self, name, default_value):
            return MockParameter(default_value)
            
    class MockLogger:
        def info(self, msg): print(f"[INFO] {msg}")
        def warn(self, msg): print(f"[WARN] {msg}")
        def error(self, msg): print(f"[ERROR] {msg}")
        def debug(self, msg): print(f"[DEBUG] {msg}")
        
    class MockTimer:
        pass
        
    class MockPublisher:
        def publish(self, msg):
            pass
            
    class MockParameter:
        def __init__(self, value):
            self.value = value


class PerformanceCategory(Enum):
    """Performance monitoring categories"""
    COMMAND_EXECUTION = "command_execution"
    COMMUNICATION = "communication"
    MOTION_CONTROL = "motion_control"
    BUFFER_UTILIZATION = "buffer_utilization"
    OVERALL_SYSTEM = "overall_system"


class PerformanceLevel(Enum):
    """Performance evaluation levels"""
    EXCELLENT = "excellent"
    GOOD = "good"
    ACCEPTABLE = "acceptable"
    POOR = "poor"
    CRITICAL = "critical"


@dataclass
class PerformanceMetric:
    """Individual performance metric"""
    name: str
    value: float
    unit: str
    timestamp: float
    category: PerformanceCategory
    level: PerformanceLevel = PerformanceLevel.GOOD
    threshold_exceeded: bool = False
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        return {
            'name': self.name,
            'value': self.value,
            'unit': self.unit,
            'timestamp': self.timestamp,
            'category': self.category.value,
            'level': self.level.value,
            'threshold_exceeded': self.threshold_exceeded
        }


@dataclass
class CommandTimingData:
    """Command execution timing data"""
    command_type: str
    start_time: float
    end_time: float
    execution_time: float
    success: bool
    error_message: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for analysis"""
        return {
            'command_type': self.command_type,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'execution_time': self.execution_time,
            'success': self.success,
            'error_message': self.error_message
        }


@dataclass
class CommunicationStats:
    """Communication performance statistics"""
    total_commands: int = 0
    successful_commands: int = 0
    failed_commands: int = 0
    total_latency: float = 0.0
    min_latency: float = float('inf')
    max_latency: float = 0.0
    timeout_count: int = 0
    retry_count: int = 0
    
    @property
    def success_rate(self) -> float:
        """Calculate success rate percentage"""
        if self.total_commands == 0:
            return 100.0
        return (self.successful_commands / self.total_commands) * 100.0
    
    @property
    def average_latency(self) -> float:
        """Calculate average latency"""
        if self.successful_commands == 0:
            return 0.0
        return self.total_latency / self.successful_commands
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for reporting"""
        return {
            'total_commands': self.total_commands,
            'successful_commands': self.successful_commands,
            'failed_commands': self.failed_commands,
            'success_rate': self.success_rate,
            'average_latency': self.average_latency,
            'min_latency': self.min_latency if self.min_latency != float('inf') else 0.0,
            'max_latency': self.max_latency,
            'timeout_count': self.timeout_count,
            'retry_count': self.retry_count
        }


@dataclass
class MotionControlStats:
    """Motion control performance statistics"""
    position_error_samples: List[float] = field(default_factory=list)
    speed_error_samples: List[float] = field(default_factory=list)
    trajectory_deviations: List[float] = field(default_factory=list)
    accuracy_measurements: List[float] = field(default_factory=list)
    smoothness_measurements: List[float] = field(default_factory=list)
    
    @property
    def average_position_error(self) -> float:
        """Calculate average position error"""
        return statistics.mean(self.position_error_samples) if self.position_error_samples else 0.0
    
    @property
    def average_speed_error(self) -> float:
        """Calculate average speed error"""
        return statistics.mean(self.speed_error_samples) if self.speed_error_samples else 0.0
    
    @property
    def trajectory_accuracy(self) -> float:
        """Calculate trajectory accuracy percentage"""
        if not self.trajectory_deviations:
            return 100.0
        
        # Consider accuracy as inverse of average deviation (normalized)
        avg_deviation = statistics.mean(self.trajectory_deviations)
        # Assume 1000 encoder counts as 100% deviation for normalization
        accuracy = max(0.0, 100.0 - (avg_deviation / 1000.0) * 100.0)
        return min(100.0, accuracy)
    
    @property
    def motion_smoothness(self) -> float:
        """Calculate motion smoothness score"""
        return statistics.mean(self.smoothness_measurements) if self.smoothness_measurements else 100.0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for reporting"""
        return {
            'average_position_error': self.average_position_error,
            'average_speed_error': self.average_speed_error,
            'trajectory_accuracy': self.trajectory_accuracy,
            'motion_smoothness': self.motion_smoothness,
            'position_error_std': statistics.stdev(self.position_error_samples) if len(self.position_error_samples) > 1 else 0.0,
            'speed_error_std': statistics.stdev(self.speed_error_samples) if len(self.speed_error_samples) > 1 else 0.0,
            'sample_count': len(self.position_error_samples)
        }


@dataclass
class BufferStats:
    """Buffer utilization statistics"""
    buffer_depth_samples: List[int] = field(default_factory=list)
    buffer_overflow_count: int = 0
    buffer_underrun_count: int = 0
    optimal_utilization_time: float = 0.0
    total_monitoring_time: float = 0.0
    
    @property
    def average_buffer_depth(self) -> float:
        """Calculate average buffer depth"""
        return statistics.mean(self.buffer_depth_samples) if self.buffer_depth_samples else 0.0
    
    @property
    def buffer_efficiency(self) -> float:
        """Calculate buffer efficiency percentage"""
        if not self.buffer_depth_samples:
            return 100.0
        
        # Optimal range is 2-8 commands for smooth motion
        optimal_count = sum(1 for depth in self.buffer_depth_samples if 2 <= depth <= 8)
        return (optimal_count / len(self.buffer_depth_samples)) * 100.0
    
    @property
    def utilization_rate(self) -> float:
        """Calculate buffer utilization rate"""
        if self.total_monitoring_time == 0:
            return 0.0
        return (self.optimal_utilization_time / self.total_monitoring_time) * 100.0
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for reporting"""
        return {
            'average_buffer_depth': self.average_buffer_depth,
            'buffer_efficiency': self.buffer_efficiency,
            'utilization_rate': self.utilization_rate,
            'buffer_overflow_count': self.buffer_overflow_count,
            'buffer_underrun_count': self.buffer_underrun_count,
            'max_buffer_depth': max(self.buffer_depth_samples) if self.buffer_depth_samples else 0,
            'min_buffer_depth': min(self.buffer_depth_samples) if self.buffer_depth_samples else 0
        }


@dataclass
class OptimizationRecommendation:
    """Performance optimization recommendation"""
    category: PerformanceCategory
    severity: PerformanceLevel
    title: str
    description: str
    action_items: List[str]
    expected_improvement: str
    implementation_effort: str  # "low", "medium", "high"
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for reporting"""
        return {
            'category': self.category.value,
            'severity': self.severity.value,
            'title': self.title,
            'description': self.description,
            'action_items': self.action_items,
            'expected_improvement': self.expected_improvement,
            'implementation_effort': self.implementation_effort
        }


class PerformanceThresholds:
    """Performance monitoring thresholds"""
    
    def __init__(self):
        # Command execution thresholds (milliseconds)
        self.max_command_latency = 5.0
        self.critical_command_latency = 10.0
        
        # Communication thresholds
        self.min_success_rate = 95.0
        self.critical_success_rate = 90.0
        self.max_average_latency = 50.0
        self.critical_average_latency = 100.0
        
        # Motion control thresholds
        self.max_position_error = 1000.0  # encoder counts
        self.critical_position_error = 2000.0
        self.max_speed_error = 500.0  # counts/sec
        self.critical_speed_error = 1000.0
        self.min_trajectory_accuracy = 95.0  # percentage
        self.critical_trajectory_accuracy = 90.0
        
        # Buffer utilization thresholds
        self.optimal_buffer_min = 2
        self.optimal_buffer_max = 8
        self.buffer_efficiency_threshold = 80.0  # percentage
        self.critical_buffer_efficiency = 60.0
        
    def evaluate_performance_level(self, metric_name: str, value: float) -> PerformanceLevel:
        """Evaluate performance level based on thresholds"""
        
        # Command execution thresholds
        if metric_name == "command_latency":
            if value <= self.max_command_latency:
                return PerformanceLevel.EXCELLENT if value <= 2.0 else PerformanceLevel.GOOD
            elif value <= self.critical_command_latency:
                return PerformanceLevel.ACCEPTABLE
            else:
                return PerformanceLevel.CRITICAL
        
        # Communication thresholds
        elif metric_name == "success_rate":
            if value >= self.min_success_rate:
                return PerformanceLevel.EXCELLENT if value >= 99.0 else PerformanceLevel.GOOD
            elif value >= self.critical_success_rate:
                return PerformanceLevel.ACCEPTABLE
            else:
                return PerformanceLevel.CRITICAL
        
        elif metric_name == "average_latency":
            if value <= self.max_average_latency:
                return PerformanceLevel.EXCELLENT if value <= 20.0 else PerformanceLevel.GOOD
            elif value <= self.critical_average_latency:
                return PerformanceLevel.ACCEPTABLE
            else:
                return PerformanceLevel.CRITICAL
        
        # Motion control thresholds
        elif metric_name == "position_error":
            if abs(value) <= self.max_position_error:
                return PerformanceLevel.EXCELLENT if abs(value) <= 500.0 else PerformanceLevel.GOOD
            elif abs(value) <= self.critical_position_error:
                return PerformanceLevel.ACCEPTABLE
            else:
                return PerformanceLevel.CRITICAL
        
        elif metric_name == "trajectory_accuracy":
            if value >= self.min_trajectory_accuracy:
                return PerformanceLevel.EXCELLENT if value >= 99.0 else PerformanceLevel.GOOD
            elif value >= self.critical_trajectory_accuracy:
                return PerformanceLevel.ACCEPTABLE
            else:
                return PerformanceLevel.CRITICAL
        
        # Buffer utilization thresholds
        elif metric_name == "buffer_efficiency":
            if value >= self.buffer_efficiency_threshold:
                return PerformanceLevel.EXCELLENT if value >= 90.0 else PerformanceLevel.GOOD
            elif value >= self.critical_buffer_efficiency:
                return PerformanceLevel.ACCEPTABLE
            else:
                return PerformanceLevel.CRITICAL
        
        # Default case
        return PerformanceLevel.GOOD


class PerformanceMonitor(Node if ROS2_AVAILABLE else object):
    """
    Comprehensive performance monitoring and optimization system.
    
    Monitors:
    - Command execution timing and latency
    - Communication performance and reliability
    - Motion control accuracy and smoothness
    - Buffer utilization efficiency
    - Provides optimization recommendations
    """
    
    def __init__(self, hardware_interface=None, trajectory_monitor=None, buffer_manager=None):
        """
        Initialize performance monitor.
        
        Args:
            hardware_interface: Hardware interface for command monitoring
            trajectory_monitor: Trajectory monitor for motion control data
            buffer_manager: Buffer manager for utilization monitoring
        """
        if ROS2_AVAILABLE:
            super().__init__('performance_monitor')
            
            # ROS2 publishers
            self.performance_publisher = self.create_publisher(
                String, 
                'performance_metrics', 
                10
            )
            
            self.optimization_publisher = self.create_publisher(
                String,
                'optimization_recommendations',
                10
            )
            
            # Timer for periodic reporting
            self.monitoring_timer = self.create_timer(
                10.0,  # 10 second reporting interval
                self.periodic_performance_report
            )
            
            # Parameters
            self.monitoring_enabled = self.declare_parameter(
                'performance_monitoring_enabled', True
            ).value
            
            self.monitoring_rate = self.declare_parameter(
                'monitoring_rate_hz', 10.0
            ).value
            
            self.report_interval = self.declare_parameter(
                'report_interval_seconds', 10.0
            ).value
            
            self.optimization_enabled = self.declare_parameter(
                'optimization_recommendations_enabled', True
            ).value
            
            self.logger = self.get_logger()
        else:
            # Non-ROS2 initialization
            self.monitoring_enabled = True
            self.monitoring_rate = 10.0
            self.report_interval = 10.0
            self.optimization_enabled = True
            self.logger = MockLogger()
        
        # Component references
        self.hardware_interface = hardware_interface
        self.trajectory_monitor = trajectory_monitor
        self.buffer_manager = buffer_manager
        
        # Performance data storage
        self.command_timing_history = deque(maxlen=1000)
        self.communication_stats = CommunicationStats()
        self.motion_control_stats = MotionControlStats()
        self.buffer_stats = BufferStats()
        
        # Performance metrics
        self.performance_metrics = {}
        self.performance_history = defaultdict(lambda: deque(maxlen=100))
        
        # Thresholds and evaluation
        self.thresholds = PerformanceThresholds()
        
        # Optimization system
        self.current_recommendations = []
        self.recommendation_callbacks = []
        
        # Monitoring state
        self.monitoring_active = False
        self.monitoring_thread = None
        self.monitoring_lock = threading.Lock()
        
        # Timing tracking
        self.command_start_times = {}
        self.last_performance_evaluation = time.time()
        
        self.logger.info("Performance monitor initialized")
    
    def start_performance_monitoring(self) -> bool:
        """
        Start performance monitoring system.
        
        Returns:
            bool: True if monitoring started successfully
        """
        try:
            with self.monitoring_lock:
                if self.monitoring_active:
                    self.logger.warn("Performance monitoring already active")
                    return True
                
                if not self.monitoring_enabled:
                    self.logger.info("Performance monitoring disabled via parameter")
                    return False
                
                self.monitoring_active = True
                
                # Start monitoring thread
                self.monitoring_thread = threading.Thread(
                    target=self._monitoring_loop,
                    daemon=True
                )
                self.monitoring_thread.start()
                
                self.logger.info("Performance monitoring started")
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to start performance monitoring: {e}")
            return False
    
    def stop_performance_monitoring(self) -> bool:
        """
        Stop performance monitoring system.
        
        Returns:
            bool: True if monitoring stopped successfully
        """
        try:
            with self.monitoring_lock:
                if not self.monitoring_active:
                    self.logger.info("Performance monitoring not active")
                    return True
                
                self.monitoring_active = False
                
                # Wait for monitoring thread to finish
                if self.monitoring_thread and self.monitoring_thread.is_alive():
                    self.monitoring_thread.join(timeout=5.0)
                
                self.logger.info("Performance monitoring stopped")
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to stop performance monitoring: {e}")
            return False
    
    def track_command_execution(self, command_type: str, execution_time: float, 
                               success: bool, error_message: Optional[str] = None):
        """
        Track command execution performance.
        
        Args:
            command_type: Type of command executed
            execution_time: Execution time in seconds
            success: Whether command succeeded
            error_message: Error message if command failed
        """
        try:
            current_time = time.time()
            
            # Create timing data
            timing_data = CommandTimingData(
                command_type=command_type,
                start_time=current_time - execution_time,
                end_time=current_time,
                execution_time=execution_time,
                success=success,
                error_message=error_message
            )
            
            # Store timing data
            self.command_timing_history.append(timing_data)
            
            # Update communication stats
            self.communication_stats.total_commands += 1
            if success:
                self.communication_stats.successful_commands += 1
                self.communication_stats.total_latency += execution_time
                self.communication_stats.min_latency = min(
                    self.communication_stats.min_latency, 
                    execution_time
                )
                self.communication_stats.max_latency = max(
                    self.communication_stats.max_latency, 
                    execution_time
                )
            else:
                self.communication_stats.failed_commands += 1
                
                # Classify error types
                if error_message and 'timeout' in error_message.lower():
                    self.communication_stats.timeout_count += 1
                elif error_message and 'retry' in error_message.lower():
                    self.communication_stats.retry_count += 1
            
            # Create performance metrics
            latency_ms = execution_time * 1000.0
            latency_metric = PerformanceMetric(
                name="command_latency",
                value=latency_ms,
                unit="ms",
                timestamp=current_time,
                category=PerformanceCategory.COMMAND_EXECUTION,
                level=self.thresholds.evaluate_performance_level("command_latency", latency_ms),
                threshold_exceeded=(latency_ms > self.thresholds.max_command_latency)
            )
            
            self.performance_metrics["command_latency"] = latency_metric
            self.performance_history["command_latency"].append(latency_metric)
            
        except Exception as e:
            self.logger.error(f"Error tracking command execution: {e}")
    
    def track_communication_performance(self, latency: float, success: bool, 
                                      timeout_occurred: bool = False):
        """
        Track communication performance metrics.
        
        Args:
            latency: Communication latency in seconds
            success: Whether communication succeeded
            timeout_occurred: Whether a timeout occurred
        """
        try:
            current_time = time.time()
            
            # Update communication statistics
            if timeout_occurred:
                self.communication_stats.timeout_count += 1
            
            # Create communication performance metrics
            success_rate_metric = PerformanceMetric(
                name="success_rate",
                value=self.communication_stats.success_rate,
                unit="%",
                timestamp=current_time,
                category=PerformanceCategory.COMMUNICATION,
                level=self.thresholds.evaluate_performance_level(
                    "success_rate", 
                    self.communication_stats.success_rate
                ),
                threshold_exceeded=(
                    self.communication_stats.success_rate < self.thresholds.min_success_rate
                )
            )
            
            avg_latency_ms = self.communication_stats.average_latency * 1000.0
            latency_metric = PerformanceMetric(
                name="average_latency",
                value=avg_latency_ms,
                unit="ms",
                timestamp=current_time,
                category=PerformanceCategory.COMMUNICATION,
                level=self.thresholds.evaluate_performance_level("average_latency", avg_latency_ms),
                threshold_exceeded=(avg_latency_ms > self.thresholds.max_average_latency)
            )
            
            self.performance_metrics["success_rate"] = success_rate_metric
            self.performance_metrics["average_latency"] = latency_metric
            
            self.performance_history["success_rate"].append(success_rate_metric)
            self.performance_history["average_latency"].append(latency_metric)
            
        except Exception as e:
            self.logger.error(f"Error tracking communication performance: {e}")
    
    def track_motion_control_performance(self, position_error: float, speed_error: float,
                                       trajectory_deviation: float = 0.0,
                                       smoothness_score: float = 100.0):
        """
        Track motion control accuracy and smoothness.
        
        Args:
            position_error: Position error in encoder counts
            speed_error: Speed error in counts/sec
            trajectory_deviation: Trajectory deviation in encoder counts
            smoothness_score: Motion smoothness score (0-100)
        """
        try:
            current_time = time.time()
            
            # Update motion control statistics
            self.motion_control_stats.position_error_samples.append(position_error)
            self.motion_control_stats.speed_error_samples.append(speed_error)
            self.motion_control_stats.trajectory_deviations.append(trajectory_deviation)
            self.motion_control_stats.smoothness_measurements.append(smoothness_score)
            
            # Limit sample history
            max_samples = 1000
            if len(self.motion_control_stats.position_error_samples) > max_samples:
                self.motion_control_stats.position_error_samples = \
                    self.motion_control_stats.position_error_samples[-max_samples:]
            if len(self.motion_control_stats.speed_error_samples) > max_samples:
                self.motion_control_stats.speed_error_samples = \
                    self.motion_control_stats.speed_error_samples[-max_samples:]
            
            # Create motion control performance metrics
            position_error_metric = PerformanceMetric(
                name="position_error",
                value=abs(position_error),
                unit="counts",
                timestamp=current_time,
                category=PerformanceCategory.MOTION_CONTROL,
                level=self.thresholds.evaluate_performance_level("position_error", position_error),
                threshold_exceeded=(abs(position_error) > self.thresholds.max_position_error)
            )
            
            trajectory_accuracy = self.motion_control_stats.trajectory_accuracy
            accuracy_metric = PerformanceMetric(
                name="trajectory_accuracy",
                value=trajectory_accuracy,
                unit="%",
                timestamp=current_time,
                category=PerformanceCategory.MOTION_CONTROL,
                level=self.thresholds.evaluate_performance_level("trajectory_accuracy", trajectory_accuracy),
                threshold_exceeded=(trajectory_accuracy < self.thresholds.min_trajectory_accuracy)
            )
            
            self.performance_metrics["position_error"] = position_error_metric
            self.performance_metrics["trajectory_accuracy"] = accuracy_metric
            
            self.performance_history["position_error"].append(position_error_metric)
            self.performance_history["trajectory_accuracy"].append(accuracy_metric)
            
        except Exception as e:
            self.logger.error(f"Error tracking motion control performance: {e}")
    
    def track_buffer_utilization(self, buffer_depth: int, is_optimal: bool = False):
        """
        Track buffer utilization efficiency.
        
        Args:
            buffer_depth: Current buffer depth (0-32)
            is_optimal: Whether current utilization is optimal
        """
        try:
            current_time = time.time()
            
            # Update buffer statistics
            self.buffer_stats.buffer_depth_samples.append(buffer_depth)
            
            if is_optimal:
                self.buffer_stats.optimal_utilization_time += (1.0 / self.monitoring_rate)
            
            self.buffer_stats.total_monitoring_time += (1.0 / self.monitoring_rate)
            
            # Check for overflow/underrun
            if buffer_depth >= 32:
                self.buffer_stats.buffer_overflow_count += 1
            elif buffer_depth == 0:
                self.buffer_stats.buffer_underrun_count += 1
            
            # Limit sample history
            max_samples = 1000
            if len(self.buffer_stats.buffer_depth_samples) > max_samples:
                self.buffer_stats.buffer_depth_samples = \
                    self.buffer_stats.buffer_depth_samples[-max_samples:]
            
            # Create buffer performance metrics
            buffer_efficiency = self.buffer_stats.buffer_efficiency
            efficiency_metric = PerformanceMetric(
                name="buffer_efficiency",
                value=buffer_efficiency,
                unit="%",
                timestamp=current_time,
                category=PerformanceCategory.BUFFER_UTILIZATION,
                level=self.thresholds.evaluate_performance_level("buffer_efficiency", buffer_efficiency),
                threshold_exceeded=(buffer_efficiency < self.thresholds.buffer_efficiency_threshold)
            )
            
            self.performance_metrics["buffer_efficiency"] = efficiency_metric
            self.performance_history["buffer_efficiency"].append(efficiency_metric)
            
        except Exception as e:
            self.logger.error(f"Error tracking buffer utilization: {e}")
    
    def generate_optimization_recommendations(self) -> List[OptimizationRecommendation]:
        """
        Generate performance optimization recommendations.
        
        Returns:
            List of optimization recommendations
        """
        recommendations = []
        
        try:
            # Command execution optimization
            if "command_latency" in self.performance_metrics:
                latency_metric = self.performance_metrics["command_latency"]
                if latency_metric.level in [PerformanceLevel.POOR, PerformanceLevel.CRITICAL]:
                    recommendations.append(OptimizationRecommendation(
                        category=PerformanceCategory.COMMAND_EXECUTION,
                        severity=latency_metric.level,
                        title="High Command Execution Latency",
                        description=f"Command latency of {latency_metric.value:.1f}ms exceeds optimal range.",
                        action_items=[
                            "Check serial port baud rate configuration",
                            "Reduce command frequency if possible",
                            "Implement command batching for multiple operations",
                            "Check for USB/serial port conflicts (consider Async Serial for production)"
                        ],
                        expected_improvement="Reduce latency to <5ms",
                        implementation_effort="medium"
                    ))
            
            # Communication optimization
            if "success_rate" in self.performance_metrics:
                success_metric = self.performance_metrics["success_rate"]
                if success_metric.level in [PerformanceLevel.POOR, PerformanceLevel.CRITICAL]:
                    recommendations.append(OptimizationRecommendation(
                        category=PerformanceCategory.COMMUNICATION,
                        severity=success_metric.level,
                        title="Low Communication Success Rate",
                        description=f"Success rate of {success_metric.value:.1f}% is below optimal threshold.",
                        action_items=[
                            "Check cable connections and quality",
                            "Verify controller address and baud rate settings",
                            "Implement retry logic with exponential backoff",
                            "Check for electromagnetic interference"
                        ],
                        expected_improvement="Achieve >95% success rate",
                        implementation_effort="low"
                    ))
            
            # Motion control optimization
            if "trajectory_accuracy" in self.performance_metrics:
                accuracy_metric = self.performance_metrics["trajectory_accuracy"]
                if accuracy_metric.level in [PerformanceLevel.POOR, PerformanceLevel.CRITICAL]:
                    recommendations.append(OptimizationRecommendation(
                        category=PerformanceCategory.MOTION_CONTROL,
                        severity=accuracy_metric.level,
                        title="Poor Trajectory Accuracy",
                        description=f"Trajectory accuracy of {accuracy_metric.value:.1f}% is below optimal threshold.",
                        action_items=[
                            "Tune PID controller parameters in Motion Studio",
                            "Check encoder calibration and mounting",
                            "Verify wheel parameters (radius, separation)",
                            "Reduce maximum acceleration/speed if needed",
                            "Check for mechanical play or backlash"
                        ],
                        expected_improvement="Achieve >95% trajectory accuracy",
                        implementation_effort="high"
                    ))
            
            # Buffer utilization optimization
            if "buffer_efficiency" in self.performance_metrics:
                buffer_metric = self.performance_metrics["buffer_efficiency"]
                if buffer_metric.level in [PerformanceLevel.POOR, PerformanceLevel.CRITICAL]:
                    recommendations.append(OptimizationRecommendation(
                        category=PerformanceCategory.BUFFER_UTILIZATION,
                        severity=buffer_metric.level,
                        title="Inefficient Buffer Utilization",
                        description=f"Buffer efficiency of {buffer_metric.value:.1f}% indicates suboptimal command scheduling.",
                        action_items=[
                            "Adjust command buffering strategy (target 2-8 commands)",
                            "Implement predictive buffer management",
                            "Optimize trajectory segment timing",
                            "Consider using distance-based commands for smoother motion"
                        ],
                        expected_improvement="Achieve >80% buffer efficiency",
                        implementation_effort="medium"
                    ))
            
            # Overall system recommendations
            current_time = time.time()
            if current_time - self.last_performance_evaluation > 300:  # 5 minutes
                overall_issues = sum(
                    1 for metric in self.performance_metrics.values()
                    if metric.level in [PerformanceLevel.POOR, PerformanceLevel.CRITICAL]
                )
                
                if overall_issues >= 2:
                    recommendations.append(OptimizationRecommendation(
                        category=PerformanceCategory.OVERALL_SYSTEM,
                        severity=PerformanceLevel.POOR,
                        title="Multiple Performance Issues Detected",
                        description=f"System has {overall_issues} performance issues requiring attention.",
                        action_items=[
                            "Perform comprehensive system health check",
                            "Review hardware configuration and connections",
                            "Update controller firmware if available",
                            "Consider hardware upgrade if performance requirements exceed capabilities"
                        ],
                        expected_improvement="Restore overall system performance",
                        implementation_effort="high"
                    ))
                
                self.last_performance_evaluation = current_time
            
            self.current_recommendations = recommendations
            
            # Publish recommendations if enabled
            if self.optimization_enabled and ROS2_AVAILABLE:
                self._publish_optimization_recommendations(recommendations)
            
            return recommendations
            
        except Exception as e:
            self.logger.error(f"Error generating optimization recommendations: {e}")
            return []
    
    def get_performance_report(self) -> Dict[str, Any]:
        """
        Generate comprehensive performance report.
        
        Returns:
            Dictionary containing complete performance data
        """
        try:
            current_time = time.time()
            
            # Generate current recommendations
            recommendations = self.generate_optimization_recommendations()
            
            report = {
                'timestamp': current_time,
                'monitoring_active': self.monitoring_active,
                'monitoring_duration': current_time - (self.last_performance_evaluation or current_time),
                
                # Command execution performance
                'command_execution': {
                    'recent_commands': len(self.command_timing_history),
                    'average_latency_ms': self.communication_stats.average_latency * 1000.0,
                    'success_rate': self.communication_stats.success_rate,
                    'max_latency_ms': self.communication_stats.max_latency * 1000.0,
                    'min_latency_ms': (self.communication_stats.min_latency * 1000.0 
                                     if self.communication_stats.min_latency != float('inf') else 0.0)
                },
                
                # Communication performance
                'communication': self.communication_stats.to_dict(),
                
                # Motion control performance
                'motion_control': self.motion_control_stats.to_dict(),
                
                # Buffer utilization
                'buffer_utilization': self.buffer_stats.to_dict(),
                
                # Current performance metrics
                'current_metrics': {
                    name: metric.to_dict() 
                    for name, metric in self.performance_metrics.items()
                },
                
                # Optimization recommendations
                'recommendations': [rec.to_dict() for rec in recommendations],
                
                # Overall system status
                'overall_status': self._evaluate_overall_performance()
            }
            
            return report
            
        except Exception as e:
            self.logger.error(f"Error generating performance report: {e}")
            return {'error': str(e), 'timestamp': time.time()}
    
    def _monitoring_loop(self):
        """Performance monitoring loop (runs in separate thread)"""
        try:
            monitoring_interval = 1.0 / self.monitoring_rate
            
            while self.monitoring_active:
                start_time = time.time()
                
                # Collect performance data from components
                self._collect_component_performance_data()
                
                # Sleep for remaining interval
                elapsed_time = time.time() - start_time
                sleep_time = max(0, monitoring_interval - elapsed_time)
                time.sleep(sleep_time)
                
        except Exception as e:
            self.logger.error(f"Error in monitoring loop: {e}")
            with self.monitoring_lock:
                self.monitoring_active = False
    
    def _collect_component_performance_data(self):
        """Collect performance data from hardware components"""
        try:
            # Collect trajectory monitor data
            if self.trajectory_monitor:
                # Get current trajectory status and errors
                pass  # Implementation depends on trajectory monitor interface
            
            # Collect buffer manager data
            if self.buffer_manager:
                # Get current buffer status
                pass  # Implementation depends on buffer manager interface
            
            # Collect hardware interface data
            if self.hardware_interface:
                # Get communication statistics
                pass  # Implementation depends on hardware interface
                
        except Exception as e:
            self.logger.error(f"Error collecting component performance data: {e}")
    
    def _evaluate_overall_performance(self) -> Dict[str, Any]:
        """Evaluate overall system performance"""
        try:
            if not self.performance_metrics:
                return {
                    'level': PerformanceLevel.GOOD.value,
                    'score': 100.0,
                    'issues': [],
                    'summary': "No performance data available"
                }
            
            # Calculate overall performance score
            level_weights = {
                PerformanceLevel.EXCELLENT: 100,
                PerformanceLevel.GOOD: 80,
                PerformanceLevel.ACCEPTABLE: 60,
                PerformanceLevel.POOR: 40,
                PerformanceLevel.CRITICAL: 20
            }
            
            total_score = 0
            metric_count = 0
            issues = []
            
            for name, metric in self.performance_metrics.items():
                total_score += level_weights.get(metric.level, 50)
                metric_count += 1
                
                if metric.level in [PerformanceLevel.POOR, PerformanceLevel.CRITICAL]:
                    issues.append(f"{name}: {metric.level.value}")
            
            overall_score = total_score / metric_count if metric_count > 0 else 100
            
            # Determine overall level
            if overall_score >= 90:
                overall_level = PerformanceLevel.EXCELLENT
            elif overall_score >= 75:
                overall_level = PerformanceLevel.GOOD
            elif overall_score >= 60:
                overall_level = PerformanceLevel.ACCEPTABLE
            elif overall_score >= 40:
                overall_level = PerformanceLevel.POOR
            else:
                overall_level = PerformanceLevel.CRITICAL
            
            return {
                'level': overall_level.value,
                'score': overall_score,
                'issues': issues,
                'summary': f"Overall performance: {overall_level.value} ({overall_score:.1f}/100)"
            }
            
        except Exception as e:
            self.logger.error(f"Error evaluating overall performance: {e}")
            return {
                'level': PerformanceLevel.POOR.value,
                'score': 0.0,
                'issues': [f"Evaluation error: {e}"],
                'summary': "Performance evaluation failed"
            }
    
    def _publish_optimization_recommendations(self, recommendations: List[OptimizationRecommendation]):
        """Publish optimization recommendations to ROS2 topic"""
        try:
            if not ROS2_AVAILABLE:
                return
            
            recommendations_data = {
                'timestamp': time.time(),
                'recommendations': [rec.to_dict() for rec in recommendations]
            }
            
            msg = String()
            msg.data = json.dumps(recommendations_data, indent=2)
            self.optimization_publisher.publish(msg)
            
        except Exception as e:
            self.logger.error(f"Error publishing optimization recommendations: {e}")
    
    def periodic_performance_report(self):
        """Periodic performance report (ROS2 timer callback)"""
        try:
            if not self.monitoring_active:
                return
            
            # Generate and publish performance report
            report = self.get_performance_report()
            
            if ROS2_AVAILABLE:
                msg = String()
                msg.data = json.dumps(report, indent=2)
                self.performance_publisher.publish(msg)
            
            # Log summary
            if 'overall_status' in report:
                overall = report['overall_status']
                self.logger.info(f"Performance Status: {overall['summary']}")
                
                if overall['issues']:
                    self.logger.warn(f"Performance Issues: {', '.join(overall['issues'])}")
            
        except Exception as e:
            self.logger.error(f"Error in periodic performance report: {e}")
    
    def register_optimization_callback(self, callback: Callable):
        """Register callback for optimization recommendations"""
        self.recommendation_callbacks.append(callback)
    
    def reset_performance_statistics(self):
        """Reset all performance statistics"""
        try:
            with self.monitoring_lock:
                self.command_timing_history.clear()
                self.communication_stats = CommunicationStats()
                self.motion_control_stats = MotionControlStats()
                self.buffer_stats = BufferStats()
                self.performance_metrics.clear()
                self.performance_history.clear()
                self.current_recommendations.clear()
                
                self.logger.info("Performance statistics reset")
                
        except Exception as e:
            self.logger.error(f"Error resetting performance statistics: {e}")


def main():
    """Main function for standalone testing"""
    if ROS2_AVAILABLE:
        rclpy.init()
        
        try:
            performance_monitor = PerformanceMonitor()
            performance_monitor.start_performance_monitoring()
            
            rclpy.spin(performance_monitor)
            
        finally:
            performance_monitor.stop_performance_monitoring()
            rclpy.shutdown()
    else:
        print("ROS2 not available - running in standalone mode")
        performance_monitor = PerformanceMonitor()
        performance_monitor.start_performance_monitoring()
        
        # Example usage
        import random
        for i in range(10):
            # Simulate command execution
            exec_time = random.uniform(0.001, 0.010)
            success = random.random() > 0.05
            performance_monitor.track_command_execution(
                "SpeedM1M2", exec_time, success
            )
            
            # Simulate motion control data
            pos_error = random.uniform(-500, 500)
            speed_error = random.uniform(-100, 100)
            performance_monitor.track_motion_control_performance(
                pos_error, speed_error
            )
            
            time.sleep(0.1)
        
        # Generate report
        report = performance_monitor.get_performance_report()
        print(json.dumps(report, indent=2))
        
        performance_monitor.stop_performance_monitoring()


if __name__ == '__main__':
    main()