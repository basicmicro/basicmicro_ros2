#!/usr/bin/env python3
"""
Trajectory Monitoring System for Basicmicro Driver

Implements comprehensive trajectory monitoring using position and speed error feedback,
with servo-specific monitoring capabilities and real-time trajectory execution feedback.

Author: ROS2 Driver Development
"""

import time
import math
import threading
from typing import Dict, Any, Optional, List, Tuple, Callable
from enum import Enum
from collections import deque
import statistics

# Handle ROS2 imports gracefully for testing
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
    from std_msgs.msg import Float32MultiArray, Bool
    from geometry_msgs.msg import Twist
    ROS2_AVAILABLE = True
except ImportError:
    # Mock ROS2 classes for testing
    ROS2_AVAILABLE = False
    
    class Node:
        def __init__(self, node_name):
            self.node_name = node_name
            self.parameters = {}
            self.publishers = {}
            self.timers = {}
            
        def declare_parameter(self, name, default_value):
            from unittest.mock import Mock
            self.parameters[name] = Mock()
            self.parameters[name].value = default_value
            
        def get_parameter(self, name):
            from unittest.mock import Mock
            return self.parameters.get(name, Mock())
        
        def create_publisher(self, msg_type, topic, qos):
            from unittest.mock import Mock
            return Mock()
        
        def create_timer(self, period, callback):
            from unittest.mock import Mock
            return Mock()
        
        def get_logger(self):
            from unittest.mock import Mock
            return Mock()

# Add Basicmicro Python library to path
import sys
import os
basicmicro_path = os.path.join(os.path.dirname(__file__), '..', '..', 'Basicmicro_python')
if os.path.exists(basicmicro_path):
    sys.path.insert(0, basicmicro_path)

try:
    from basicmicro import Basicmicro
except ImportError:
    # Use mock for testing without hardware
    from test_mocks.mock_basicmicro import MockBasicmicro as Basicmicro

try:
    from basicmicro_driver.unit_converter import UnitConverter
    from basicmicro_driver.buffer_manager import BufferManager
    from basicmicro_driver.servo_position_service import ServoPositionService, ServoState
except ImportError:
    # Fallback for relative import in package context
    from .unit_converter import UnitConverter
    from .buffer_manager import BufferManager
    from .servo_position_service import ServoPositionService, ServoState


class TrajectoryMonitoringState(Enum):
    """Trajectory monitoring states"""
    IDLE = "idle"
    MONITORING = "monitoring"
    TRAJECTORY_ACTIVE = "trajectory_active"
    ERROR_DETECTED = "error_detected"
    RECOVERY = "recovery"


class ErrorSeverity(Enum):
    """Error severity levels"""
    OK = "ok"
    WARNING = "warning" 
    ERROR = "error"
    CRITICAL = "critical"


class MonitoringMode(Enum):
    """Monitoring operation modes"""
    PASSIVE = "passive"          # Monitor only, no intervention
    ACTIVE = "active"            # Monitor and provide feedback
    CORRECTIVE = "corrective"    # Monitor and attempt corrections


class TrajectoryDeviation:
    """Container for trajectory deviation information"""
    
    def __init__(self, timestamp: float, left_pos_error: int, right_pos_error: int,
                 left_speed_error: int, right_speed_error: int, 
                 deviation_magnitude: float, severity: ErrorSeverity):
        self.timestamp = timestamp
        self.left_pos_error = left_pos_error
        self.right_pos_error = right_pos_error
        self.left_speed_error = left_speed_error
        self.right_speed_error = right_speed_error
        self.deviation_magnitude = deviation_magnitude
        self.severity = severity


class TrajectoryMonitor(Node if ROS2_AVAILABLE else object):
    """
    Comprehensive trajectory monitoring system with real-time error tracking,
    servo-specific monitoring, and configurable thresholds for different applications.
    """
    
    def __init__(self, hardware_interface=None, buffer_manager=None, 
                 unit_converter=None, servo_position_service=None):
        """
        Initialize trajectory monitoring system
        
        Args:
            hardware_interface: Hardware interface for controller communication
            buffer_manager: Buffer manager for buffer status monitoring
            unit_converter: Unit converter for position/velocity conversions
            servo_position_service: Servo position service for error monitoring
        """
        if ROS2_AVAILABLE:
            super().__init__('trajectory_monitor')
        
        self.hardware_interface = hardware_interface
        self.buffer_manager = buffer_manager
        self.unit_converter = unit_converter
        self.servo_position_service = servo_position_service
        
        # Monitoring state
        self.monitoring_state = TrajectoryMonitoringState.IDLE
        self.monitoring_active = False
        self.last_monitoring_time = 0.0
        
        # Configuration parameters
        self._setup_parameters()
        
        # Error tracking
        self.error_history = deque(maxlen=self.max_error_history)
        self.deviation_history = deque(maxlen=self.max_deviation_history)
        self.performance_metrics = {}
        
        # Current error state
        self.current_position_errors = (0, 0)
        self.current_speed_errors = (0, 0)
        self.current_trajectory_deviation = 0.0
        self.error_limit_violated = False
        self.controller_error_state = False
        
        # Monitoring thread control
        self.monitoring_thread = None
        self.monitoring_stop_event = threading.Event()
        self.monitoring_lock = threading.Lock()
        
        # Setup ROS2 interfaces if available
        if ROS2_AVAILABLE:
            self._setup_ros2_interfaces()
        
        # Performance tracking
        self.monitoring_start_time = time.time()
        self.total_monitoring_cycles = 0
        self.successful_monitoring_cycles = 0
        
        self.logger = self.get_logger() if ROS2_AVAILABLE else self._get_mock_logger()
        self.logger.info("Trajectory Monitor initialized")
    
    def _setup_parameters(self):
        """Setup monitoring configuration parameters"""
        # Monitoring rates and timing
        self.monitoring_rate_hz = 25.0  # Hz (respects communication timing)
        self.monitoring_period = 1.0 / self.monitoring_rate_hz
        
        # Error thresholds (configurable)
        self.position_error_threshold = 500      # encoder counts
        self.speed_error_threshold = 100         # counts/sec
        self.trajectory_deviation_threshold = 0.1  # meters
        
        # Servo error limits (configurable)
        self.max_position_error_limit = 1000    # encoder counts
        self.max_speed_error_limit = 200        # counts/sec
        
        # Monitoring behavior
        self.monitoring_mode = MonitoringMode.ACTIVE
        self.enable_deviation_detection = True
        self.enable_error_limit_monitoring = True
        self.enable_controller_error_detection = True
        
        # History and buffer sizes
        self.max_error_history = 100
        self.max_deviation_history = 50
        
        # Performance thresholds
        self.max_monitoring_latency_ms = 10.0
        self.communication_timeout_ms = 100.0
        
        # Recovery parameters
        self.error_recovery_enabled = True
        self.recovery_timeout_sec = 5.0
        
        if ROS2_AVAILABLE:
            # Declare ROS2 parameters
            self.declare_parameter('monitoring_rate_hz', self.monitoring_rate_hz)
            self.declare_parameter('position_error_threshold', self.position_error_threshold)
            self.declare_parameter('speed_error_threshold', self.speed_error_threshold)
            self.declare_parameter('trajectory_deviation_threshold', self.trajectory_deviation_threshold)
            self.declare_parameter('max_position_error_limit', self.max_position_error_limit)
            self.declare_parameter('max_speed_error_limit', self.max_speed_error_limit)
            self.declare_parameter('monitoring_mode', self.monitoring_mode.value)
            self.declare_parameter('enable_deviation_detection', self.enable_deviation_detection)
            self.declare_parameter('enable_error_limit_monitoring', self.enable_error_limit_monitoring)
            self.declare_parameter('enable_controller_error_detection', self.enable_controller_error_detection)
            
            # Update parameters from ROS2 values
            self._update_parameters_from_ros2()
    
    def _update_parameters_from_ros2(self):
        """Update parameters from ROS2 parameter server"""
        if not ROS2_AVAILABLE:
            return
            
        try:
            self.monitoring_rate_hz = self.get_parameter('monitoring_rate_hz').value
            self.monitoring_period = 1.0 / self.monitoring_rate_hz
            
            self.position_error_threshold = self.get_parameter('position_error_threshold').value
            self.speed_error_threshold = self.get_parameter('speed_error_threshold').value
            self.trajectory_deviation_threshold = self.get_parameter('trajectory_deviation_threshold').value
            
            self.max_position_error_limit = self.get_parameter('max_position_error_limit').value
            self.max_speed_error_limit = self.get_parameter('max_speed_error_limit').value
            
            mode_str = self.get_parameter('monitoring_mode').value
            self.monitoring_mode = MonitoringMode(mode_str)
            
            self.enable_deviation_detection = self.get_parameter('enable_deviation_detection').value
            self.enable_error_limit_monitoring = self.get_parameter('enable_error_limit_monitoring').value
            self.enable_controller_error_detection = self.get_parameter('enable_controller_error_detection').value
            
        except Exception as e:
            self.logger.warn(f"Error updating parameters from ROS2: {e}")
    
    def _setup_ros2_interfaces(self):
        """Setup ROS2 publishers and subscribers"""
        if not ROS2_AVAILABLE:
            return
            
        # Publishers
        self.trajectory_status_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics/trajectory_monitor', 10)
        
        self.error_feedback_pub = self.create_publisher(
            Float32MultiArray, '~/trajectory_errors', 10)
        
        self.deviation_alert_pub = self.create_publisher(
            Bool, '~/trajectory_deviation_alert', 10)
        
        # Monitoring timer
        self.monitoring_timer = self.create_timer(
            self.monitoring_period, self._monitoring_timer_callback)
    
    def _get_mock_logger(self):
        """Get mock logger for testing"""
        from unittest.mock import Mock
        return Mock()
    
    def start_monitoring(self) -> bool:
        """
        Start trajectory monitoring system
        
        Returns:
            True if monitoring started successfully, False otherwise
        """
        try:
            with self.monitoring_lock:
                if self.monitoring_active:
                    self.logger.info("Trajectory monitoring already active")
                    return True
                
                # Validate required components
                if not self._validate_monitoring_prerequisites():
                    return False
                
                # Reset monitoring state
                self.monitoring_state = TrajectoryMonitoringState.MONITORING
                self.monitoring_active = True
                self.monitoring_stop_event.clear()
                self.last_monitoring_time = time.time()
                
                # Start monitoring thread for high-frequency monitoring
                if not ROS2_AVAILABLE:
                    self.monitoring_thread = threading.Thread(
                        target=self._monitoring_thread_worker,
                        daemon=True
                    )
                    self.monitoring_thread.start()
                
                self.logger.info(f"Trajectory monitoring started at {self.monitoring_rate_hz} Hz")
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to start trajectory monitoring: {e}")
            return False
    
    def stop_monitoring(self) -> bool:
        """
        Stop trajectory monitoring system
        
        Returns:
            True if monitoring stopped successfully, False otherwise
        """
        try:
            with self.monitoring_lock:
                if not self.monitoring_active:
                    self.logger.info("Trajectory monitoring already inactive")
                    return True
                
                # Signal stop to monitoring thread
                self.monitoring_stop_event.set()
                self.monitoring_active = False
                self.monitoring_state = TrajectoryMonitoringState.IDLE
                
                # Wait for thread to finish
                if self.monitoring_thread and self.monitoring_thread.is_alive():
                    self.monitoring_thread.join(timeout=1.0)
                
                self.logger.info("Trajectory monitoring stopped")
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to stop trajectory monitoring: {e}")
            return False
    
    def _validate_monitoring_prerequisites(self) -> bool:
        """Validate that required components are available for monitoring"""
        if not self.hardware_interface:
            self.logger.error("Hardware interface not available for monitoring")
            return False
        
        if not hasattr(self.hardware_interface, 'controller'):
            self.logger.error("Controller not available in hardware interface")
            return False
        
        if not self.unit_converter:
            self.logger.warn("Unit converter not available - using default conversions")
        
        return True
    
    def _monitoring_thread_worker(self):
        """Worker thread for high-frequency monitoring"""
        self.logger.info("Trajectory monitoring thread started")
        
        while not self.monitoring_stop_event.is_set():
            try:
                cycle_start_time = time.time()
                
                # Perform monitoring cycle
                self._perform_monitoring_cycle()
                
                # Track performance
                cycle_duration = time.time() - cycle_start_time
                self._update_performance_metrics(cycle_duration)
                
                # Sleep for remaining time to maintain rate
                sleep_time = max(0, self.monitoring_period - cycle_duration)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"Error in monitoring thread: {e}")
                time.sleep(self.monitoring_period)  # Continue monitoring despite error
        
        self.logger.info("Trajectory monitoring thread stopped")
    
    def _monitoring_timer_callback(self):
        """ROS2 timer callback for monitoring"""
        if self.monitoring_active:
            self._perform_monitoring_cycle()
    
    def _perform_monitoring_cycle(self):
        """Perform a single monitoring cycle"""
        try:
            cycle_start_time = time.time()
            
            # Read error data
            position_errors, speed_errors = self._read_error_data()
            
            # Update current state
            self.current_position_errors = position_errors
            self.current_speed_errors = speed_errors
            
            # Detect trajectory deviations
            if self.enable_deviation_detection:
                deviation = self._detect_trajectory_deviation(position_errors, speed_errors)
                if deviation:
                    self.deviation_history.append(deviation)
                    self._handle_trajectory_deviation(deviation)
            
            # Monitor error limits
            if self.enable_error_limit_monitoring:
                self._monitor_servo_error_limits(position_errors, speed_errors)
            
            # Check controller error state
            if self.enable_controller_error_detection:
                self._check_controller_error_state()
            
            # Update error history
            self._update_error_history(position_errors, speed_errors, cycle_start_time)
            
            # Publish monitoring feedback
            self._publish_monitoring_feedback()
            
            self.total_monitoring_cycles += 1
            self.successful_monitoring_cycles += 1
            
        except Exception as e:
            self.logger.error(f"Error in monitoring cycle: {e}")
            self.total_monitoring_cycles += 1
    
    def _read_error_data(self) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        """
        Read position and speed error data from controller
        
        Returns:
            Tuple of ((left_pos_error, right_pos_error), (left_speed_error, right_speed_error))
        """
        position_errors = (0, 0)
        speed_errors = (0, 0)
        
        try:
            if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
                return position_errors, speed_errors
            
            controller = self.hardware_interface.controller
            address = getattr(self.hardware_interface, 'address', 0x80)
            
            # Read position errors (per PID cycle: RoboClaw 1/300s, MCP 1/625s)
            pos_result = controller.GetPosErrors(address)
            if pos_result[0]:  # Success
                position_errors = (pos_result[1], pos_result[2])
            
            # Read speed errors (per PID cycle)
            speed_result = controller.GetSpeedErrors(address)
            if speed_result[0]:  # Success
                speed_errors = (speed_result[1], speed_result[2])
            
        except Exception as e:
            self.logger.error(f"Failed to read error data: {e}")
        
        return position_errors, speed_errors
    
    def _detect_trajectory_deviation(self, position_errors: Tuple[int, int], 
                                   speed_errors: Tuple[int, int]) -> Optional[TrajectoryDeviation]:
        """
        Detect trajectory deviations based on error thresholds
        
        Args:
            position_errors: Left and right position errors (encoder counts)
            speed_errors: Left and right speed errors (counts/sec)
            
        Returns:
            TrajectoryDeviation object if deviation detected, None otherwise
        """
        try:
            # Calculate error magnitudes
            max_pos_error = max(abs(position_errors[0]), abs(position_errors[1]))
            max_speed_error = max(abs(speed_errors[0]), abs(speed_errors[1]))
            
            # Convert to physical units for deviation calculation
            if self.unit_converter:
                pos_deviation_rad = max(
                    abs(self.unit_converter.counts_to_radians(position_errors[0])),
                    abs(self.unit_converter.counts_to_radians(position_errors[1]))
                )
                # Convert to linear distance at wheel radius
                wheel_radius = getattr(self.hardware_interface, 'wheel_radius', 0.1)
                deviation_meters = pos_deviation_rad * wheel_radius
            else:
                # Fallback calculation
                deviation_meters = max_pos_error * 0.001  # Rough approximation
            
            # Check if deviation exceeds thresholds
            pos_threshold_exceeded = max_pos_error > self.position_error_threshold
            speed_threshold_exceeded = max_speed_error > self.speed_error_threshold
            trajectory_threshold_exceeded = deviation_meters > self.trajectory_deviation_threshold
            
            if pos_threshold_exceeded or speed_threshold_exceeded or trajectory_threshold_exceeded:
                # Determine severity
                severity = ErrorSeverity.OK
                if trajectory_threshold_exceeded or max_pos_error > (self.position_error_threshold * 2):
                    severity = ErrorSeverity.CRITICAL
                elif pos_threshold_exceeded or speed_threshold_exceeded:
                    severity = ErrorSeverity.WARNING
                
                return TrajectoryDeviation(
                    timestamp=time.time(),
                    left_pos_error=position_errors[0],
                    right_pos_error=position_errors[1],
                    left_speed_error=speed_errors[0],
                    right_speed_error=speed_errors[1],
                    deviation_magnitude=deviation_meters,
                    severity=severity
                )
            
        except Exception as e:
            self.logger.error(f"Error detecting trajectory deviation: {e}")
        
        return None
    
    def _handle_trajectory_deviation(self, deviation: TrajectoryDeviation):
        """Handle detected trajectory deviation"""
        try:
            # Log deviation
            self.logger.warn(f"Trajectory deviation detected: {deviation.deviation_magnitude:.3f}m, "
                           f"severity: {deviation.severity.value}")
            
            # Update monitoring state
            if deviation.severity in [ErrorSeverity.ERROR, ErrorSeverity.CRITICAL]:
                self.monitoring_state = TrajectoryMonitoringState.ERROR_DETECTED
            
            # Publish deviation alert
            if ROS2_AVAILABLE and hasattr(self, 'deviation_alert_pub'):
                alert_msg = Bool()
                alert_msg.data = True
                self.deviation_alert_pub.publish(alert_msg)
            
            # Take corrective action if enabled
            if (self.monitoring_mode == MonitoringMode.CORRECTIVE and 
                deviation.severity == ErrorSeverity.CRITICAL):
                self._attempt_trajectory_correction(deviation)
                
        except Exception as e:
            self.logger.error(f"Error handling trajectory deviation: {e}")
    
    def _monitor_servo_error_limits(self, position_errors: Tuple[int, int], 
                                  speed_errors: Tuple[int, int]):
        """Monitor for servo error limit violations"""
        try:
            max_pos_error = max(abs(position_errors[0]), abs(position_errors[1]))
            max_speed_error = max(abs(speed_errors[0]), abs(speed_errors[1]))
            
            # Check error limits
            pos_limit_exceeded = max_pos_error > self.max_position_error_limit
            speed_limit_exceeded = max_speed_error > self.max_speed_error_limit
            
            previous_violation = self.error_limit_violated
            self.error_limit_violated = pos_limit_exceeded or speed_limit_exceeded
            
            # Log limit violations
            if self.error_limit_violated and not previous_violation:
                self.logger.error(f"Servo error limits exceeded - "
                                f"Position: {max_pos_error}/{self.max_position_error_limit}, "
                                f"Speed: {max_speed_error}/{self.max_speed_error_limit}")
            elif not self.error_limit_violated and previous_violation:
                self.logger.info("Servo error limits returned to normal")
                
        except Exception as e:
            self.logger.error(f"Error monitoring servo error limits: {e}")
    
    def _check_controller_error_state(self):
        """Check for controller error state requiring Power-On-Reset"""
        try:
            if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
                return
            
            controller = self.hardware_interface.controller
            address = getattr(self.hardware_interface, 'address', 0x80)
            
            # Read error status from controller
            error_result = controller.ReadError(address)
            
            previous_error_state = self.controller_error_state
            
            if error_result[0]:  # Success reading error status
                # Check for error conditions (implementation depends on controller type)
                self.controller_error_state = error_result[1] != 0
            else:
                # Communication failure could indicate error state
                self.controller_error_state = True
            
            # Log error state changes
            if self.controller_error_state and not previous_error_state:
                self.logger.error("Controller error state detected - Power-On-Reset may be required")
                self.monitoring_state = TrajectoryMonitoringState.ERROR_DETECTED
            elif not self.controller_error_state and previous_error_state:
                self.logger.info("Controller error state cleared")
                
        except Exception as e:
            self.logger.error(f"Error checking controller error state: {e}")
    
    def _attempt_trajectory_correction(self, deviation: TrajectoryDeviation):
        """Attempt to correct trajectory deviation"""
        try:
            self.logger.info("Attempting trajectory correction for critical deviation")
            
            # Basic correction strategy: reduce speed to allow system to catch up
            if self.hardware_interface and hasattr(self.hardware_interface, 'controller'):
                controller = self.hardware_interface.controller
                address = getattr(self.hardware_interface, 'address', 0x80)
                
                # Apply reduced speed command to allow system to stabilize
                # This is a simple corrective action - more sophisticated corrections
                # would require trajectory replanning
                controller.SpeedM1M2(address, 0, 0)  # Temporary stop
                
                self.monitoring_state = TrajectoryMonitoringState.RECOVERY
                self.logger.info("Applied emergency stop for trajectory correction")
            
        except Exception as e:
            self.logger.error(f"Error attempting trajectory correction: {e}")
    
    def _update_error_history(self, position_errors: Tuple[int, int], 
                            speed_errors: Tuple[int, int], timestamp: float):
        """Update error history for trend analysis"""
        try:
            error_entry = {
                'timestamp': timestamp,
                'left_pos_error': position_errors[0],
                'right_pos_error': position_errors[1],
                'left_speed_error': speed_errors[0],
                'right_speed_error': speed_errors[1],
                'max_pos_error': max(abs(position_errors[0]), abs(position_errors[1])),
                'max_speed_error': max(abs(speed_errors[0]), abs(speed_errors[1]))
            }
            
            self.error_history.append(error_entry)
            
        except Exception as e:
            self.logger.error(f"Error updating error history: {e}")
    
    def _update_performance_metrics(self, cycle_duration: float):
        """Update performance monitoring metrics"""
        try:
            cycle_duration_ms = cycle_duration * 1000.0
            
            if 'cycle_durations' not in self.performance_metrics:
                self.performance_metrics['cycle_durations'] = deque(maxlen=100)
                self.performance_metrics['max_cycle_duration'] = 0.0
                self.performance_metrics['avg_cycle_duration'] = 0.0
            
            self.performance_metrics['cycle_durations'].append(cycle_duration_ms)
            self.performance_metrics['max_cycle_duration'] = max(
                self.performance_metrics['max_cycle_duration'], cycle_duration_ms)
            
            # Calculate rolling average
            if len(self.performance_metrics['cycle_durations']) > 0:
                self.performance_metrics['avg_cycle_duration'] = statistics.mean(
                    self.performance_metrics['cycle_durations'])
            
            # Alert if performance is degraded
            if cycle_duration_ms > self.max_monitoring_latency_ms:
                self.logger.warn(f"Monitoring cycle exceeded target latency: {cycle_duration_ms:.1f}ms")
                
        except Exception as e:
            self.logger.error(f"Error updating performance metrics: {e}")
    
    def _publish_monitoring_feedback(self):
        """Publish monitoring feedback and diagnostics"""
        if not ROS2_AVAILABLE:
            return
            
        try:
            # Publish error feedback
            if hasattr(self, 'error_feedback_pub'):
                error_msg = Float32MultiArray()
                error_msg.data = [
                    float(self.current_position_errors[0]),
                    float(self.current_position_errors[1]),
                    float(self.current_speed_errors[0]),
                    float(self.current_speed_errors[1]),
                    float(self.current_trajectory_deviation)
                ]
                self.error_feedback_pub.publish(error_msg)
            
            # Publish diagnostic status
            if hasattr(self, 'trajectory_status_pub'):
                diagnostic_array = DiagnosticArray()
                diagnostic_array.header.stamp = self.get_clock().now().to_msg()
                
                # Create monitoring status diagnostic
                status = DiagnosticStatus()
                status.name = "trajectory_monitor/monitoring_status"
                status.hardware_id = "basicmicro_controller"
                
                if self.controller_error_state:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "Controller error state detected"
                elif self.error_limit_violated:
                    status.level = DiagnosticStatus.ERROR
                    status.message = "Servo error limits exceeded"
                elif self.current_trajectory_deviation > self.trajectory_deviation_threshold:
                    status.level = DiagnosticStatus.WARN
                    status.message = f"Trajectory deviation: {self.current_trajectory_deviation:.3f}m"
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = f"Monitoring active: {self.monitoring_state.value}"
                
                # Add diagnostic values
                status.values = [
                    KeyValue(key="monitoring_state", value=self.monitoring_state.value),
                    KeyValue(key="monitoring_rate_hz", value=str(self.monitoring_rate_hz)),
                    KeyValue(key="left_pos_error", value=str(self.current_position_errors[0])),
                    KeyValue(key="right_pos_error", value=str(self.current_position_errors[1])),
                    KeyValue(key="left_speed_error", value=str(self.current_speed_errors[0])),
                    KeyValue(key="right_speed_error", value=str(self.current_speed_errors[1])),
                    KeyValue(key="trajectory_deviation_m", value=f"{self.current_trajectory_deviation:.6f}"),
                    KeyValue(key="error_limit_violated", value=str(self.error_limit_violated)),
                    KeyValue(key="controller_error_state", value=str(self.controller_error_state)),
                    KeyValue(key="total_cycles", value=str(self.total_monitoring_cycles)),
                    KeyValue(key="successful_cycles", value=str(self.successful_monitoring_cycles))
                ]
                
                # Add performance metrics
                if 'avg_cycle_duration' in self.performance_metrics:
                    status.values.append(KeyValue(
                        key="avg_cycle_duration_ms", 
                        value=f"{self.performance_metrics['avg_cycle_duration']:.2f}"))
                
                diagnostic_array.status.append(status)
                self.trajectory_status_pub.publish(diagnostic_array)
                
        except Exception as e:
            self.logger.error(f"Error publishing monitoring feedback: {e}")
    
    def get_monitoring_status(self) -> Dict[str, Any]:
        """
        Get current monitoring status and metrics
        
        Returns:
            Dictionary with comprehensive monitoring status
        """
        try:
            uptime = time.time() - self.monitoring_start_time
            success_rate = (self.successful_monitoring_cycles / max(self.total_monitoring_cycles, 1)) * 100.0
            
            status = {
                'monitoring_active': self.monitoring_active,
                'monitoring_state': self.monitoring_state.value,
                'monitoring_rate_hz': self.monitoring_rate_hz,
                'uptime_seconds': uptime,
                'total_cycles': self.total_monitoring_cycles,
                'successful_cycles': self.successful_monitoring_cycles,
                'success_rate_percent': success_rate,
                
                # Current error state
                'current_position_errors': self.current_position_errors,
                'current_speed_errors': self.current_speed_errors,
                'current_trajectory_deviation': self.current_trajectory_deviation,
                'error_limit_violated': self.error_limit_violated,
                'controller_error_state': self.controller_error_state,
                
                # Configuration
                'position_error_threshold': self.position_error_threshold,
                'speed_error_threshold': self.speed_error_threshold,
                'trajectory_deviation_threshold': self.trajectory_deviation_threshold,
                'monitoring_mode': self.monitoring_mode.value,
                
                # Performance metrics
                'performance_metrics': self.performance_metrics.copy(),
                
                # History statistics
                'error_history_size': len(self.error_history),
                'deviation_history_size': len(self.deviation_history)
            }
            
            # Add error statistics if history available
            if len(self.error_history) > 0:
                recent_errors = list(self.error_history)[-10:]  # Last 10 entries
                max_pos_errors = [e['max_pos_error'] for e in recent_errors]
                max_speed_errors = [e['max_speed_error'] for e in recent_errors]
                
                status['recent_error_stats'] = {
                    'avg_pos_error': statistics.mean(max_pos_errors),
                    'max_pos_error': max(max_pos_errors),
                    'avg_speed_error': statistics.mean(max_speed_errors),
                    'max_speed_error': max(max_speed_errors)
                }
            
            return status
            
        except Exception as e:
            self.logger.error(f"Error getting monitoring status: {e}")
            return {'error': str(e)}
    
    def get_trajectory_deviation_history(self) -> List[Dict[str, Any]]:
        """
        Get history of trajectory deviations
        
        Returns:
            List of deviation entries
        """
        try:
            return [
                {
                    'timestamp': deviation.timestamp,
                    'left_pos_error': deviation.left_pos_error,
                    'right_pos_error': deviation.right_pos_error,
                    'left_speed_error': deviation.left_speed_error,
                    'right_speed_error': deviation.right_speed_error,
                    'deviation_magnitude': deviation.deviation_magnitude,
                    'severity': deviation.severity.value
                }
                for deviation in self.deviation_history
            ]
        except Exception as e:
            self.logger.error(f"Error getting deviation history: {e}")
            return []
    
    def reset_monitoring_statistics(self):
        """Reset monitoring statistics and history"""
        try:
            with self.monitoring_lock:
                self.error_history.clear()
                self.deviation_history.clear()
                self.performance_metrics.clear()
                self.total_monitoring_cycles = 0
                self.successful_monitoring_cycles = 0
                self.monitoring_start_time = time.time()
                
                self.logger.info("Monitoring statistics reset")
                
        except Exception as e:
            self.logger.error(f"Error resetting monitoring statistics: {e}")
    
    def update_monitoring_configuration(self, config: Dict[str, Any]) -> bool:
        """
        Update monitoring configuration parameters
        
        Args:
            config: Dictionary with configuration parameters
            
        Returns:
            True if configuration updated successfully, False otherwise
        """
        try:
            with self.monitoring_lock:
                # Update thresholds
                if 'position_error_threshold' in config:
                    self.position_error_threshold = config['position_error_threshold']
                
                if 'speed_error_threshold' in config:
                    self.speed_error_threshold = config['speed_error_threshold']
                
                if 'trajectory_deviation_threshold' in config:
                    self.trajectory_deviation_threshold = config['trajectory_deviation_threshold']
                
                if 'max_position_error_limit' in config:
                    self.max_position_error_limit = config['max_position_error_limit']
                
                if 'max_speed_error_limit' in config:
                    self.max_speed_error_limit = config['max_speed_error_limit']
                
                # Update monitoring behavior
                if 'monitoring_mode' in config:
                    self.monitoring_mode = MonitoringMode(config['monitoring_mode'])
                
                if 'enable_deviation_detection' in config:
                    self.enable_deviation_detection = config['enable_deviation_detection']
                
                if 'enable_error_limit_monitoring' in config:
                    self.enable_error_limit_monitoring = config['enable_error_limit_monitoring']
                
                if 'enable_controller_error_detection' in config:
                    self.enable_controller_error_detection = config['enable_controller_error_detection']
                
                # Update monitoring rate (requires restart of monitoring)
                if 'monitoring_rate_hz' in config:
                    new_rate = config['monitoring_rate_hz']
                    if new_rate != self.monitoring_rate_hz:
                        self.monitoring_rate_hz = new_rate
                        self.monitoring_period = 1.0 / new_rate
                        
                        # Restart monitoring if active
                        if self.monitoring_active:
                            self.stop_monitoring()
                            time.sleep(0.1)
                            self.start_monitoring()
                
                self.logger.info("Monitoring configuration updated")
                return True
                
        except Exception as e:
            self.logger.error(f"Error updating monitoring configuration: {e}")
            return False


# Standalone test functions for hardware-free testing
def test_trajectory_monitor():
    """Test trajectory monitoring functionality"""
    print("Testing Trajectory Monitor...")
    
    # Create mock objects
    from unittest.mock import Mock
    
    hardware_interface = Mock()
    hardware_interface.controller = Mock()
    hardware_interface.address = 0x80
    hardware_interface.wheel_radius = 0.1
    hardware_interface.controller.GetPosErrors.return_value = (True, 150, -75)
    hardware_interface.controller.GetSpeedErrors.return_value = (True, 30, -20)
    hardware_interface.controller.ReadError.return_value = (True, 0)
    
    buffer_manager = Mock()
    unit_converter = Mock()
    unit_converter.counts_to_radians.side_effect = lambda x: x * 0.001
    servo_position_service = Mock()
    
    # Create trajectory monitor
    monitor = TrajectoryMonitor(hardware_interface, buffer_manager, unit_converter, servo_position_service)
    
    # Test monitoring lifecycle
    assert monitor.start_monitoring() == True
    print("✓ Monitoring started successfully")
    
    # Test single monitoring cycle
    monitor._perform_monitoring_cycle()
    print("✓ Monitoring cycle executed")
    
    # Test status retrieval
    status = monitor.get_monitoring_status()
    assert status['monitoring_active'] == True
    assert status['monitoring_state'] == 'monitoring'
    print("✓ Status retrieval works")
    
    # Test configuration update
    config = {
        'position_error_threshold': 200,
        'speed_error_threshold': 50,
        'monitoring_mode': 'corrective'
    }
    assert monitor.update_monitoring_configuration(config) == True
    print("✓ Configuration update works")
    
    # Test monitoring stop
    assert monitor.stop_monitoring() == True
    print("✓ Monitoring stopped successfully")
    
    # Test deviation detection
    deviation = monitor._detect_trajectory_deviation((600, -300), (150, 75))
    assert deviation is not None
    assert deviation.severity in [ErrorSeverity.WARNING, ErrorSeverity.CRITICAL]
    print("✓ Trajectory deviation detection works")
    
    print("All trajectory monitor tests passed!")


if __name__ == "__main__":
    test_trajectory_monitor()