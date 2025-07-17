#!/usr/bin/env python3
"""
Buffer Management System for Basicmicro Driver

Implements comprehensive buffer status monitoring and management for optimal motion control.
Provides real-time buffer tracking, overflow prevention, and optimization strategies.

Author: ROS2 Driver Development
"""

import time
from typing import Dict, Any, Optional, List, Tuple
from enum import Enum
import threading
from collections import deque
import statistics

# Handle ROS2 imports gracefully for testing
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
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
        
        def get_clock(self):
            from unittest.mock import Mock
            clock = Mock()
            clock.now.return_value.to_msg.return_value = Mock()
            return clock
        
        def get_logger(self):
            from unittest.mock import Mock
            return Mock()
    
    # Mock diagnostic message classes
    class DiagnosticStatus:
        OK = 0
        WARN = 1
        ERROR = 2
        
        def __init__(self):
            self.name = ""
            self.hardware_id = ""
            self.level = self.OK
            self.message = ""
            self.values = []
    
    class DiagnosticArray:
        def __init__(self):
            self.header = type('Header', (), {'stamp': None})()
            self.status = []
    
    class KeyValue:
        def __init__(self, key="", value=""):
            self.key = key
            self.value = value


class BufferStatus(Enum):
    """Buffer status interpretation from ReadBuffers command."""
    IDLE = "IDLE"                    # 0xFF - No commands in buffer, no commands executing
    EXECUTING = "EXECUTING"          # 0x00 - Last command executing, buffer empty
    BUFFERED = "BUFFERED"           # 1-32 - Number of commands in buffer


class BufferManager(Node):
    """
    Comprehensive buffer management system for Basicmicro motor controllers.
    
    Features:
    - Real-time buffer status monitoring using ReadBuffers command
    - Buffer overflow prevention with configurable thresholds
    - Buffer optimization strategies for smooth motion
    - Diagnostic reporting for buffer performance
    - Historical buffer usage tracking and analysis
    """
    
    def __init__(self, hardware_interface=None):
        """
        Initialize buffer management system.
        
        Args:
            hardware_interface: Reference to hardware interface for buffer access
        """
        super().__init__('buffer_manager')
        
        # Hardware interface reference
        self.hardware_interface = hardware_interface
        
        # Buffer configuration
        self.max_buffer_size = 32  # Maximum commands per motor channel
        self.min_buffer_available = 4  # Minimum buffer slots to keep available
        self.buffer_warning_threshold = 28  # Warn when buffer usage exceeds this
        self.buffer_critical_threshold = 30  # Critical threshold for buffer usage
        
        # Monitoring configuration
        self.monitoring_rate_hz = 20.0  # Buffer status monitoring frequency
        self.monitoring_enabled = True
        
        # Buffer status tracking
        self.current_buffer_status = {
            'status': BufferStatus.IDLE,
            'used_slots': 0,
            'available_slots': self.max_buffer_size,
            'utilization_percent': 0.0,
            'last_updated': time.time()
        }
        
        # Historical tracking
        self.buffer_history = deque(maxlen=1000)  # Last 1000 readings
        self.utilization_history = deque(maxlen=100)  # Last 100 utilization readings
        
        # Performance statistics
        self.stats = {
            'total_reads': 0,
            'failed_reads': 0,
            'max_utilization': 0.0,
            'avg_utilization': 0.0,
            'overflow_warnings': 0,
            'overflow_preventions': 0
        }
        
        # Optimization parameters
        self.optimal_utilization_target = 0.6  # Target 60% utilization for smooth operation
        self.hysteresis_threshold = 0.1  # Prevent oscillation in optimization decisions
        
        # Declare parameters for buffer management
        self._declare_buffer_parameters()
        
        # Initialize diagnostic publisher
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # Initialize monitoring timer
        self.monitoring_timer = self.create_timer(
            1.0 / self.monitoring_rate_hz,
            self._monitor_buffer_status
        )
        
        # Thread safety
        self._status_lock = threading.Lock()
        
        self.get_logger().info("Buffer management system initialized")
    
    def _declare_buffer_parameters(self):
        """Declare ROS2 parameters for buffer management configuration."""
        self.declare_parameter('buffer_management.max_buffer_size', self.max_buffer_size)
        self.declare_parameter('buffer_management.min_buffer_available', self.min_buffer_available)
        self.declare_parameter('buffer_management.warning_threshold', self.buffer_warning_threshold)
        self.declare_parameter('buffer_management.critical_threshold', self.buffer_critical_threshold)
        self.declare_parameter('buffer_management.monitoring_rate_hz', self.monitoring_rate_hz)
        self.declare_parameter('buffer_management.optimal_utilization_target', self.optimal_utilization_target)
        
        # Update configuration from parameters
        self._update_configuration_from_parameters()
    
    def _update_configuration_from_parameters(self):
        """Update buffer management configuration from ROS2 parameters."""
        self.max_buffer_size = self.get_parameter('buffer_management.max_buffer_size').value
        self.min_buffer_available = self.get_parameter('buffer_management.min_buffer_available').value
        self.buffer_warning_threshold = self.get_parameter('buffer_management.warning_threshold').value
        self.buffer_critical_threshold = self.get_parameter('buffer_management.critical_threshold').value
        self.monitoring_rate_hz = self.get_parameter('buffer_management.monitoring_rate_hz').value
        self.optimal_utilization_target = self.get_parameter('buffer_management.optimal_utilization_target').value
    
    def _monitor_buffer_status(self):
        """
        Periodic buffer status monitoring callback.
        
        Reads buffer status from hardware and updates internal tracking.
        """
        if not self.monitoring_enabled:
            return
        
        try:
            # Read buffer status from hardware
            buffer_data = self._read_buffer_status()
            
            if buffer_data['success']:
                with self._status_lock:
                    # Update current status
                    self.current_buffer_status.update(buffer_data)
                    self.current_buffer_status['last_updated'] = time.time()
                    
                    # Add to history
                    self.buffer_history.append({
                        'timestamp': time.time(),
                        'used_slots': buffer_data['used_slots'],
                        'utilization_percent': buffer_data['utilization_percent'],
                        'status': buffer_data['status']
                    })
                    
                    # Update utilization history
                    self.utilization_history.append(buffer_data['utilization_percent'])
                    
                    # Update statistics
                    self.stats['total_reads'] += 1
                    self.stats['max_utilization'] = max(
                        self.stats['max_utilization'], 
                        buffer_data['utilization_percent']
                    )
                    
                    if len(self.utilization_history) > 0:
                        self.stats['avg_utilization'] = statistics.mean(self.utilization_history)
                    
                    # Check for warnings/critical conditions
                    self._check_buffer_thresholds(buffer_data)
                
                # Publish diagnostics
                self._publish_buffer_diagnostics()
                
            else:
                with self._status_lock:
                    self.stats['failed_reads'] += 1
                
                self.get_logger().warning(f"Buffer status read failed: {buffer_data.get('message', 'Unknown error')}")
                
        except Exception as e:
            with self._status_lock:
                self.stats['failed_reads'] += 1
            self.get_logger().error(f"Buffer monitoring error: {str(e)}")
    
    def _read_buffer_status(self) -> Dict[str, Any]:
        """
        Read buffer status from hardware controller.
        
        Returns:
            Dictionary with buffer status information
        """
        if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
            # For testing without hardware interface
            return {
                'success': True,
                'status': BufferStatus.IDLE,
                'used_slots': 0,
                'available_slots': self.max_buffer_size,
                'utilization_percent': 0.0,
                'raw_value': 0xFF,
                'message': 'Simulated buffer status (no hardware interface)'
            }
        
        try:
            # Read buffer status using Basicmicro library
            buffer_result = self.hardware_interface.controller.ReadBuffers(
                self.hardware_interface.address
            )
            
            if not buffer_result[0]:
                return {
                    'success': False,
                    'message': 'Failed to read buffer status from controller'
                }
            
            # Interpret buffer status from raw value
            raw_value = buffer_result[1]
            
            if raw_value == 0xFF:
                # No commands in buffer, no commands executing
                status = BufferStatus.IDLE
                used_slots = 0
            elif raw_value == 0:
                # Last command is executing, buffer empty
                status = BufferStatus.EXECUTING
                used_slots = 0
            else:
                # raw_value = number of commands in buffer (1-32)
                status = BufferStatus.BUFFERED
                used_slots = raw_value
            
            available_slots = self.max_buffer_size - used_slots
            utilization_percent = (used_slots / self.max_buffer_size) * 100.0
            
            return {
                'success': True,
                'status': status,
                'used_slots': used_slots,
                'available_slots': available_slots,
                'utilization_percent': utilization_percent,
                'raw_value': raw_value,
                'message': f'Buffer status: {status.value}, {used_slots}/{self.max_buffer_size} slots used'
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'Buffer status read exception: {str(e)}'
            }
    
    def _check_buffer_thresholds(self, buffer_data: Dict[str, Any]):
        """
        Check buffer status against warning and critical thresholds.
        
        Args:
            buffer_data: Current buffer status data
        """
        used_slots = buffer_data['used_slots']
        utilization_percent = buffer_data['utilization_percent']
        
        # Check critical threshold
        if used_slots >= self.buffer_critical_threshold:
            self.get_logger().error(
                f"CRITICAL: Buffer usage at {used_slots}/{self.max_buffer_size} slots ({utilization_percent:.1f}%)"
            )
            self.stats['overflow_warnings'] += 1
            
        # Check warning threshold
        elif used_slots >= self.buffer_warning_threshold:
            self.get_logger().warning(
                f"WARNING: High buffer usage at {used_slots}/{self.max_buffer_size} slots ({utilization_percent:.1f}%)"
            )
    
    def _publish_buffer_diagnostics(self):
        """Publish buffer status diagnostics to ROS2 diagnostic system."""
        try:
            # Create diagnostic array message
            diagnostic_array = DiagnosticArray()
            diagnostic_array.header.stamp = self.get_clock().now().to_msg()
            
            # Create buffer status diagnostic
            buffer_status = DiagnosticStatus()
            buffer_status.name = "buffer_manager/buffer_status"
            buffer_status.hardware_id = "basicmicro_controller"
            
            with self._status_lock:
                current_status = self.current_buffer_status
                stats = self.stats.copy()
            
            # Determine diagnostic level
            utilization = current_status['utilization_percent']
            if utilization >= (self.buffer_critical_threshold / self.max_buffer_size) * 100:
                buffer_status.level = DiagnosticStatus.ERROR
                buffer_status.message = f"Critical buffer usage: {utilization:.1f}%"
            elif utilization >= (self.buffer_warning_threshold / self.max_buffer_size) * 100:
                buffer_status.level = DiagnosticStatus.WARN
                buffer_status.message = f"High buffer usage: {utilization:.1f}%"
            else:
                buffer_status.level = DiagnosticStatus.OK
                buffer_status.message = f"Buffer status normal: {utilization:.1f}%"
            
            # Add diagnostic key-value pairs
            buffer_status.values = [
                KeyValue(key="buffer_status", value=current_status['status'].value),
                KeyValue(key="used_slots", value=str(current_status['used_slots'])),
                KeyValue(key="available_slots", value=str(current_status['available_slots'])),
                KeyValue(key="utilization_percent", value=f"{utilization:.2f}"),
                KeyValue(key="max_buffer_size", value=str(self.max_buffer_size)),
                KeyValue(key="total_reads", value=str(stats['total_reads'])),
                KeyValue(key="failed_reads", value=str(stats['failed_reads'])),
                KeyValue(key="max_utilization", value=f"{stats['max_utilization']:.2f}"),
                KeyValue(key="avg_utilization", value=f"{stats['avg_utilization']:.2f}"),
                KeyValue(key="overflow_warnings", value=str(stats['overflow_warnings'])),
                KeyValue(key="monitoring_rate_hz", value=f"{self.monitoring_rate_hz:.1f}")
            ]
            
            # Add to diagnostic array and publish
            diagnostic_array.status.append(buffer_status)
            self.diagnostic_publisher.publish(diagnostic_array)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish buffer diagnostics: {str(e)}")
    
    def check_availability(self, required_slots: int) -> Dict[str, Any]:
        """
        Check if sufficient buffer space is available for operation.
        
        Args:
            required_slots: Number of buffer slots required
            
        Returns:
            Dictionary with availability status and recommendations
        """
        with self._status_lock:
            current_status = self.current_buffer_status.copy()
        
        available_slots = current_status['available_slots']
        
        # Check availability with safety margin
        required_with_margin = required_slots + self.min_buffer_available
        
        if available_slots < required_with_margin:
            return {
                'available': False,
                'message': f"Insufficient buffer space: need {required_with_margin} slots, only {available_slots} available",
                'recommendation': 'Wait for buffer to drain or reduce command batch size',
                'current_utilization': current_status['utilization_percent'],
                'estimated_wait_time': self._estimate_wait_time(required_with_margin - available_slots)
            }
        
        return {
            'available': True,
            'message': f"Buffer space available: {available_slots} slots free",
            'recommendation': 'Proceed with operation',
            'current_utilization': current_status['utilization_percent'],
            'optimal_batch_size': self._calculate_optimal_batch_size()
        }
    
    def get_optimization_recommendations(self) -> Dict[str, Any]:
        """
        Get buffer optimization recommendations based on current usage patterns.
        
        Returns:
            Dictionary with optimization recommendations
        """
        with self._status_lock:
            current_status = self.current_buffer_status.copy()
            avg_utilization = self.stats['avg_utilization']
            max_utilization = self.stats['max_utilization']
        
        recommendations = []
        
        # Analyze utilization patterns
        if avg_utilization > 80.0:
            recommendations.append({
                'type': 'performance',
                'priority': 'high',
                'message': 'High average buffer utilization detected',
                'action': 'Consider increasing buffer size or optimizing command frequency'
            })
        
        if max_utilization >= 95.0:
            recommendations.append({
                'type': 'reliability',
                'priority': 'critical',
                'message': 'Buffer near overflow detected',
                'action': 'Implement adaptive batch sizing to prevent overflow'
            })
        
        if len(self.utilization_history) > 10:
            # Check for utilization variance
            utilization_variance = statistics.variance(list(self.utilization_history)[-10:])
            if utilization_variance > 100.0:  # High variance
                recommendations.append({
                    'type': 'smoothness',
                    'priority': 'medium',
                    'message': 'High buffer utilization variance detected',
                    'action': 'Consider smoothing command execution rate'
                })
        
        # Monitor fail rate
        total_reads = self.stats['total_reads']
        failed_reads = self.stats['failed_reads']
        if total_reads > 0 and (failed_reads / total_reads) > 0.05:  # >5% failure rate
            recommendations.append({
                'type': 'reliability',
                'priority': 'high',
                'message': f'High buffer read failure rate: {(failed_reads/total_reads)*100:.1f}%',
                'action': 'Check communication reliability and reduce monitoring frequency'
            })
        
        return {
            'current_utilization': current_status['utilization_percent'],
            'average_utilization': avg_utilization,
            'max_utilization': max_utilization,
            'optimal_target': self.optimal_utilization_target * 100,
            'recommendations': recommendations,
            'optimal_batch_size': self._calculate_optimal_batch_size()
        }
    
    def _estimate_wait_time(self, slots_needed: int) -> float:
        """
        Estimate time to wait for buffer slots to become available.
        
        Args:
            slots_needed: Number of additional slots needed
            
        Returns:
            Estimated wait time in seconds
        """
        if len(self.buffer_history) < 2:
            return 1.0  # Default estimate
        
        # Calculate average buffer drain rate from recent history
        recent_history = list(self.buffer_history)[-10:]  # Last 10 readings
        if len(recent_history) < 2:
            return 1.0
        
        time_span = recent_history[-1]['timestamp'] - recent_history[0]['timestamp']
        if time_span <= 0:
            return 1.0
        
        # Calculate slots drained per second
        initial_used = recent_history[0]['used_slots']
        final_used = recent_history[-1]['used_slots']
        slots_drained = max(0, initial_used - final_used)
        
        if slots_drained > 0:
            drain_rate = slots_drained / time_span
            estimated_wait = slots_needed / drain_rate
            return min(max(estimated_wait, 0.1), 10.0)  # Clamp between 0.1 and 10 seconds
        
        return 2.0  # Conservative estimate if no drain detected
    
    def _calculate_optimal_batch_size(self) -> int:
        """
        Calculate optimal batch size based on current buffer status and target utilization.
        
        Returns:
            Recommended batch size for optimal performance
        """
        with self._status_lock:
            available_slots = self.current_buffer_status['available_slots']
        
        # Target utilization considering safety margin
        target_slots = int(self.max_buffer_size * self.optimal_utilization_target)
        optimal_batch = min(available_slots, target_slots)
        
        # Ensure minimum safety margin
        return max(1, optimal_batch - self.min_buffer_available)
    
    def get_status_summary(self) -> Dict[str, Any]:
        """
        Get comprehensive buffer status summary.
        
        Returns:
            Dictionary with complete buffer status information
        """
        with self._status_lock:
            current_status = self.current_buffer_status.copy()
            stats = self.stats.copy()
        
        return {
            'current_status': current_status,
            'statistics': stats,
            'configuration': {
                'max_buffer_size': self.max_buffer_size,
                'min_buffer_available': self.min_buffer_available,
                'warning_threshold': self.buffer_warning_threshold,
                'critical_threshold': self.buffer_critical_threshold,
                'monitoring_rate_hz': self.monitoring_rate_hz,
                'optimal_utilization_target': self.optimal_utilization_target
            },
            'recommendations': self.get_optimization_recommendations(),
            'history_length': len(self.buffer_history)
        }
    
    def enable_monitoring(self):
        """Enable buffer status monitoring."""
        self.monitoring_enabled = True
        self.get_logger().info("Buffer monitoring enabled")
    
    def disable_monitoring(self):
        """Disable buffer status monitoring."""
        self.monitoring_enabled = False
        self.get_logger().info("Buffer monitoring disabled")
    
    def reset_statistics(self):
        """Reset buffer performance statistics."""
        with self._status_lock:
            self.stats = {
                'total_reads': 0,
                'failed_reads': 0,
                'max_utilization': 0.0,
                'avg_utilization': 0.0,
                'overflow_warnings': 0,
                'overflow_preventions': 0
            }
            self.buffer_history.clear()
            self.utilization_history.clear()
        
        self.get_logger().info("Buffer statistics reset")