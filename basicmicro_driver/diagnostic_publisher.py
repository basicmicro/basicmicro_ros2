#!/usr/bin/env python3
"""
Basicmicro Diagnostic Publisher - Error Reporting and Diagnostics System

This module implements comprehensive error reporting and diagnostic system using ROS2 diagnostic framework.
It integrates with existing ROS2 diagnostic tools and provides comprehensive system health monitoring.

Author: Claude Code Assistant
Date: 2025-07-10
"""

import time
import threading
from typing import Dict, Any, List, Optional, Tuple, Callable
from enum import Enum
from dataclasses import dataclass, field
from collections import deque
import statistics

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.callback_groups import ReentrantCallbackGroup
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from std_msgs.msg import String, Float64, Bool
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    # Define minimal interface for testing without ROS2
    class Node:
        def __init__(self, *args, **kwargs):
            pass
        def get_logger(self):
            return type('Logger', (), {'info': print, 'warn': print, 'error': print, 'debug': print})()
        def create_timer(self, *args, **kwargs):
            return None
        def create_publisher(self, *args, **kwargs):
            return None
        def declare_parameter(self, name, default):
            return type('Parameter', (), {'value': default})()
    
    # Define mock diagnostic messages for testing
    class DiagnosticStatus:
        OK = 0
        WARN = 1
        ERROR = 2
        STALE = 3
        def __init__(self):
            self.name = ""
            self.level = 0
            self.message = ""
            self.hardware_id = ""
            self.values = []
    
    class DiagnosticArray:
        def __init__(self):
            self.header = type('Header', (), {'stamp': None})()
            self.status = []
    
    class KeyValue:
        def __init__(self, key="", value=""):
            self.key = key
            self.value = value
    
    class String:
        def __init__(self):
            self.data = ""


class DiagnosticLevel(Enum):
    """Diagnostic status levels following ROS2 diagnostic conventions"""
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3


class DiagnosticCategory(Enum):
    """Categories of diagnostic information"""
    HARDWARE = "hardware"
    MOTION_CONTROL = "motion_control"
    COMMUNICATION = "communication" 
    SAFETY = "safety"
    PERFORMANCE = "performance"
    SERVO = "servo"


@dataclass
class DiagnosticData:
    """Container for diagnostic information"""
    name: str
    level: DiagnosticLevel
    message: str
    category: DiagnosticCategory
    hardware_id: str = ""
    values: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class HardwareStatus:
    """Hardware status data from Basicmicro controller"""
    main_battery_voltage: float = 0.0
    logic_battery_voltage: float = 0.0
    motor1_current: float = 0.0
    motor2_current: float = 0.0
    temperature1: float = 0.0
    temperature2: float = 0.0
    status_flags: int = 0
    error_state: bool = False
    communication_errors: int = 0
    last_update: float = field(default_factory=time.time)


@dataclass
class MotionControlStatus:
    """Motion control status and performance data"""
    position_error_left: int = 0
    position_error_right: int = 0
    speed_error_left: int = 0
    speed_error_right: int = 0
    current_left_speed: int = 0
    current_right_speed: int = 0
    target_left_speed: int = 0
    target_right_speed: int = 0
    buffer_depth_left: int = 0
    buffer_depth_right: int = 0
    trajectory_active: bool = False
    servo_mode_active: bool = False
    emergency_stop_active: bool = False
    last_update: float = field(default_factory=time.time)


@dataclass
class CommunicationStatus:
    """Communication status and timing data"""
    total_commands_sent: int = 0
    successful_commands: int = 0
    failed_commands: int = 0
    timeout_errors: int = 0
    communication_failures: int = 0
    average_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    last_successful_communication: float = field(default_factory=time.time)
    communication_active: bool = True


@dataclass
class SafetyStatus:
    """Safety system status"""
    emergency_stop_active: bool = False
    position_limits_enabled: bool = False
    position_limit_violations: int = 0
    error_limit_violations: int = 0
    thermal_warnings: int = 0
    voltage_warnings: int = 0
    current_warnings: int = 0
    last_safety_check: float = field(default_factory=time.time)


class DiagnosticPublisher(Node if ROS2_AVAILABLE else object):
    """
    Comprehensive diagnostic publisher for Basicmicro driver
    
    Integrates with ROS2 diagnostic framework to provide comprehensive system health monitoring
    using diagnostic_msgs for standard error reporting. Provides real-time diagnostic data
    for hardware status, motion control, communication, and safety systems.
    """
    
    def __init__(self, hardware_interface=None, trajectory_monitor=None, buffer_manager=None):
        """
        Initialize diagnostic publisher
        
        Args:
            hardware_interface: Hardware interface instance for data access
            trajectory_monitor: Trajectory monitor for motion control diagnostics
            buffer_manager: Buffer manager for command buffer status
        """
        if ROS2_AVAILABLE:
            super().__init__('basicmicro_diagnostic_publisher')
        else:
            # Create a mock logger for testing without ROS2
            self._logger = type('Logger', (), {
                'info': lambda self, msg: print(f"INFO: {msg}"),
                'warn': lambda self, msg: print(f"WARN: {msg}"),
                'error': lambda self, msg: print(f"ERROR: {msg}"),
                'debug': lambda self, msg: print(f"DEBUG: {msg}")
            })()
            
        # Component references
        self.hardware_interface = hardware_interface
        self.trajectory_monitor = trajectory_monitor
        self.buffer_manager = buffer_manager
        
        # Configuration parameters
        self._load_diagnostic_parameters()
        
        # Diagnostic data storage
        self.hardware_status = HardwareStatus()
        self.motion_control_status = MotionControlStatus()
        self.communication_status = CommunicationStatus()
        self.safety_status = SafetyStatus()
        
        # Diagnostic history for trending
        self.diagnostic_history: Dict[str, deque] = {
            'hardware': deque(maxlen=self.diagnostic_history_size),
            'motion': deque(maxlen=self.diagnostic_history_size),
            'communication': deque(maxlen=self.diagnostic_history_size),
            'safety': deque(maxlen=self.diagnostic_history_size)
        }
        
        # Performance tracking
        self.latency_history = deque(maxlen=100)
        self.success_rate_history = deque(maxlen=50)
        
        # Threading and timing
        self.diagnostic_thread = None
        self.diagnostic_active = False
        self.diagnostic_lock = threading.RLock()
        
        # ROS2 interface setup
        if ROS2_AVAILABLE:
            self._setup_ros2_interface()
            
        # Diagnostic callbacks
        self.diagnostic_callbacks: Dict[DiagnosticCategory, List[Callable]] = {
            category: [] for category in DiagnosticCategory
        }
        
        self.get_logger().info("Diagnostic publisher initialized")
    
    def get_logger(self):
        """Get logger instance (compatible with ROS2 and testing)"""
        if ROS2_AVAILABLE:
            return super().get_logger()
        else:
            return self._logger
    
    def _load_diagnostic_parameters(self):
        """Load diagnostic configuration parameters"""
        if ROS2_AVAILABLE:
            # Diagnostic publishing configuration
            self.diagnostic_rate = self.declare_parameter('diagnostic_rate', 5.0).value
            self.diagnostic_timeout = self.declare_parameter('diagnostic_timeout', 2.0).value
            self.diagnostic_history_size = self.declare_parameter('diagnostic_history_size', 100).value
            
            # Hardware monitoring thresholds
            self.min_battery_voltage = self.declare_parameter('min_battery_voltage', 10.0).value
            self.max_battery_voltage = self.declare_parameter('max_battery_voltage', 16.0).value
            self.max_motor_current = self.declare_parameter('max_motor_current', 10.0).value
            self.max_temperature = self.declare_parameter('max_temperature', 70.0).value
            
            # Communication monitoring thresholds
            self.max_communication_latency = self.declare_parameter('max_communication_latency', 50.0).value
            self.min_communication_success_rate = self.declare_parameter('min_communication_success_rate', 0.95).value
            self.communication_timeout_threshold = self.declare_parameter('communication_timeout_threshold', 5.0).value
            
            # Motion control thresholds
            self.max_position_error = self.declare_parameter('max_position_error', 1000).value
            self.max_speed_error = self.declare_parameter('max_speed_error', 500).value
            
            # Safety monitoring configuration
            self.enable_safety_monitoring = self.declare_parameter('enable_safety_monitoring', True).value
            self.enable_thermal_monitoring = self.declare_parameter('enable_thermal_monitoring', True).value
            self.enable_voltage_monitoring = self.declare_parameter('enable_voltage_monitoring', True).value
            self.enable_current_monitoring = self.declare_parameter('enable_current_monitoring', True).value
        else:
            # Default values for testing
            self.diagnostic_rate = 5.0
            self.diagnostic_timeout = 2.0
            self.diagnostic_history_size = 100
            self.min_battery_voltage = 10.0
            self.max_battery_voltage = 16.0
            self.max_motor_current = 10.0
            self.max_temperature = 70.0
            self.max_communication_latency = 50.0
            self.min_communication_success_rate = 0.95
            self.communication_timeout_threshold = 5.0
            self.max_position_error = 1000
            self.max_speed_error = 500
            self.enable_safety_monitoring = True
            self.enable_thermal_monitoring = True
            self.enable_voltage_monitoring = True
            self.enable_current_monitoring = True
    
    def _setup_ros2_interface(self):
        """Setup ROS2 publishers and timers"""
        if not ROS2_AVAILABLE:
            return
            
        # Diagnostic array publisher (standard ROS2 diagnostics)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # Detailed status publishers
        self.hardware_status_publisher = self.create_publisher(
            String,
            '~/hardware_status',
            10
        )
        
        self.motion_status_publisher = self.create_publisher(
            String,
            '~/motion_status',
            10
        )
        
        self.communication_status_publisher = self.create_publisher(
            String,
            '~/communication_status',
            10
        )
        
        self.safety_status_publisher = self.create_publisher(
            String,
            '~/safety_status',
            10
        )
        
        # Diagnostic timer
        self.diagnostic_timer = self.create_timer(
            1.0 / self.diagnostic_rate,
            self._diagnostic_timer_callback
        )
    
    def start_diagnostic_monitoring(self) -> bool:
        """
        Start diagnostic monitoring system
        
        Returns:
            bool: True if started successfully, False otherwise
        """
        try:
            with self.diagnostic_lock:
                if self.diagnostic_active:
                    self.get_logger().warn("Diagnostic monitoring already active")
                    return True
                    
                self.diagnostic_active = True
                
                # Start diagnostic thread
                self.diagnostic_thread = threading.Thread(
                    target=self._diagnostic_monitoring_loop,
                    daemon=True
                )
                self.diagnostic_thread.start()
                
                self.get_logger().info("Diagnostic monitoring started")
                return True
                
        except Exception as e:
            self.get_logger().error(f"Failed to start diagnostic monitoring: {e}")
            return False
    
    def stop_diagnostic_monitoring(self) -> bool:
        """
        Stop diagnostic monitoring system
        
        Returns:
            bool: True if stopped successfully, False otherwise
        """
        try:
            with self.diagnostic_lock:
                if not self.diagnostic_active:
                    self.get_logger().warn("Diagnostic monitoring not active")
                    return True
                    
                self.diagnostic_active = False
                
                # Wait for thread to finish
                if self.diagnostic_thread and self.diagnostic_thread.is_alive():
                    self.diagnostic_thread.join(timeout=2.0)
                    
                self.get_logger().info("Diagnostic monitoring stopped")
                return True
                
        except Exception as e:
            self.get_logger().error(f"Failed to stop diagnostic monitoring: {e}")
            return False
    
    def _diagnostic_monitoring_loop(self):
        """Main diagnostic monitoring loop"""
        self.get_logger().info("Diagnostic monitoring loop started")
        
        while self.diagnostic_active:
            try:
                # Collect diagnostic data
                self._collect_hardware_diagnostics()
                self._collect_motion_control_diagnostics()
                self._collect_communication_diagnostics()
                self._collect_safety_diagnostics()
                
                # Update diagnostic history
                self._update_diagnostic_history()
                
                # Sleep for next cycle
                time.sleep(1.0 / self.diagnostic_rate)
                
            except Exception as e:
                self.get_logger().error(f"Error in diagnostic monitoring loop: {e}")
                time.sleep(0.1)
                
        self.get_logger().info("Diagnostic monitoring loop ended")
    
    def _collect_hardware_diagnostics(self):
        """Collect hardware status diagnostics from Basicmicro controller"""
        if not self.hardware_interface:
            return
            
        try:
            # Read comprehensive hardware status
            status_result = self.hardware_interface.get_status()
            if status_result and status_result[0]:  # Check success
                self.hardware_status.status_flags = status_result[1]
            elif status_result and not status_result[0]:
                self.hardware_status.communication_errors += 1
                
            # Read voltage levels
            voltage_result = self.hardware_interface.get_voltages()
            if voltage_result and voltage_result[0]:
                self.hardware_status.main_battery_voltage = voltage_result[1] / 10.0
                self.hardware_status.logic_battery_voltage = voltage_result[2] / 10.0
            elif voltage_result and not voltage_result[0]:
                self.hardware_status.communication_errors += 1
                
            # Read motor currents
            current_result = self.hardware_interface.read_currents()
            if current_result and current_result[0]:
                self.hardware_status.motor1_current = current_result[1] / 100.0
                self.hardware_status.motor2_current = current_result[2] / 100.0
            elif current_result and not current_result[0]:
                self.hardware_status.communication_errors += 1
                
            # Read temperatures
            temp_result = self.hardware_interface.get_temperatures()
            if temp_result and temp_result[0]:
                self.hardware_status.temperature1 = temp_result[1] / 10.0
                self.hardware_status.temperature2 = temp_result[2] / 10.0
            elif temp_result and not temp_result[0]:
                self.hardware_status.communication_errors += 1
                
            # Read error state
            error_result = self.hardware_interface.read_error()
            if error_result and error_result[0]:
                self.hardware_status.error_state = bool(error_result[1])
            elif error_result and not error_result[0]:
                self.hardware_status.communication_errors += 1
                
            self.hardware_status.last_update = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Error collecting hardware diagnostics: {e}")
            self.hardware_status.communication_errors += 1
    
    def _collect_motion_control_diagnostics(self):
        """Collect motion control status and performance diagnostics"""
        if not self.hardware_interface:
            return
            
        try:
            # Read position errors
            pos_errors = self.hardware_interface.get_position_errors()
            if pos_errors and pos_errors[0]:
                self.motion_control_status.position_error_left = pos_errors[1]
                self.motion_control_status.position_error_right = pos_errors[2]
                
            # Read speed errors
            speed_errors = self.hardware_interface.get_speed_errors()
            if speed_errors and speed_errors[0]:
                self.motion_control_status.speed_error_left = speed_errors[1]
                self.motion_control_status.speed_error_right = speed_errors[2]
                
            # Read current speeds
            current_speeds = self.hardware_interface.get_speeds()
            if current_speeds and current_speeds[0]:
                self.motion_control_status.current_left_speed = current_speeds[1]
                self.motion_control_status.current_right_speed = current_speeds[2]
                
            # Read buffer status
            if self.buffer_manager:
                buffer_status = self.buffer_manager.get_buffer_status()
                if buffer_status:
                    self.motion_control_status.buffer_depth_left = buffer_status.get('left_buffer', 0)
                    self.motion_control_status.buffer_depth_right = buffer_status.get('right_buffer', 0)
                    
            # Check trajectory monitor status
            if self.trajectory_monitor:
                monitoring_status = self.trajectory_monitor.get_monitoring_status()
                if monitoring_status:
                    self.motion_control_status.trajectory_active = monitoring_status.get('trajectory_active', False)
                    
            self.motion_control_status.last_update = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Error collecting motion control diagnostics: {e}")
    
    def _collect_communication_diagnostics(self):
        """Collect communication status and performance diagnostics"""
        if not self.hardware_interface:
            return
            
        try:
            # Get communication statistics
            comm_stats = self.hardware_interface.get_communication_statistics()
            if comm_stats:
                self.communication_status.total_commands_sent = comm_stats.get('total_commands', 0)
                self.communication_status.successful_commands = comm_stats.get('successful_commands', 0)
                self.communication_status.failed_commands = comm_stats.get('failed_commands', 0)
                self.communication_status.timeout_errors = comm_stats.get('timeout_errors', 0)
                
                # Calculate success rate
                total = self.communication_status.total_commands_sent
                successful = self.communication_status.successful_commands
                if total > 0:
                    success_rate = successful / total
                    self.success_rate_history.append(success_rate)
                    
            # Get latency statistics
            latency_stats = self.hardware_interface.get_latency_statistics()
            if latency_stats:
                self.communication_status.average_latency_ms = latency_stats.get('average_ms', 0.0)
                self.communication_status.max_latency_ms = latency_stats.get('max_ms', 0.0)
                
                # Update latency history
                self.latency_history.append(self.communication_status.average_latency_ms)
                
            # Check if communication is active
            time_since_last = time.time() - self.communication_status.last_successful_communication
            self.communication_status.communication_active = time_since_last < self.communication_timeout_threshold
            
        except Exception as e:
            self.get_logger().error(f"Error collecting communication diagnostics: {e}")
            self.communication_status.communication_failures += 1
    
    def _collect_safety_diagnostics(self):
        """Collect safety system status and alerts"""
        if not self.enable_safety_monitoring:
            return
            
        try:
            # Check emergency stop status
            if self.hardware_interface:
                estop_status = self.hardware_interface.is_emergency_stop_active()
                self.safety_status.emergency_stop_active = estop_status
                
            # Check position limits
            if hasattr(self.hardware_interface, 'position_limits_enabled'):
                self.safety_status.position_limits_enabled = self.hardware_interface.position_limits_enabled
                
            # Analyze hardware status for safety concerns
            self._analyze_hardware_safety()
            
            self.safety_status.last_safety_check = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Error collecting safety diagnostics: {e}")
    
    def _analyze_hardware_safety(self):
        """Analyze hardware status for safety concerns"""
        # Check voltage levels
        if self.enable_voltage_monitoring:
            if (self.hardware_status.main_battery_voltage < self.min_battery_voltage or
                self.hardware_status.main_battery_voltage > self.max_battery_voltage):
                self.safety_status.voltage_warnings += 1
                
        # Check temperature levels
        if self.enable_thermal_monitoring:
            if (self.hardware_status.temperature1 > self.max_temperature or
                self.hardware_status.temperature2 > self.max_temperature):
                self.safety_status.thermal_warnings += 1
                
        # Check current levels
        if self.enable_current_monitoring:
            if (self.hardware_status.motor1_current > self.max_motor_current or
                self.hardware_status.motor2_current > self.max_motor_current):
                self.safety_status.current_warnings += 1
                
        # Check position errors
        if (abs(self.motion_control_status.position_error_left) > self.max_position_error or
            abs(self.motion_control_status.position_error_right) > self.max_position_error):
            self.safety_status.position_limit_violations += 1
    
    def _update_diagnostic_history(self):
        """Update diagnostic history for trending analysis"""
        current_time = time.time()
        
        # Hardware diagnostics
        hardware_diag = {
            'timestamp': current_time,
            'battery_voltage': self.hardware_status.main_battery_voltage,
            'motor_currents': [self.hardware_status.motor1_current, self.hardware_status.motor2_current],
            'temperatures': [self.hardware_status.temperature1, self.hardware_status.temperature2],
            'error_state': self.hardware_status.error_state
        }
        self.diagnostic_history['hardware'].append(hardware_diag)
        
        # Motion control diagnostics
        motion_diag = {
            'timestamp': current_time,
            'position_errors': [self.motion_control_status.position_error_left, 
                              self.motion_control_status.position_error_right],
            'speed_errors': [self.motion_control_status.speed_error_left,
                           self.motion_control_status.speed_error_right],
            'trajectory_active': self.motion_control_status.trajectory_active
        }
        self.diagnostic_history['motion'].append(motion_diag)
        
        # Communication diagnostics
        comm_diag = {
            'timestamp': current_time,
            'success_rate': (self.communication_status.successful_commands / 
                           max(1, self.communication_status.total_commands_sent)),
            'average_latency': self.communication_status.average_latency_ms,
            'communication_active': self.communication_status.communication_active
        }
        self.diagnostic_history['communication'].append(comm_diag)
        
        # Safety diagnostics
        safety_diag = {
            'timestamp': current_time,
            'emergency_stop': self.safety_status.emergency_stop_active,
            'total_warnings': (self.safety_status.thermal_warnings + 
                             self.safety_status.voltage_warnings + 
                             self.safety_status.current_warnings)
        }
        self.diagnostic_history['safety'].append(safety_diag)
    
    def _diagnostic_timer_callback(self):
        """ROS2 timer callback for publishing diagnostics"""
        if not ROS2_AVAILABLE:
            return
            
        try:
            # Create diagnostic array
            diagnostic_array = DiagnosticArray()
            diagnostic_array.header.stamp = self.get_clock().now().to_msg()
            
            # Generate diagnostic messages
            diagnostics = []
            diagnostics.extend(self._generate_hardware_diagnostics())
            diagnostics.extend(self._generate_motion_control_diagnostics())
            diagnostics.extend(self._generate_communication_diagnostics())
            diagnostics.extend(self._generate_safety_diagnostics())
            
            diagnostic_array.status = diagnostics
            
            # Publish diagnostic array
            self.diagnostic_publisher.publish(diagnostic_array)
            
            # Publish detailed status messages
            self._publish_detailed_status()
            
        except Exception as e:
            self.get_logger().error(f"Error in diagnostic timer callback: {e}")
    
    def _generate_hardware_diagnostics(self) -> List[DiagnosticStatus]:
        """Generate hardware diagnostic status messages"""
        diagnostics = []
        
        # Battery voltage diagnostic
        voltage_diag = DiagnosticStatus()
        voltage_diag.name = "basicmicro/battery_voltage"
        voltage_diag.hardware_id = "basicmicro_controller"
        
        if (self.hardware_status.main_battery_voltage < self.min_battery_voltage or
            self.hardware_status.main_battery_voltage > self.max_battery_voltage):
            voltage_diag.level = DiagnosticStatus.ERROR
            voltage_diag.message = f"Battery voltage out of range: {self.hardware_status.main_battery_voltage:.1f}V"
        elif self.hardware_status.main_battery_voltage < self.min_battery_voltage + 1.0:
            voltage_diag.level = DiagnosticStatus.WARN
            voltage_diag.message = f"Battery voltage low: {self.hardware_status.main_battery_voltage:.1f}V"
        else:
            voltage_diag.level = DiagnosticStatus.OK
            voltage_diag.message = f"Battery voltage normal: {self.hardware_status.main_battery_voltage:.1f}V"
            
        voltage_diag.values = [
            KeyValue(key="main_battery_voltage", value=str(self.hardware_status.main_battery_voltage)),
            KeyValue(key="logic_battery_voltage", value=str(self.hardware_status.logic_battery_voltage)),
            KeyValue(key="min_voltage_threshold", value=str(self.min_battery_voltage)),
            KeyValue(key="max_voltage_threshold", value=str(self.max_battery_voltage))
        ]
        diagnostics.append(voltage_diag)
        
        # Motor current diagnostic
        current_diag = DiagnosticStatus()
        current_diag.name = "basicmicro/motor_currents"
        current_diag.hardware_id = "basicmicro_controller"
        
        max_current = max(self.hardware_status.motor1_current, self.hardware_status.motor2_current)
        if max_current > self.max_motor_current:
            current_diag.level = DiagnosticStatus.ERROR
            current_diag.message = f"Motor current too high: {max_current:.1f}A"
        elif max_current > self.max_motor_current * 0.8:
            current_diag.level = DiagnosticStatus.WARN
            current_diag.message = f"Motor current elevated: {max_current:.1f}A"
        else:
            current_diag.level = DiagnosticStatus.OK
            current_diag.message = f"Motor currents normal: M1={self.hardware_status.motor1_current:.1f}A, M2={self.hardware_status.motor2_current:.1f}A"
            
        current_diag.values = [
            KeyValue(key="motor1_current", value=str(self.hardware_status.motor1_current)),
            KeyValue(key="motor2_current", value=str(self.hardware_status.motor2_current)),
            KeyValue(key="max_current_threshold", value=str(self.max_motor_current))
        ]
        diagnostics.append(current_diag)
        
        # Temperature diagnostic
        temp_diag = DiagnosticStatus()
        temp_diag.name = "basicmicro/temperatures"
        temp_diag.hardware_id = "basicmicro_controller"
        
        max_temp = max(self.hardware_status.temperature1, self.hardware_status.temperature2)
        if max_temp > self.max_temperature:
            temp_diag.level = DiagnosticStatus.ERROR
            temp_diag.message = f"Temperature too high: {max_temp:.1f}°C"
        elif max_temp > self.max_temperature * 0.8:
            temp_diag.level = DiagnosticStatus.WARN
            temp_diag.message = f"Temperature elevated: {max_temp:.1f}°C"
        else:
            temp_diag.level = DiagnosticStatus.OK
            temp_diag.message = f"Temperatures normal: T1={self.hardware_status.temperature1:.1f}°C, T2={self.hardware_status.temperature2:.1f}°C"
            
        temp_diag.values = [
            KeyValue(key="temperature1", value=str(self.hardware_status.temperature1)),
            KeyValue(key="temperature2", value=str(self.hardware_status.temperature2)),
            KeyValue(key="max_temperature_threshold", value=str(self.max_temperature))
        ]
        diagnostics.append(temp_diag)
        
        return diagnostics
    
    def _generate_motion_control_diagnostics(self) -> List[DiagnosticStatus]:
        """Generate motion control diagnostic status messages"""
        diagnostics = []
        
        # Position error diagnostic
        pos_error_diag = DiagnosticStatus()
        pos_error_diag.name = "basicmicro/position_errors"
        pos_error_diag.hardware_id = "basicmicro_controller"
        
        max_pos_error = max(abs(self.motion_control_status.position_error_left),
                           abs(self.motion_control_status.position_error_right))
        
        if max_pos_error > self.max_position_error:
            pos_error_diag.level = DiagnosticStatus.ERROR
            pos_error_diag.message = f"Position error too high: {max_pos_error} counts"
        elif max_pos_error > self.max_position_error * 0.5:
            pos_error_diag.level = DiagnosticStatus.WARN
            pos_error_diag.message = f"Position error elevated: {max_pos_error} counts"
        else:
            pos_error_diag.level = DiagnosticStatus.OK
            pos_error_diag.message = f"Position errors normal: L={self.motion_control_status.position_error_left}, R={self.motion_control_status.position_error_right}"
            
        pos_error_diag.values = [
            KeyValue(key="position_error_left", value=str(self.motion_control_status.position_error_left)),
            KeyValue(key="position_error_right", value=str(self.motion_control_status.position_error_right)),
            KeyValue(key="max_position_error_threshold", value=str(self.max_position_error))
        ]
        diagnostics.append(pos_error_diag)
        
        # Speed error diagnostic
        speed_error_diag = DiagnosticStatus()
        speed_error_diag.name = "basicmicro/speed_errors"
        speed_error_diag.hardware_id = "basicmicro_controller"
        
        max_speed_error = max(abs(self.motion_control_status.speed_error_left),
                             abs(self.motion_control_status.speed_error_right))
        
        if max_speed_error > self.max_speed_error:
            speed_error_diag.level = DiagnosticStatus.ERROR
            speed_error_diag.message = f"Speed error too high: {max_speed_error} counts/sec"
        elif max_speed_error > self.max_speed_error * 0.5:
            speed_error_diag.level = DiagnosticStatus.WARN
            speed_error_diag.message = f"Speed error elevated: {max_speed_error} counts/sec"
        else:
            speed_error_diag.level = DiagnosticStatus.OK
            speed_error_diag.message = f"Speed errors normal: L={self.motion_control_status.speed_error_left}, R={self.motion_control_status.speed_error_right}"
            
        speed_error_diag.values = [
            KeyValue(key="speed_error_left", value=str(self.motion_control_status.speed_error_left)),
            KeyValue(key="speed_error_right", value=str(self.motion_control_status.speed_error_right)),
            KeyValue(key="max_speed_error_threshold", value=str(self.max_speed_error))
        ]
        diagnostics.append(speed_error_diag)
        
        return diagnostics
    
    def _generate_communication_diagnostics(self) -> List[DiagnosticStatus]:
        """Generate communication diagnostic status messages"""
        diagnostics = []
        
        # Communication performance diagnostic
        comm_diag = DiagnosticStatus()
        comm_diag.name = "basicmicro/communication"
        comm_diag.hardware_id = "basicmicro_controller"
        
        # Calculate success rate
        total_commands = self.communication_status.total_commands_sent
        success_rate = 0.0
        if total_commands > 0:
            success_rate = self.communication_status.successful_commands / total_commands
            
        # Determine diagnostic level
        if not self.communication_status.communication_active:
            comm_diag.level = DiagnosticStatus.ERROR
            comm_diag.message = "Communication timeout - no recent successful commands"
        elif success_rate < self.min_communication_success_rate:
            comm_diag.level = DiagnosticStatus.ERROR
            comm_diag.message = f"Communication success rate too low: {success_rate:.2%}"
        elif self.communication_status.average_latency_ms > self.max_communication_latency:
            comm_diag.level = DiagnosticStatus.WARN
            comm_diag.message = f"Communication latency high: {self.communication_status.average_latency_ms:.1f}ms"
        else:
            comm_diag.level = DiagnosticStatus.OK
            comm_diag.message = f"Communication normal: {success_rate:.2%} success, {self.communication_status.average_latency_ms:.1f}ms latency"
            
        comm_diag.values = [
            KeyValue(key="total_commands", value=str(self.communication_status.total_commands_sent)),
            KeyValue(key="successful_commands", value=str(self.communication_status.successful_commands)),
            KeyValue(key="failed_commands", value=str(self.communication_status.failed_commands)),
            KeyValue(key="success_rate", value=f"{success_rate:.3f}"),
            KeyValue(key="average_latency_ms", value=str(self.communication_status.average_latency_ms)),
            KeyValue(key="max_latency_ms", value=str(self.communication_status.max_latency_ms)),
            KeyValue(key="timeout_errors", value=str(self.communication_status.timeout_errors))
        ]
        diagnostics.append(comm_diag)
        
        return diagnostics
    
    def _generate_safety_diagnostics(self) -> List[DiagnosticStatus]:
        """Generate safety diagnostic status messages"""
        diagnostics = []
        
        # Safety system diagnostic
        safety_diag = DiagnosticStatus()
        safety_diag.name = "basicmicro/safety_system"
        safety_diag.hardware_id = "basicmicro_controller"
        
        total_warnings = (self.safety_status.thermal_warnings + 
                         self.safety_status.voltage_warnings + 
                         self.safety_status.current_warnings)
        
        if self.safety_status.emergency_stop_active:
            safety_diag.level = DiagnosticStatus.ERROR
            safety_diag.message = "Emergency stop active"
        elif total_warnings > 10:
            safety_diag.level = DiagnosticStatus.ERROR
            safety_diag.message = f"Multiple safety warnings: {total_warnings} total"
        elif total_warnings > 0:
            safety_diag.level = DiagnosticStatus.WARN
            safety_diag.message = f"Safety warnings present: {total_warnings} total"
        else:
            safety_diag.level = DiagnosticStatus.OK
            safety_diag.message = "Safety system normal"
            
        safety_diag.values = [
            KeyValue(key="emergency_stop_active", value=str(self.safety_status.emergency_stop_active)),
            KeyValue(key="position_limits_enabled", value=str(self.safety_status.position_limits_enabled)),
            KeyValue(key="thermal_warnings", value=str(self.safety_status.thermal_warnings)),
            KeyValue(key="voltage_warnings", value=str(self.safety_status.voltage_warnings)),
            KeyValue(key="current_warnings", value=str(self.safety_status.current_warnings)),
            KeyValue(key="total_warnings", value=str(total_warnings))
        ]
        diagnostics.append(safety_diag)
        
        return diagnostics
    
    def _publish_detailed_status(self):
        """Publish detailed status messages on individual topics"""
        if not ROS2_AVAILABLE:
            return
            
        try:
            # Hardware status
            hardware_msg = String()
            hardware_msg.data = f"Battery: {self.hardware_status.main_battery_voltage:.1f}V, " \
                              f"Currents: M1={self.hardware_status.motor1_current:.1f}A M2={self.hardware_status.motor2_current:.1f}A, " \
                              f"Temps: T1={self.hardware_status.temperature1:.1f}°C T2={self.hardware_status.temperature2:.1f}°C"
            self.hardware_status_publisher.publish(hardware_msg)
            
            # Motion status
            motion_msg = String()
            motion_msg.data = f"Pos Errors: L={self.motion_control_status.position_error_left} R={self.motion_control_status.position_error_right}, " \
                            f"Speed Errors: L={self.motion_control_status.speed_error_left} R={self.motion_control_status.speed_error_right}, " \
                            f"Trajectory: {'Active' if self.motion_control_status.trajectory_active else 'Inactive'}"
            self.motion_status_publisher.publish(motion_msg)
            
            # Communication status
            comm_msg = String()
            success_rate = 0.0
            if self.communication_status.total_commands_sent > 0:
                success_rate = self.communication_status.successful_commands / self.communication_status.total_commands_sent
            comm_msg.data = f"Success Rate: {success_rate:.2%}, " \
                          f"Latency: {self.communication_status.average_latency_ms:.1f}ms, " \
                          f"Active: {'Yes' if self.communication_status.communication_active else 'No'}"
            self.communication_status_publisher.publish(comm_msg)
            
            # Safety status
            safety_msg = String()
            total_warnings = (self.safety_status.thermal_warnings + 
                            self.safety_status.voltage_warnings + 
                            self.safety_status.current_warnings)
            safety_msg.data = f"E-Stop: {'Active' if self.safety_status.emergency_stop_active else 'Inactive'}, " \
                            f"Warnings: {total_warnings}, " \
                            f"Limits: {'Enabled' if self.safety_status.position_limits_enabled else 'Disabled'}"
            self.safety_status_publisher.publish(safety_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing detailed status: {e}")
    
    def register_diagnostic_callback(self, category: DiagnosticCategory, callback: Callable):
        """Register callback for specific diagnostic category updates"""
        self.diagnostic_callbacks[category].append(callback)
    
    def get_diagnostic_summary(self) -> Dict[str, Any]:
        """Get comprehensive diagnostic summary"""
        summary = {
            'hardware': {
                'battery_voltage': self.hardware_status.main_battery_voltage,
                'motor_currents': [self.hardware_status.motor1_current, self.hardware_status.motor2_current],
                'temperatures': [self.hardware_status.temperature1, self.hardware_status.temperature2],
                'error_state': self.hardware_status.error_state,
                'last_update': self.hardware_status.last_update
            },
            'motion_control': {
                'position_errors': [self.motion_control_status.position_error_left, 
                                  self.motion_control_status.position_error_right],
                'speed_errors': [self.motion_control_status.speed_error_left,
                               self.motion_control_status.speed_error_right],
                'trajectory_active': self.motion_control_status.trajectory_active,
                'servo_mode_active': self.motion_control_status.servo_mode_active,
                'emergency_stop_active': self.motion_control_status.emergency_stop_active,
                'last_update': self.motion_control_status.last_update
            },
            'communication': {
                'success_rate': (self.communication_status.successful_commands / 
                               max(1, self.communication_status.total_commands_sent)),
                'average_latency_ms': self.communication_status.average_latency_ms,
                'communication_active': self.communication_status.communication_active,
                'total_commands': self.communication_status.total_commands_sent,
                'failed_commands': self.communication_status.failed_commands
            },
            'safety': {
                'emergency_stop_active': self.safety_status.emergency_stop_active,
                'position_limits_enabled': self.safety_status.position_limits_enabled,
                'total_warnings': (self.safety_status.thermal_warnings + 
                                 self.safety_status.voltage_warnings + 
                                 self.safety_status.current_warnings),
                'last_safety_check': self.safety_status.last_safety_check
            },
            'monitoring': {
                'diagnostic_active': self.diagnostic_active,
                'diagnostic_rate': self.diagnostic_rate,
                'history_size': len(self.diagnostic_history['hardware'])
            }
        }
        
        return summary
    
    def get_diagnostic_trends(self, category: str, duration_minutes: float = 5.0) -> Dict[str, Any]:
        """Get diagnostic trends for specified category and time duration"""
        if category not in self.diagnostic_history:
            return {}
            
        current_time = time.time()
        cutoff_time = current_time - (duration_minutes * 60)
        
        # Filter history by time
        recent_data = [data for data in self.diagnostic_history[category] 
                      if data['timestamp'] >= cutoff_time]
        
        if not recent_data:
            return {}
            
        trends = {
            'sample_count': len(recent_data),
            'time_range': duration_minutes,
            'data_points': recent_data
        }
        
        # Calculate category-specific trends
        if category == 'hardware':
            voltages = [data['battery_voltage'] for data in recent_data]
            trends['voltage_trend'] = {
                'mean': statistics.mean(voltages) if voltages else 0,
                'min': min(voltages) if voltages else 0,
                'max': max(voltages) if voltages else 0
            }
            
        elif category == 'communication':
            success_rates = [data['success_rate'] for data in recent_data]
            latencies = [data['average_latency'] for data in recent_data]
            trends['performance_trend'] = {
                'avg_success_rate': statistics.mean(success_rates) if success_rates else 0,
                'avg_latency': statistics.mean(latencies) if latencies else 0
            }
            
        return trends
    
    def reset_diagnostic_counters(self):
        """Reset diagnostic counters and statistics"""
        # Reset hardware status counters
        self.hardware_status.communication_errors = 0
        
        # Reset motion control counters - preserve current values
        
        # Reset communication status counters
        self.communication_status.total_commands_sent = 0
        self.communication_status.successful_commands = 0
        self.communication_status.failed_commands = 0
        self.communication_status.timeout_errors = 0
        self.communication_status.communication_failures = 0
        
        # Reset safety status counters
        self.safety_status.position_limit_violations = 0
        self.safety_status.error_limit_violations = 0
        self.safety_status.thermal_warnings = 0
        self.safety_status.voltage_warnings = 0
        self.safety_status.current_warnings = 0
        
        # Clear history
        for category in self.diagnostic_history:
            self.diagnostic_history[category].clear()
            
        self.get_logger().info("Diagnostic counters reset")


# Export main classes
__all__ = [
    'DiagnosticPublisher',
    'DiagnosticLevel',
    'DiagnosticCategory',
    'DiagnosticData',
    'HardwareStatus',
    'MotionControlStatus',
    'CommunicationStatus',
    'SafetyStatus'
]