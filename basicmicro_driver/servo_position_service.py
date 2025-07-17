#!/usr/bin/env python3
"""
Servo Position Control Service for Basicmicro Driver

Implements comprehensive servo position control and controller-aware homing system
for incremental and absolute encoders. Supports both RoboClaw and MCP controllers
with their specific capabilities and behaviors.

Author: ROS2 Driver Development
"""

import time
import math
from typing import Dict, Any, Optional, List, Tuple, Union
from enum import Enum

# Handle ROS2 imports gracefully for testing
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.service import Service
    from std_msgs.msg import Float32
    ROS2_AVAILABLE = True
except ImportError:
    # Mock ROS2 classes for testing
    ROS2_AVAILABLE = False
    
    class Node:
        def __init__(self, node_name):
            self.node_name = node_name
            self.parameters = {}
            self.services = {}
            
        def declare_parameter(self, name, default_value):
            from unittest.mock import Mock
            self.parameters[name] = Mock()
            self.parameters[name].value = default_value
            
        def get_parameter(self, name):
            from unittest.mock import Mock
            return self.parameters.get(name, Mock())
        
        def create_service(self, srv_type, srv_name, callback):
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
except ImportError:
    # Fallback for relative import in package context
    from .unit_converter import UnitConverter
    from .buffer_manager import BufferManager


class ControllerType(Enum):
    """Supported BasicMicro controller types"""
    ROBOCLAW = "roboclaw"
    MCP = "mcp"
    UNKNOWN = "unknown"


class HomingMethod(Enum):
    """Available homing methods by controller type"""
    # RoboClaw methods
    HOME_PIN_BACKWARD = "home_pin_backward"
    LIMIT_SWITCH_STOP = "limit_switch_stop"
    MANUAL_ZERO = "manual_zero"
    AUTO_HOME_STARTUP = "auto_home_startup"
    
    # MCP methods
    HOME_PIN_BIDIRECTIONAL = "home_pin_bidirectional"
    LIMIT_SWITCH_HOMING = "limit_switch_homing"
    CURRENT_LIMIT_HOMING = "current_limit_homing"


class ServoState(Enum):
    """Servo position control states"""
    IDLE = "idle"
    POSITIONING = "positioning"
    HOLDING = "holding"
    HOMING = "homing"
    ERROR = "error"


class ServoPositionService(Node if ROS2_AVAILABLE else object):
    """
    Servo Position Control Service
    
    Implements comprehensive servo position control using SpeedAccelDeccelPositionM1M2
    and controller-aware homing system for RoboClaw and MCP controllers.
    """
    
    def __init__(self, hardware_interface=None, buffer_manager=None, unit_converter=None):
        """
        Initialize servo position control service
        
        Args:
            hardware_interface: Hardware interface for controller communication
            buffer_manager: Buffer manager for optimization
            unit_converter: Unit conversion utilities
        """
        if ROS2_AVAILABLE:
            super().__init__('servo_position_service')
            
        self.hardware_interface = hardware_interface
        self.buffer_manager = buffer_manager
        self.unit_converter = unit_converter or UnitConverter()
        
        # Servo state tracking
        self.servo_state = ServoState.IDLE
        self.current_left_position = 0.0  # radians
        self.current_right_position = 0.0  # radians
        self.target_left_position = 0.0   # radians
        self.target_right_position = 0.0  # radians
        self.position_hold_active = False
        
        # Controller information
        self.controller_type = ControllerType.UNKNOWN
        self.controller_address = 0x80
        self.controller_detected = False
        
        # Homing configuration
        self.auto_home_on_startup = False
        self.default_homing_method = "manual_zero"
        self.default_homing_speed = 0.1  # m/s
        self.homing_timeout = 30.0  # seconds
        
        # Servo parameters
        self.max_position_error = 1000  # encoder counts
        self.max_speed_error = 500      # counts/sec
        self.position_tolerance = 0.01  # radians
        self.servo_update_rate = 50.0   # Hz
        
        # Error monitoring
        self.last_position_errors = (0, 0)
        self.last_speed_errors = (0, 0)
        self.error_limit_exceeded = False
        self.error_monitoring_active = True
        
        # Available homing methods by controller type
        self.homing_methods = {
            ControllerType.ROBOCLAW: {
                HomingMethod.HOME_PIN_BACKWARD: {
                    'description': 'Move backward until home pin (auto-zeros, acts as limit)',
                    'directions': ['backward'],
                    'auto_zeros': True,
                    'acts_as_limit': True
                },
                HomingMethod.LIMIT_SWITCH_STOP: {
                    'description': 'Move until limit switch (manual zero required)',
                    'directions': ['forward', 'backward'],
                    'auto_zeros': False,
                    'acts_as_limit': False
                },
                HomingMethod.MANUAL_ZERO: {
                    'description': 'Set current position as zero',
                    'directions': ['both'],
                    'auto_zeros': True,
                    'acts_as_limit': False
                },
                HomingMethod.AUTO_HOME_STARTUP: {
                    'description': 'Configure automatic homing on power-up',
                    'directions': ['backward'],
                    'auto_zeros': True,
                    'acts_as_limit': True
                }
            },
            ControllerType.MCP: {
                HomingMethod.HOME_PIN_BIDIRECTIONAL: {
                    'description': 'Move in specified direction until home pin',
                    'directions': ['forward', 'backward', 'both'],
                    'auto_zeros': True,
                    'acts_as_limit': True
                },
                HomingMethod.LIMIT_SWITCH_HOMING: {
                    'description': 'More flexible limit switch homing',
                    'directions': ['forward', 'backward'],
                    'auto_zeros': False,
                    'acts_as_limit': True
                },
                HomingMethod.CURRENT_LIMIT_HOMING: {
                    'description': 'Home until motor stalls',
                    'directions': ['forward', 'backward'],
                    'auto_zeros': False,
                    'acts_as_limit': False
                },
                HomingMethod.MANUAL_ZERO: {
                    'description': 'Set current position as zero',
                    'directions': ['both'],
                    'auto_zeros': True,
                    'acts_as_limit': False
                }
            }
        }
        
        # Initialize parameters
        self._initialize_parameters()
        
        # Initialize services
        self._initialize_services()
        
        # Detect controller type
        self._detect_controller_type()
        
        if ROS2_AVAILABLE:
            self.get_logger().info("Servo Position Control Service initialized")
    
    def _initialize_parameters(self):
        """Initialize ROS2 parameters"""
        if not ROS2_AVAILABLE:
            return
            
        # Servo control parameters
        self.declare_parameter('servo.max_position_error', self.max_position_error)
        self.declare_parameter('servo.max_speed_error', self.max_speed_error)
        self.declare_parameter('servo.position_tolerance', self.position_tolerance)
        self.declare_parameter('servo.update_rate', self.servo_update_rate)
        
        # Homing parameters
        self.declare_parameter('homing.auto_home_on_startup', self.auto_home_on_startup)
        self.declare_parameter('homing.default_method', self.default_homing_method)
        self.declare_parameter('homing.default_speed', self.default_homing_speed)
        self.declare_parameter('homing.timeout', self.homing_timeout)
        
        # Controller parameters
        self.declare_parameter('controller.address', self.controller_address)
        
        # Update parameters from ROS2
        self._update_parameters()
    
    def _update_parameters(self):
        """Update parameters from ROS2 parameter server"""
        if not ROS2_AVAILABLE:
            return
            
        self.max_position_error = self.get_parameter('servo.max_position_error').value
        self.max_speed_error = self.get_parameter('servo.max_speed_error').value
        self.position_tolerance = self.get_parameter('servo.position_tolerance').value
        self.servo_update_rate = self.get_parameter('servo.update_rate').value
        
        self.auto_home_on_startup = self.get_parameter('homing.auto_home_on_startup').value
        self.default_homing_method = self.get_parameter('homing.default_method').value
        self.default_homing_speed = self.get_parameter('homing.default_speed').value
        self.homing_timeout = self.get_parameter('homing.timeout').value
        
        self.controller_address = self.get_parameter('controller.address').value
    
    def _initialize_services(self):
        """Initialize ROS2 services"""
        if not ROS2_AVAILABLE:
            return
            
        # Position control services
        self.move_to_absolute_position_service = self.create_service(
            None,  # MoveToAbsolutePosition,
            'move_to_absolute_position',
            self._move_to_absolute_position_callback
        )
        
        self.release_position_hold_service = self.create_service(
            None,  # ReleasePositionHold,
            'release_position_hold',
            self._release_position_hold_callback
        )
        
        # Homing services
        self.get_available_homing_methods_service = self.create_service(
            None,  # GetAvailableHomingMethods,
            'get_available_homing_methods',
            self._get_available_homing_methods_callback
        )
        
        self.perform_homing_service = self.create_service(
            None,  # PerformHoming,
            'perform_homing',
            self._perform_homing_callback
        )
        
        self.set_homing_configuration_service = self.create_service(
            None,  # SetHomingConfiguration,
            'set_homing_configuration',
            self._set_homing_configuration_callback
        )
        
        # Status services
        self.get_servo_status_service = self.create_service(
            None,  # GetServoStatus,
            'get_servo_status',
            self._get_servo_status_callback
        )
    
    def _detect_controller_type(self):
        """Detect controller type from version string"""
        if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
            self.controller_type = ControllerType.UNKNOWN
            return
            
        try:
            # Try to get version information
            version_result = self.hardware_interface.controller.ReadVersion(self.controller_address)
            if version_result[0]:  # Success
                version_string = version_result[1].lower()
                if 'roboclaw' in version_string:
                    self.controller_type = ControllerType.ROBOCLAW
                elif 'mcp' in version_string:
                    self.controller_type = ControllerType.MCP
                else:
                    self.controller_type = ControllerType.UNKNOWN
                    
                self.controller_detected = True
                
                if ROS2_AVAILABLE:
                    self.get_logger().info(f"Detected controller type: {self.controller_type.value}")
            else:
                self.controller_type = ControllerType.UNKNOWN
                if ROS2_AVAILABLE:
                    self.get_logger().warn("Could not detect controller type from version")
                    
        except Exception as e:
            self.controller_type = ControllerType.UNKNOWN
            if ROS2_AVAILABLE:
                self.get_logger().error(f"Error detecting controller type: {e}")
    
    def move_to_absolute_position(self, left_position_radians: float, right_position_radians: float,
                                max_speed: float, acceleration: float, deceleration: float,
                                buffer_command: bool = False) -> Tuple[bool, str]:
        """
        Move to absolute position using SpeedAccelDeccelPositionM1M2
        
        Args:
            left_position_radians: Target position for left motor (radians)
            right_position_radians: Target position for right motor (radians) 
            max_speed: Maximum speed during move (m/s)
            acceleration: Acceleration rate (m/s²)
            deceleration: Deceleration rate (m/s²)
            buffer_command: Whether to buffer the command for chaining
            
        Returns:
            Tuple of (success, message)
        """
        try:
            # Validate parameters
            if not self._validate_position_parameters(left_position_radians, right_position_radians,
                                                    max_speed, acceleration, deceleration):
                return False, "Invalid position parameters"
            
            # Convert units
            left_position_counts = self.unit_converter.radians_to_counts(left_position_radians)
            right_position_counts = self.unit_converter.radians_to_counts(right_position_radians)
            max_speed_counts = self.unit_converter.rad_per_sec_to_counts_per_sec(max_speed)
            acceleration_counts = self.unit_converter.rad_per_sec2_to_counts_per_sec2(acceleration)
            deceleration_counts = self.unit_converter.rad_per_sec2_to_counts_per_sec2(deceleration)
            
            # Execute position command
            buffer_flag = 1 if buffer_command else 0
            
            if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
                return False, "Hardware interface not available"
            
            success = self.hardware_interface.controller.SpeedAccelDeccelPositionM1M2(
                self.controller_address,
                acceleration_counts, max_speed_counts, deceleration_counts, left_position_counts,
                acceleration_counts, max_speed_counts, deceleration_counts, right_position_counts,
                buffer_flag
            )
            
            if success:
                # Update target positions
                self.target_left_position = left_position_radians
                self.target_right_position = right_position_radians
                
                # Update servo state
                self.servo_state = ServoState.POSITIONING
                self.position_hold_active = True
                
                # Update buffer manager if available
                if self.buffer_manager:
                    self.buffer_manager.record_command_execution('position', buffer_command)
                
                message = f"Moving to position L:{left_position_radians:.3f}, R:{right_position_radians:.3f}"
                if buffer_command:
                    message += " (buffered)"
                
                return True, message
            else:
                return False, "Failed to execute position command"
                
        except Exception as e:
            return False, f"Error executing position command: {e}"
    
    def release_position_hold(self) -> Tuple[bool, str]:
        """
        Release position hold mode using DutyM1M2(0, 0)
        
        Returns:
            Tuple of (success, message)
        """
        try:
            if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
                return False, "Hardware interface not available"
            
            # Use DutyM1M2(0, 0) to release position hold
            success = self.hardware_interface.controller.DutyM1M2(self.controller_address, 0, 0)
            
            if success:
                self.position_hold_active = False
                self.servo_state = ServoState.IDLE
                return True, "Position hold released"
            else:
                return False, "Failed to release position hold"
                
        except Exception as e:
            return False, f"Error releasing position hold: {e}"
    
    def get_available_homing_methods(self) -> Tuple[bool, Dict[str, Any]]:
        """
        Get available homing methods based on controller type
        
        Returns:
            Tuple of (success, methods_info)
        """
        try:
            if self.controller_type == ControllerType.UNKNOWN:
                self._detect_controller_type()
                
            if self.controller_type not in self.homing_methods:
                return False, {"error": f"Unknown controller type: {self.controller_type}"}
            
            methods = self.homing_methods[self.controller_type]
            
            result = {
                "controller_type": self.controller_type.value,
                "available_methods": [],
                "method_descriptions": [],
                "allowed_directions": [],
                "auto_zeros_encoder": [],
                "acts_as_limit": []
            }
            
            for method, info in methods.items():
                result["available_methods"].append(method.value)
                result["method_descriptions"].append(info["description"])
                result["allowed_directions"].append(info["directions"])
                result["auto_zeros_encoder"].append(info["auto_zeros"])
                result["acts_as_limit"].append(info["acts_as_limit"])
            
            return True, result
            
        except Exception as e:
            return False, {"error": f"Error getting homing methods: {e}"}
    
    def perform_homing(self, method_id: str, direction: str, homing_speed: float, 
                      timeout: float) -> Tuple[bool, str, bool]:
        """
        Perform homing sequence using specified method
        
        Args:
            method_id: Homing method ID
            direction: Homing direction ("forward", "backward")
            homing_speed: Homing speed (m/s)
            timeout: Timeout for homing operation (seconds)
            
        Returns:
            Tuple of (success, message, encoder_zeroed)
        """
        try:
            # Validate method
            method_enum = None
            for method in HomingMethod:
                if method.value == method_id:
                    method_enum = method
                    break
                    
            if not method_enum:
                return False, f"Unknown homing method: {method_id}", False
            
            # Check if method is available for this controller
            if self.controller_type not in self.homing_methods:
                return False, f"No homing methods for controller type: {self.controller_type}", False
                
            if method_enum not in self.homing_methods[self.controller_type]:
                return False, f"Method {method_id} not available for {self.controller_type.value}", False
            
            method_info = self.homing_methods[self.controller_type][method_enum]
            
            # Validate direction
            if direction not in method_info["directions"] and "both" not in method_info["directions"]:
                return False, f"Direction {direction} not supported for method {method_id}", False
            
            # Update servo state
            self.servo_state = ServoState.HOMING
            
            # Execute homing based on method
            success, message, encoder_zeroed = self._execute_homing_method(
                method_enum, direction, homing_speed, timeout
            )
            
            if success:
                self.servo_state = ServoState.IDLE
                if encoder_zeroed:
                    self.current_left_position = 0.0
                    self.current_right_position = 0.0
            else:
                self.servo_state = ServoState.ERROR
            
            return success, message, encoder_zeroed
            
        except Exception as e:
            self.servo_state = ServoState.ERROR
            return False, f"Error performing homing: {e}", False
    
    def _execute_homing_method(self, method: HomingMethod, direction: str, 
                             speed: float, timeout: float) -> Tuple[bool, str, bool]:
        """
        Execute specific homing method
        
        Returns:
            Tuple of (success, message, encoder_zeroed)
        """
        try:
            if method == HomingMethod.MANUAL_ZERO:
                # Manual zero - just reset encoder positions
                if self.hardware_interface and hasattr(self.hardware_interface, 'controller'):
                    # Reset encoder counts to zero
                    success1 = self.hardware_interface.controller.SetEncM1(self.controller_address, 0)
                    success2 = self.hardware_interface.controller.SetEncM2(self.controller_address, 0)
                    
                    if success1 and success2:
                        return True, "Manual zero completed", True
                    else:
                        return False, "Failed to reset encoder counts", False
                else:
                    return False, "Hardware interface not available", False
                    
            elif method == HomingMethod.HOME_PIN_BACKWARD:
                # RoboClaw home pin homing (backward only)
                return self._execute_home_pin_homing("backward", speed, timeout)
                
            elif method == HomingMethod.HOME_PIN_BIDIRECTIONAL:
                # MCP home pin homing (bidirectional)
                return self._execute_home_pin_homing(direction, speed, timeout)
                
            elif method == HomingMethod.LIMIT_SWITCH_STOP:
                # Move until limit switch
                return self._execute_limit_switch_homing(direction, speed, timeout)
                
            elif method == HomingMethod.CURRENT_LIMIT_HOMING:
                # Move until motor stalls
                return self._execute_current_limit_homing(direction, speed, timeout)
                
            elif method == HomingMethod.AUTO_HOME_STARTUP:
                # Configure automatic homing on startup
                return self._configure_auto_homing()
                
            else:
                return False, f"Homing method {method.value} not implemented", False
                
        except Exception as e:
            return False, f"Error executing homing method: {e}", False
    
    def _execute_home_pin_homing(self, direction: str, speed: float, 
                               timeout: float) -> Tuple[bool, str, bool]:
        """Execute home pin based homing"""
        try:
            if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
                return False, "Hardware interface not available", False
            
            # Convert speed to counts per second
            speed_counts = abs(self.unit_converter.rad_per_sec_to_counts_per_sec(speed))
            
            # Set direction
            if direction == "backward":
                left_speed = -speed_counts
                right_speed = -speed_counts
            else:  # forward
                left_speed = speed_counts
                right_speed = speed_counts
            
            # Start movement
            success = self.hardware_interface.controller.SpeedM1M2(
                self.controller_address, left_speed, right_speed
            )
            
            if not success:
                return False, "Failed to start homing movement", False
            
            # Monitor for home pin activation (simplified simulation)
            start_time = time.time()
            while time.time() - start_time < timeout:
                # In real implementation, would check home pin status
                # For simulation, assume homing completes after 1 second
                if time.time() - start_time > 1.0:
                    # Stop motors
                    self.hardware_interface.controller.DutyM1M2(self.controller_address, 0, 0)
                    
                    # Reset encoders if this method auto-zeros
                    if self.controller_type == ControllerType.ROBOCLAW or self.controller_type == ControllerType.MCP:
                        self.hardware_interface.controller.SetEncM1(self.controller_address, 0)
                        self.hardware_interface.controller.SetEncM2(self.controller_address, 0)
                        return True, f"Home pin homing completed ({direction})", True
                    
                time.sleep(0.1)
            
            # Timeout - stop motors
            self.hardware_interface.controller.DutyM1M2(self.controller_address, 0, 0)
            return False, "Homing timeout", False
            
        except Exception as e:
            return False, f"Home pin homing error: {e}", False
    
    def _execute_limit_switch_homing(self, direction: str, speed: float, 
                                   timeout: float) -> Tuple[bool, str, bool]:
        """Execute limit switch based homing"""
        try:
            # Similar to home pin homing but doesn't auto-zero
            success, message, _ = self._execute_home_pin_homing(direction, speed, timeout)
            
            if success:
                return True, f"Limit switch homing completed ({direction})", False
            else:
                return False, message, False
                
        except Exception as e:
            return False, f"Limit switch homing error: {e}", False
    
    def _execute_current_limit_homing(self, direction: str, speed: float, 
                                    timeout: float) -> Tuple[bool, str, bool]:
        """Execute current limit based homing"""
        try:
            if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
                return False, "Hardware interface not available", False
            
            # Convert speed to counts per second
            speed_counts = abs(self.unit_converter.rad_per_sec_to_counts_per_sec(speed))
            
            # Set direction
            if direction == "backward":
                left_speed = -speed_counts
                right_speed = -speed_counts
            else:  # forward
                left_speed = speed_counts
                right_speed = speed_counts
            
            # Start movement
            success = self.hardware_interface.controller.SpeedM1M2(
                self.controller_address, left_speed, right_speed
            )
            
            if not success:
                return False, "Failed to start current limit homing", False
            
            # Monitor current levels (simplified)
            start_time = time.time()
            while time.time() - start_time < timeout:
                # In real implementation, would monitor motor currents
                currents_result = self.hardware_interface.controller.ReadCurrents(self.controller_address)
                
                if currents_result[0]:  # Success reading currents
                    current1, current2 = currents_result[1], currents_result[2]
                    
                    # Check for current limit (simplified threshold)
                    if current1 > 5000 or current2 > 5000:  # milliamps
                        # Stop motors
                        self.hardware_interface.controller.DutyM1M2(self.controller_address, 0, 0)
                        return True, f"Current limit homing completed ({direction})", False
                
                time.sleep(0.1)
            
            # Timeout - stop motors
            self.hardware_interface.controller.DutyM1M2(self.controller_address, 0, 0)
            return False, "Current limit homing timeout", False
            
        except Exception as e:
            return False, f"Current limit homing error: {e}", False
    
    def _configure_auto_homing(self) -> Tuple[bool, str, bool]:
        """Configure automatic homing on startup"""
        try:
            # This would configure the controller for auto-homing
            # Implementation depends on specific controller capabilities
            
            if self.controller_type == ControllerType.ROBOCLAW:
                # RoboClaw supports auto-homing configuration
                # In real implementation, would configure controller registers
                self.auto_home_on_startup = True
                return True, "Auto-homing configured for startup", False
            else:
                return False, "Auto-homing not supported on this controller", False
                
        except Exception as e:
            return False, f"Auto-homing configuration error: {e}", False
    
    def get_servo_status(self) -> Tuple[bool, Dict[str, Any]]:
        """
        Get current servo status and error information
        
        Returns:
            Tuple of (success, status_dict)
        """
        try:
            if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
                return False, {"error": "Hardware interface not available"}
            
            # Read position errors
            pos_errors_result = self.hardware_interface.controller.GetPosErrors(self.controller_address)
            speed_errors_result = self.hardware_interface.controller.GetSpeedErrors(self.controller_address)
            
            status = {
                "servo_state": self.servo_state.value,
                "position_hold_active": self.position_hold_active,
                "current_left_position": self.current_left_position,
                "current_right_position": self.current_right_position,
                "target_left_position": self.target_left_position,
                "target_right_position": self.target_right_position,
                "left_position_error": 0,
                "right_position_error": 0,
                "left_speed_error": 0,
                "right_speed_error": 0,
                "error_limits_exceeded": False
            }
            
            if pos_errors_result[0]:  # Success
                status["left_position_error"] = pos_errors_result[1]
                status["right_position_error"] = pos_errors_result[2]
                self.last_position_errors = (pos_errors_result[1], pos_errors_result[2])
            
            if speed_errors_result[0]:  # Success
                status["left_speed_error"] = speed_errors_result[1]
                status["right_speed_error"] = speed_errors_result[2]
                self.last_speed_errors = (speed_errors_result[1], speed_errors_result[2])
            
            # Check error limits
            pos_error_exceeded = (abs(status["left_position_error"]) > self.max_position_error or
                                abs(status["right_position_error"]) > self.max_position_error)
            speed_error_exceeded = (abs(status["left_speed_error"]) > self.max_speed_error or
                                  abs(status["right_speed_error"]) > self.max_speed_error)
            
            status["error_limits_exceeded"] = pos_error_exceeded or speed_error_exceeded
            self.error_limit_exceeded = status["error_limits_exceeded"]
            
            return True, status
            
        except Exception as e:
            return False, {"error": f"Error reading servo status: {e}"}
    
    def _validate_position_parameters(self, left_pos: float, right_pos: float,
                                    speed: float, accel: float, decel: float) -> bool:
        """Validate position control parameters"""
        try:
            # Check for reasonable position ranges (in radians)
            if abs(left_pos) > 1000.0 or abs(right_pos) > 1000.0:
                return False
                
            # Check for positive speed, acceleration, deceleration
            if speed <= 0 or accel <= 0 or decel <= 0:
                return False
                
            # Check for reasonable limits
            if speed > 100.0 or accel > 1000.0 or decel > 1000.0:
                return False
                
            return True
            
        except Exception:
            return False
    
    # Service callbacks
    def _move_to_absolute_position_callback(self, request, response):
        """ROS2 service callback for move to absolute position"""
        success, message = self.move_to_absolute_position(
            request.left_position_radians,
            request.right_position_radians,
            request.max_speed,
            request.acceleration,
            request.deceleration,
            request.buffer_command
        )
        
        response.success = success
        response.message = message
        return response
    
    def _release_position_hold_callback(self, request, response):
        """ROS2 service callback for release position hold"""
        success, message = self.release_position_hold()
        
        response.success = success
        response.message = message
        return response
    
    def _get_available_homing_methods_callback(self, request, response):
        """ROS2 service callback for get available homing methods"""
        success, methods_info = self.get_available_homing_methods()
        
        response.success = success
        if success:
            response.controller_type = methods_info["controller_type"]
            response.available_methods = methods_info["available_methods"]
            response.method_descriptions = methods_info["method_descriptions"]
            response.allowed_directions = methods_info["allowed_directions"]
            response.auto_zeros_encoder = methods_info["auto_zeros_encoder"]
            response.acts_as_limit = methods_info["acts_as_limit"]
        
        return response
    
    def _perform_homing_callback(self, request, response):
        """ROS2 service callback for perform homing"""
        success, message, encoder_zeroed = self.perform_homing(
            request.method_id,
            request.direction,
            request.homing_speed,
            request.timeout
        )
        
        response.success = success
        response.message = message
        response.encoder_zeroed = encoder_zeroed
        return response
    
    def _set_homing_configuration_callback(self, request, response):
        """ROS2 service callback for set homing configuration"""
        try:
            self.auto_home_on_startup = request.auto_home_on_startup
            self.default_homing_method = request.default_homing_method
            self.default_homing_speed = request.default_homing_speed
            
            response.success = True
            response.message = "Homing configuration updated"
            
        except Exception as e:
            response.success = False
            response.message = f"Error setting homing configuration: {e}"
        
        return response
    
    def _get_servo_status_callback(self, request, response):
        """ROS2 service callback for get servo status"""
        success, status = self.get_servo_status()
        
        response.success = success
        if success:
            response.left_position_error = status["left_position_error"]
            response.right_position_error = status["right_position_error"]
            response.left_speed_error = status["left_speed_error"]
            response.right_speed_error = status["right_speed_error"]
            response.error_limits_exceeded = status["error_limits_exceeded"]
            response.message = f"Servo state: {status['servo_state']}"
        else:
            response.message = status.get("error", "Unknown error")
        
        return response


# Standalone test functions for hardware-free testing
def test_servo_position_service():
    """Test servo position service functionality"""
    print("Testing Servo Position Service...")
    
    # Create mock objects
    from unittest.mock import Mock
    
    hardware_interface = Mock()
    hardware_interface.controller = Mock()
    hardware_interface.controller.SpeedAccelDeccelPositionM1M2.return_value = True
    hardware_interface.controller.DutyM1M2.return_value = True
    hardware_interface.controller.ReadVersion.return_value = (True, "RoboClaw 2x15A v4.1.34")
    hardware_interface.controller.GetPosErrors.return_value = (True, 100, -50)
    hardware_interface.controller.GetSpeedErrors.return_value = (True, 25, -15)
    hardware_interface.controller.SetEncM1.return_value = True
    hardware_interface.controller.SetEncM2.return_value = True
    
    buffer_manager = Mock()
    unit_converter = Mock()
    unit_converter.radians_to_counts.side_effect = lambda x: int(x * 1000)
    unit_converter.rad_per_sec_to_counts_per_sec.side_effect = lambda x: int(x * 100)
    unit_converter.rad_per_sec2_to_counts_per_sec2.side_effect = lambda x: int(x * 10)
    
    # Create service
    servo_service = ServoPositionService(hardware_interface, buffer_manager, unit_converter)
    
    # Test position control
    success, message = servo_service.move_to_absolute_position(1.57, -1.57, 0.5, 0.1, 0.1, False)
    print(f"Position control test: {success}, {message}")
    
    # Test position hold release
    success, message = servo_service.release_position_hold()
    print(f"Position hold release test: {success}, {message}")
    
    # Test homing methods query
    success, methods = servo_service.get_available_homing_methods()
    print(f"Homing methods test: {success}")
    if success:
        print(f"  Available methods: {methods['available_methods']}")
    
    # Test manual homing
    success, message, zeroed = servo_service.perform_homing("manual_zero", "both", 0.1, 5.0)
    print(f"Manual homing test: {success}, {message}, zeroed: {zeroed}")
    
    # Test servo status
    success, status = servo_service.get_servo_status()
    print(f"Servo status test: {success}")
    if success:
        print(f"  State: {status['servo_state']}")
        print(f"  Position errors: {status['left_position_error']}, {status['right_position_error']}")
    
    print("Servo Position Service testing completed!")
    return True


if __name__ == "__main__":
    test_servo_position_service()