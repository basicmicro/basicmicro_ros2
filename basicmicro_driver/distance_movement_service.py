#!/usr/bin/env python3
"""
Distance-Based Movement Service for Basicmicro Driver

Implements distance-based movement commands using SpeedAccelDistanceM1M2
with buffer management and position limits enforcement.

Author: ROS2 Driver Development
"""

# Handle ROS2 imports gracefully for testing
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
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
            
        def set_parameters(self, params):
            pass
    
    class Parameter:
        class Type:
            BOOL = 'bool'
            DOUBLE = 'double'
            STRING = 'string'
        
        def __init__(self, name, param_type, value):
            self.name = name
            self.type = param_type
            self.value = value

import math
from typing import Tuple, Optional, Dict, Any

# ROS2 service imports
try:
    from basicmicro_driver.srv import MoveDistance, SetPositionLimits, GetPositionLimits
except ImportError:
    # Mock service types for testing without ROS2
    class MoveDistance:
        class Request:
            def __init__(self):
                self.left_distance = 0.0
                self.right_distance = 0.0
                self.speed = 1.0
                self.acceleration = 1.0
                self.use_buffer = False
        
        class Response:
            def __init__(self):
                self.success = False
                self.message = ""
                self.buffer_slots_used = 0
    
    class SetPositionLimits:
        class Request:
            def __init__(self):
                self.enable_limits = False
                self.left_min_position = 0.0
                self.left_max_position = 0.0
                self.right_min_position = 0.0
                self.right_max_position = 0.0
                self.violation_behavior = "warning"
                self.decel_rate = 1000.0
        
        class Response:
            def __init__(self):
                self.success = False
                self.message = ""
    
    class GetPositionLimits:
        class Request:
            def __init__(self):
                pass
        
        class Response:
            def __init__(self):
                self.limits_enabled = False
                self.left_min_position = 0.0
                self.left_max_position = 0.0
                self.right_min_position = 0.0
                self.right_max_position = 0.0
                self.violation_behavior = "warning"
                self.decel_rate = 1000.0

# Import existing components
try:
    from basicmicro_driver.unit_converter import UnitConverter
except ImportError:
    # Fallback for relative import in package context
    from .unit_converter import UnitConverter


class DistanceMovementService(Node if ROS2_AVAILABLE else object):
    """
    Service for distance-based movement commands with position limits enforcement.
    
    Provides services for:
    - Distance-based movement with buffer management
    - Position limit configuration and enforcement
    - Soft/hard stop capabilities for limit violations
    """
    
    def __init__(self, hardware_interface=None):
        """
        Initialize distance movement service.
        
        Args:
            hardware_interface: Reference to hardware interface for direct control
        """
        if ROS2_AVAILABLE:
            super().__init__('distance_movement_service')
        
        # Hardware interface reference
        self.hardware_interface = hardware_interface
        
        # Initialize unit converter (will get parameters from hardware interface)
        self._init_unit_converter()
        
        # Position limits configuration
        self.limits_enabled = False
        self.left_min_position = -math.inf
        self.left_max_position = math.inf
        self.right_min_position = -math.inf
        self.right_max_position = math.inf
        self.violation_behavior = 'warning'  # 'hard_stop', 'soft_stop', 'warning'
        self.decel_rate = 1000.0  # Default deceleration rate for soft stops (m/s²)
        
        # Current position tracking (radians)
        self.current_left_position = 0.0
        self.current_right_position = 0.0
        
        # Service servers
        self.move_distance_service = self.create_service(
            MoveDistance, 
            'move_distance', 
            self._move_distance_callback
        )
        
        self.set_position_limits_service = self.create_service(
            SetPositionLimits,
            'set_position_limits',
            self._set_position_limits_callback
        )
        
        self.get_position_limits_service = self.create_service(
            GetPositionLimits,
            'get_position_limits',
            self._get_position_limits_callback
        )
        
        # Parameters for position limits
        self._declare_position_limit_parameters()
        
        if ROS2_AVAILABLE:
            self.get_logger().info("Distance Movement Service initialized")
        else:
            print("Distance Movement Service initialized (ROS2 mock mode)")
    
    def _init_unit_converter(self):
        """Initialize unit converter with parameters from hardware interface."""
        if self.hardware_interface:
            # Get conversion parameters from hardware interface
            wheel_radius = getattr(self.hardware_interface, 'wheel_radius', 0.1)
            encoder_counts_per_rev = getattr(self.hardware_interface, 'encoder_counts_per_rev', 1000)
            gear_ratio = getattr(self.hardware_interface, 'gear_ratio', 1.0)
        else:
            # Default values for testing
            wheel_radius = 0.1
            encoder_counts_per_rev = 1000
            gear_ratio = 1.0
            
        self.unit_converter = UnitConverter(
            wheel_radius=wheel_radius,
            encoder_counts_per_rev=encoder_counts_per_rev,
            gear_ratio=gear_ratio
        )
    
    def _declare_position_limit_parameters(self):
        """Declare ROS2 parameters for position limits."""
        self.declare_parameter('position_limits.enabled', False)
        self.declare_parameter('position_limits.left_min', -1000000.0)
        self.declare_parameter('position_limits.left_max', 1000000.0)
        self.declare_parameter('position_limits.right_min', -1000000.0)
        self.declare_parameter('position_limits.right_max', 1000000.0)
        self.declare_parameter('position_limits.violation_behavior', 'warning')
        self.declare_parameter('position_limits.decel_rate', 1000.0)
        
        # Load current parameter values
        self._load_position_limit_parameters()
    
    def _load_position_limit_parameters(self):
        """Load position limit parameters from ROS2 parameter server."""
        self.limits_enabled = self.get_parameter('position_limits.enabled').value
        self.left_min_position = self.get_parameter('position_limits.left_min').value
        self.left_max_position = self.get_parameter('position_limits.left_max').value
        self.right_min_position = self.get_parameter('position_limits.right_min').value
        self.right_max_position = self.get_parameter('position_limits.right_max').value
        self.violation_behavior = self.get_parameter('position_limits.violation_behavior').value
        self.decel_rate = self.get_parameter('position_limits.decel_rate').value
    
    def _move_distance_callback(self, request, response):
        """
        Handle distance movement service requests.
        
        Args:
            request: MoveDistance request
            response: MoveDistance response
            
        Returns:
            MoveDistance response with result
        """
        try:
            # Validate input parameters
            validation_result = self._validate_distance_request(request)
            if not validation_result['valid']:
                response.success = False
                response.message = validation_result['message']
                response.buffer_slots_used = 0
                return response
            
            # Check position limits if enabled
            if self.limits_enabled:
                limit_check = self._check_position_limits(
                    request.left_distance, 
                    request.right_distance
                )
                if not limit_check['allowed']:
                    if self.violation_behavior == 'hard_stop':
                        # Immediately stop motors
                        self._emergency_stop()
                        response.success = False
                        response.message = f"Position limit violation - emergency stop: {limit_check['message']}"
                        response.buffer_slots_used = 0
                        return response
                    elif self.violation_behavior == 'soft_stop':
                        # Modify the command to stop before limit
                        request = self._modify_command_for_soft_stop(request, limit_check)
                    else:  # warning
                        self.get_logger().warn(f"Position limit warning: {limit_check['message']}")
            
            # Execute the distance command
            execution_result = self._execute_distance_command(request)
            
            response.success = execution_result['success']
            response.message = execution_result['message']
            response.buffer_slots_used = execution_result.get('buffer_slots_used', 0)
            
            # Update position tracking
            if execution_result['success']:
                self._update_position_tracking(request.left_distance, request.right_distance)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in distance movement: {str(e)}")
            response.success = False
            response.message = f"Service error: {str(e)}"
            response.buffer_slots_used = 0
            return response
    
    def _validate_distance_request(self, request) -> Dict[str, Any]:
        """
        Validate distance movement request parameters.
        
        Args:
            request: MoveDistance request
            
        Returns:
            Dictionary with validation result
        """
        # Check distance limits (reasonable physical limits)
        max_distance = 1000.0  # 1000 meters max
        if abs(request.left_distance) > max_distance or abs(request.right_distance) > max_distance:
            return {
                'valid': False,
                'message': f"Distance too large (max: {max_distance}m)"
            }
        
        # Check speed limits
        max_speed = 10.0  # 10 m/s max
        if abs(request.speed) > max_speed:
            return {
                'valid': False,
                'message': f"Speed too high (max: {max_speed} m/s)"
            }
        
        if request.speed <= 0:
            return {
                'valid': False,
                'message': "Speed must be positive"
            }
        
        # Check acceleration limits
        max_acceleration = 50.0  # 50 m/s² max
        if abs(request.acceleration) > max_acceleration:
            return {
                'valid': False,
                'message': f"Acceleration too high (max: {max_acceleration} m/s²)"
            }
        
        if request.acceleration <= 0:
            return {
                'valid': False,
                'message': "Acceleration must be positive"
            }
        
        return {'valid': True, 'message': 'Valid request'}
    
    def _check_position_limits(self, left_distance: float, right_distance: float) -> Dict[str, Any]:
        """
        Check if the requested movement would violate position limits.
        
        Args:
            left_distance: Distance for left wheel (meters)
            right_distance: Distance for right wheel (meters)
            
        Returns:
            Dictionary with limit check result
        """
        # Convert distances to radians
        left_distance_rad = self.unit_converter.distance_to_radians(left_distance)
        right_distance_rad = self.unit_converter.distance_to_radians(right_distance)
        
        # Calculate final positions
        final_left_pos = self.current_left_position + left_distance_rad
        final_right_pos = self.current_right_position + right_distance_rad
        
        # Check limits
        violations = []
        
        if final_left_pos < self.left_min_position:
            violations.append(f"Left wheel below minimum ({final_left_pos:.3f} < {self.left_min_position:.3f} rad)")
        
        if final_left_pos > self.left_max_position:
            violations.append(f"Left wheel above maximum ({final_left_pos:.3f} > {self.left_max_position:.3f} rad)")
        
        if final_right_pos < self.right_min_position:
            violations.append(f"Right wheel below minimum ({final_right_pos:.3f} < {self.right_min_position:.3f} rad)")
        
        if final_right_pos > self.right_max_position:
            violations.append(f"Right wheel above maximum ({final_right_pos:.3f} > {self.right_max_position:.3f} rad)")
        
        if violations:
            return {
                'allowed': False,
                'message': '; '.join(violations),
                'violations': violations
            }
        
        return {'allowed': True, 'message': 'Within limits'}
    
    def _modify_command_for_soft_stop(self, request, limit_check):
        """
        Modify distance command to stop before position limits for soft stop behavior.
        
        Args:
            request: Original MoveDistance request
            limit_check: Result from position limit check
            
        Returns:
            Modified request that stops before limits
        """
        # Calculate safe distances that stay within limits
        left_available = min(
            self.left_max_position - self.current_left_position,
            self.current_left_position - self.left_min_position
        )
        
        right_available = min(
            self.right_max_position - self.current_right_position,
            self.current_right_position - self.right_min_position
        )
        
        # Convert to meters and apply safety margin
        safety_margin = 0.1  # 10cm safety margin
        left_safe_distance = max(0, self.unit_converter.radians_to_distance(left_available) - safety_margin)
        right_safe_distance = max(0, self.unit_converter.radians_to_distance(right_available) - safety_margin)
        
        # Modify request to use safe distances
        request.left_distance = min(abs(request.left_distance), left_safe_distance) * (1 if request.left_distance >= 0 else -1)
        request.right_distance = min(abs(request.right_distance), right_safe_distance) * (1 if request.right_distance >= 0 else -1)
        
        self.get_logger().warn(f"Modified command for soft stop: left={request.left_distance:.3f}m, right={request.right_distance:.3f}m")
        
        return request
    
    def _execute_distance_command(self, request) -> Dict[str, Any]:
        """
        Execute the distance movement command using Basicmicro library.
        
        Args:
            request: MoveDistance request
            
        Returns:
            Dictionary with execution result
        """
        if not self.hardware_interface:
            return {
                'success': False,
                'message': 'No hardware interface available'
            }
        
        try:
            # Convert units to encoder counts and counts/sec
            left_distance_counts = self.unit_converter.distance_to_counts(request.left_distance)
            right_distance_counts = self.unit_converter.distance_to_counts(request.right_distance)
            speed_counts = self.unit_converter.speed_to_counts_per_sec(request.speed)
            acceleration_counts = self.unit_converter.acceleration_to_counts_per_sec2(request.acceleration)
            
            # Get controller reference
            controller = self.hardware_interface.controller
            address = self.hardware_interface.address
            
            # Buffer flag: 1 for buffered, 0 for immediate execution
            buffer_flag = 1 if request.use_buffer else 0
            
            # Execute SpeedAccelDistanceM1M2 command
            result = controller.SpeedAccelDistanceM1M2(
                address,
                acceleration_counts,  # acceleration
                speed_counts,         # speed for motor 1
                left_distance_counts, # distance for motor 1
                speed_counts,         # speed for motor 2  
                right_distance_counts,# distance for motor 2
                buffer_flag           # buffer flag
            )
            
            if result[0]:  # Success
                # Check buffer status if buffered
                buffer_slots_used = 0
                if request.use_buffer:
                    buffer_status = controller.ReadBuffers(address)
                    if buffer_status[0]:
                        raw_value = buffer_status[1]
                        if raw_value != 0xFF and raw_value != 0:
                            buffer_slots_used = raw_value
                
                self.get_logger().info(f"Distance command executed: left={request.left_distance:.3f}m, right={request.right_distance:.3f}m")
                return {
                    'success': True,
                    'message': f'Distance command executed successfully',
                    'buffer_slots_used': buffer_slots_used
                }
            else:
                return {
                    'success': False,
                    'message': 'Failed to execute distance command - controller communication error'
                }
                
        except Exception as e:
            return {
                'success': False,
                'message': f'Command execution error: {str(e)}'
            }
    
    def _emergency_stop(self):
        """Execute emergency stop by clearing all buffers."""
        if self.hardware_interface:
            try:
                controller = self.hardware_interface.controller
                address = self.hardware_interface.address
                controller.DutyM1M2(address, 0, 0)  # Clear all buffers and stop
                self.get_logger().warn("Emergency stop executed due to position limit violation")
            except Exception as e:
                self.get_logger().error(f"Emergency stop failed: {e}")
    
    def _update_position_tracking(self, left_distance: float, right_distance: float):
        """
        Update internal position tracking.
        
        Args:
            left_distance: Distance traveled by left wheel (meters)
            right_distance: Distance traveled by right wheel (meters)
        """
        left_distance_rad = self.unit_converter.distance_to_radians(left_distance)
        right_distance_rad = self.unit_converter.distance_to_radians(right_distance)
        
        self.current_left_position += left_distance_rad
        self.current_right_position += right_distance_rad
    
    def _set_position_limits_callback(self, request, response):
        """
        Handle set position limits service requests.
        
        Args:
            request: SetPositionLimits request
            response: SetPositionLimits response
            
        Returns:
            SetPositionLimits response
        """
        try:
            # Validate violation behavior
            valid_behaviors = ['hard_stop', 'soft_stop', 'warning']
            if request.violation_behavior not in valid_behaviors:
                response.success = False
                response.message = f"Invalid violation behavior. Must be one of: {valid_behaviors}"
                return response
            
            # Validate position limits
            if request.left_min_position >= request.left_max_position:
                response.success = False
                response.message = "Left minimum position must be less than maximum position"
                return response
            
            if request.right_min_position >= request.right_max_position:
                response.success = False
                response.message = "Right minimum position must be less than maximum position"
                return response
            
            if request.decel_rate <= 0:
                response.success = False
                response.message = "Deceleration rate must be positive"
                return response
            
            # Update configuration
            self.limits_enabled = request.enable_limits
            self.left_min_position = request.left_min_position
            self.left_max_position = request.left_max_position
            self.right_min_position = request.right_min_position
            self.right_max_position = request.right_max_position
            self.violation_behavior = request.violation_behavior
            self.decel_rate = request.decel_rate
            
            # Update ROS2 parameters
            self._update_position_limit_parameters()
            
            response.success = True
            response.message = f"Position limits configured: enabled={self.limits_enabled}, behavior={self.violation_behavior}"
            
            self.get_logger().info(f"Position limits updated: enabled={self.limits_enabled}")
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error setting position limits: {str(e)}")
            response.success = False
            response.message = f"Service error: {str(e)}"
            return response
    
    def _get_position_limits_callback(self, request, response):
        """
        Handle get position limits service requests.
        
        Args:
            request: GetPositionLimits request
            response: GetPositionLimits response
            
        Returns:
            GetPositionLimits response
        """
        try:
            # Return current configuration
            response.limits_enabled = self.limits_enabled
            response.left_min_position = self.left_min_position
            response.left_max_position = self.left_max_position
            response.right_min_position = self.right_min_position
            response.right_max_position = self.right_max_position
            response.violation_behavior = self.violation_behavior
            response.decel_rate = self.decel_rate
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error getting position limits: {str(e)}")
            # Return default values on error
            response.limits_enabled = False
            response.left_min_position = -math.inf
            response.left_max_position = math.inf
            response.right_min_position = -math.inf
            response.right_max_position = math.inf
            response.violation_behavior = 'warning'
            response.decel_rate = 1000.0
            return response
    
    def _update_position_limit_parameters(self):
        """Update ROS2 parameters with current position limit configuration."""
        try:
            self.set_parameters([
                Parameter('position_limits.enabled', Parameter.Type.BOOL, self.limits_enabled),
                Parameter('position_limits.left_min', Parameter.Type.DOUBLE, self.left_min_position),
                Parameter('position_limits.left_max', Parameter.Type.DOUBLE, self.left_max_position),
                Parameter('position_limits.right_min', Parameter.Type.DOUBLE, self.right_min_position),
                Parameter('position_limits.right_max', Parameter.Type.DOUBLE, self.right_max_position),
                Parameter('position_limits.violation_behavior', Parameter.Type.STRING, self.violation_behavior),
                Parameter('position_limits.decel_rate', Parameter.Type.DOUBLE, self.decel_rate)
            ])
        except Exception as e:
            self.get_logger().warn(f"Failed to update position limit parameters: {e}")
    
    def update_current_position(self, left_position_rad: float, right_position_rad: float):
        """
        Update current position tracking from external source (e.g., hardware interface).
        
        Args:
            left_position_rad: Current left wheel position in radians
            right_position_rad: Current right wheel position in radians
        """
        self.current_left_position = left_position_rad
        self.current_right_position = right_position_rad
    
    def get_current_configuration(self) -> Dict[str, Any]:
        """
        Get current distance movement service configuration.
        
        Returns:
            Dictionary with current configuration
        """
        return {
            'limits_enabled': self.limits_enabled,
            'left_min_position': self.left_min_position,
            'left_max_position': self.left_max_position,
            'right_min_position': self.right_min_position,
            'right_max_position': self.right_max_position,
            'violation_behavior': self.violation_behavior,
            'decel_rate': self.decel_rate,
            'current_left_position': self.current_left_position,
            'current_right_position': self.current_right_position
        }


def main(args=None):
    """Main entry point for distance movement service."""
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        
        try:
            service = DistanceMovementService()
            rclpy.spin(service)
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    else:
        print("ROS2 not available - running in test mode")
        service = DistanceMovementService()
        print("Distance Movement Service created successfully in test mode")


if __name__ == '__main__':
    main()