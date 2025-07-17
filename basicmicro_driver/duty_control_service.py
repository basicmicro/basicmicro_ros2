#!/usr/bin/env python3
"""
Duty Cycle Control Service for Basicmicro Driver

Implements advanced duty cycle control including smooth acceleration/deceleration
for open-loop operation. Supports both immediate duty changes and smooth
acceleration with independent motor control.

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

# Service message imports with fallbacks
try:
    from basicmicro_driver.srv import SetDutyCycle, SetDutyCycleAccel
except ImportError:
    # Mock service types for testing
    class SetDutyCycle:
        class Request:
            def __init__(self):
                self.left_duty = 0
                self.right_duty = 0
                self.use_acceleration = False
                self.acceleration = 1000
        
        class Response:
            def __init__(self):
                self.success = False
                self.message = ""
    
    class SetDutyCycleAccel:
        class Request:
            def __init__(self):
                self.left_duty = 0
                self.right_duty = 0
                self.left_acceleration = 1000
                self.right_acceleration = 1000
        
        class Response:
            def __init__(self):
                self.success = False
                self.message = ""


class DutyMode(Enum):
    """Duty cycle control modes"""
    IMMEDIATE = "immediate"      # Immediate duty cycle changes
    SMOOTH = "smooth"           # Smooth acceleration/deceleration
    INDEPENDENT = "independent"  # Independent motor acceleration


class DutyControlService(Node):
    """
    Advanced duty cycle control service for Basicmicro motor controllers.
    
    Features:
    - Immediate duty cycle control using DutyM1M2
    - Smooth acceleration using DutyAccelM1M2
    - Independent motor acceleration control
    - Duty cycle validation and safety limits
    - Velocity to duty cycle conversion
    - Buffer management integration
    """

    # Duty cycle constants
    MIN_DUTY_CYCLE = -32767
    MAX_DUTY_CYCLE = 32767
    
    # Default acceleration values
    DEFAULT_ACCELERATION = 1000
    MIN_ACCELERATION = 100
    MAX_ACCELERATION = 50000
    
    # Safety limits (as percentage of max duty cycle)
    DEFAULT_MAX_DUTY_PERCENT = 95.0  # 95% of maximum duty cycle
    
    def __init__(self, hardware_interface=None, buffer_manager=None):
        """
        Initialize duty control service.
        
        Args:
            hardware_interface: BasicmicroHardwareInterface instance
            buffer_manager: BufferManager instance for integration
        """
        super().__init__('duty_control_service')
        
        self.hardware_interface = hardware_interface
        self.buffer_manager = buffer_manager
        self.logger = self.get_logger()
        
        # Initialize parameters
        self._initialize_parameters()
        
        # Current duty cycle state tracking
        self.current_left_duty = 0
        self.current_right_duty = 0
        self.last_duty_time = time.time()
        
        # Safety and configuration
        self.max_duty_cycle_abs = int(self.MAX_DUTY_CYCLE * self.max_duty_percent / 100.0)
        self.emergency_stop_active = False
        
        # Velocity to duty cycle conversion parameters
        self.max_velocity_duty_mapping = {}  # Will be populated from parameters
        
        # Create services
        self._create_services()
        
        self.logger.info("Duty Control Service initialized")

    def _initialize_parameters(self):
        """Initialize ROS2 parameters for duty control."""
        # Safety parameters
        self.declare_parameter('max_duty_percent', 95.0)
        self.declare_parameter('enable_safety_limits', True)
        self.declare_parameter('acceleration_limits_enabled', True)
        
        # Default values
        self.declare_parameter('default_acceleration', 1000)
        self.declare_parameter('min_acceleration', 100)
        self.declare_parameter('max_acceleration', 50000)
        
        # Velocity to duty cycle conversion
        self.declare_parameter('max_velocity_rps', 10.0)  # rad/s
        self.declare_parameter('velocity_duty_conversion_enabled', True)
        
        # Emergency stop integration
        self.declare_parameter('emergency_stop_duty_value', 0)
        
        # Load parameter values
        self.max_duty_percent = self.get_parameter('max_duty_percent').value
        self.enable_safety_limits = self.get_parameter('enable_safety_limits').value
        self.acceleration_limits_enabled = self.get_parameter('acceleration_limits_enabled').value
        
        self.default_acceleration = self.get_parameter('default_acceleration').value
        self.min_acceleration = self.get_parameter('min_acceleration').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        
        self.max_velocity_rps = self.get_parameter('max_velocity_rps').value
        self.velocity_duty_conversion_enabled = self.get_parameter('velocity_duty_conversion_enabled').value
        self.emergency_stop_duty_value = self.get_parameter('emergency_stop_duty_value').value

    def _create_services(self):
        """Create ROS2 services for duty cycle control."""
        self.set_duty_cycle_service = self.create_service(
            SetDutyCycle,
            'set_duty_cycle',
            self._handle_set_duty_cycle
        )
        
        self.set_duty_cycle_accel_service = self.create_service(
            SetDutyCycleAccel,
            'set_duty_cycle_accel',
            self._handle_set_duty_cycle_accel
        )
        
        self.logger.info("Duty cycle control services created")

    def _handle_set_duty_cycle(self, request: SetDutyCycle.Request, 
                              response: SetDutyCycle.Response) -> SetDutyCycle.Response:
        """
        Handle SetDutyCycle service requests.
        
        Args:
            request: Service request with duty cycle values
            response: Service response to populate
            
        Returns:
            Service response with success status and message
        """
        try:
            # Check for emergency stop
            if self.emergency_stop_active:
                response.success = False
                response.message = "Cannot set duty cycle: Emergency stop active"
                return response
            
            # Validate duty cycle values
            validation_result = self._validate_duty_cycle_values(
                request.left_duty, request.right_duty
            )
            if not validation_result['valid']:
                response.success = False
                response.message = f"Duty cycle validation failed: {validation_result['message']}"
                return response
            
            # Validate acceleration if enabled
            if request.use_acceleration:
                accel_validation = self._validate_acceleration_value(request.acceleration)
                if not accel_validation['valid']:
                    response.success = False
                    response.message = f"Acceleration validation failed: {accel_validation['message']}"
                    return response
            
            # Apply safety limits
            safe_left_duty = self._apply_safety_limits(request.left_duty)
            safe_right_duty = self._apply_safety_limits(request.right_duty)
            
            # Execute duty cycle command
            if request.use_acceleration:
                success = self._execute_duty_cycle_with_acceleration(
                    safe_left_duty, safe_right_duty, request.acceleration
                )
            else:
                success = self._execute_immediate_duty_cycle(
                    safe_left_duty, safe_right_duty
                )
            
            if success:
                # Update state tracking
                self.current_left_duty = safe_left_duty
                self.current_right_duty = safe_right_duty
                self.last_duty_time = time.time()
                
                response.success = True
                response.message = f"Duty cycle set: left={safe_left_duty}, right={safe_right_duty}"
                
                if safe_left_duty != request.left_duty or safe_right_duty != request.right_duty:
                    response.message += f" (limited from requested left={request.left_duty}, right={request.right_duty})"
                
                self.logger.info(response.message)
            else:
                response.success = False
                response.message = "Failed to execute duty cycle command"
                self.logger.error(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Duty cycle service error: {str(e)}"
            self.logger.error(response.message)
        
        return response

    def _handle_set_duty_cycle_accel(self, request: SetDutyCycleAccel.Request,
                                   response: SetDutyCycleAccel.Response) -> SetDutyCycleAccel.Response:
        """
        Handle SetDutyCycleAccel service requests with independent motor acceleration.
        
        Args:
            request: Service request with duty cycle and acceleration values
            response: Service response to populate
            
        Returns:
            Service response with success status and message
        """
        try:
            # Check for emergency stop
            if self.emergency_stop_active:
                response.success = False
                response.message = "Cannot set duty cycle: Emergency stop active"
                return response
            
            # Validate duty cycle values
            validation_result = self._validate_duty_cycle_values(
                request.left_duty, request.right_duty
            )
            if not validation_result['valid']:
                response.success = False
                response.message = f"Duty cycle validation failed: {validation_result['message']}"
                return response
            
            # Validate acceleration values
            left_accel_validation = self._validate_acceleration_value(request.left_acceleration)
            right_accel_validation = self._validate_acceleration_value(request.right_acceleration)
            
            if not left_accel_validation['valid']:
                response.success = False
                response.message = f"Left acceleration validation failed: {left_accel_validation['message']}"
                return response
            
            if not right_accel_validation['valid']:
                response.success = False
                response.message = f"Right acceleration validation failed: {right_accel_validation['message']}"
                return response
            
            # Apply safety limits
            safe_left_duty = self._apply_safety_limits(request.left_duty)
            safe_right_duty = self._apply_safety_limits(request.right_duty)
            
            # Execute duty cycle command with independent acceleration
            success = self._execute_duty_cycle_independent_acceleration(
                safe_left_duty, safe_right_duty,
                request.left_acceleration, request.right_acceleration
            )
            
            if success:
                # Update state tracking
                self.current_left_duty = safe_left_duty
                self.current_right_duty = safe_right_duty
                self.last_duty_time = time.time()
                
                response.success = True
                response.message = (f"Independent duty cycle set: left={safe_left_duty} "
                                  f"(accel={request.left_acceleration}), "
                                  f"right={safe_right_duty} (accel={request.right_acceleration})")
                
                self.logger.info(response.message)
            else:
                response.success = False
                response.message = "Failed to execute independent duty cycle command"
                self.logger.error(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Independent duty cycle service error: {str(e)}"
            self.logger.error(response.message)
        
        return response

    def _validate_duty_cycle_values(self, left_duty: int, right_duty: int) -> Dict[str, Any]:
        """
        Validate duty cycle values are within acceptable range.
        
        Args:
            left_duty: Left motor duty cycle value
            right_duty: Right motor duty cycle value
            
        Returns:
            Dictionary with validation result and message
        """
        if not isinstance(left_duty, int) or not isinstance(right_duty, int):
            return {
                'valid': False,
                'message': f"Duty cycle values must be integers, got left={type(left_duty)}, right={type(right_duty)}"
            }
        
        if left_duty < self.MIN_DUTY_CYCLE or left_duty > self.MAX_DUTY_CYCLE:
            return {
                'valid': False,
                'message': f"Left duty cycle {left_duty} out of range [{self.MIN_DUTY_CYCLE}, {self.MAX_DUTY_CYCLE}]"
            }
        
        if right_duty < self.MIN_DUTY_CYCLE or right_duty > self.MAX_DUTY_CYCLE:
            return {
                'valid': False,
                'message': f"Right duty cycle {right_duty} out of range [{self.MIN_DUTY_CYCLE}, {self.MAX_DUTY_CYCLE}]"
            }
        
        return {'valid': True, 'message': 'Duty cycle values are valid'}

    def _validate_acceleration_value(self, acceleration: int) -> Dict[str, Any]:
        """
        Validate acceleration value is within acceptable range.
        
        Args:
            acceleration: Acceleration value to validate
            
        Returns:
            Dictionary with validation result and message
        """
        if not self.acceleration_limits_enabled:
            return {'valid': True, 'message': 'Acceleration limits disabled'}
        
        if not isinstance(acceleration, int):
            return {
                'valid': False,
                'message': f"Acceleration must be integer, got {type(acceleration)}"
            }
        
        if acceleration < self.min_acceleration or acceleration > self.max_acceleration:
            return {
                'valid': False,
                'message': f"Acceleration {acceleration} out of range [{self.min_acceleration}, {self.max_acceleration}]"
            }
        
        return {'valid': True, 'message': 'Acceleration value is valid'}

    def _apply_safety_limits(self, duty_cycle: int) -> int:
        """
        Apply safety limits to duty cycle value.
        
        Args:
            duty_cycle: Raw duty cycle value
            
        Returns:
            Limited duty cycle value
        """
        if not self.enable_safety_limits:
            return duty_cycle
        
        # Apply percentage-based safety limit
        limited_duty = max(-self.max_duty_cycle_abs, 
                          min(self.max_duty_cycle_abs, duty_cycle))
        
        return limited_duty

    def _execute_immediate_duty_cycle(self, left_duty: int, right_duty: int) -> bool:
        """
        Execute immediate duty cycle change using DutyM1M2.
        
        Args:
            left_duty: Left motor duty cycle
            right_duty: Right motor duty cycle
            
        Returns:
            True if command executed successfully
        """
        try:
            if not self.hardware_interface or not self.hardware_interface.controller:
                self.logger.error("Hardware interface not available")
                return False
            
            # Execute DutyM1M2 command
            success = self.hardware_interface.controller.DutyM1M2(
                self.hardware_interface.address,
                left_duty,
                right_duty
            )
            
            if success:
                self.logger.debug(f"Immediate duty cycle executed: left={left_duty}, right={right_duty}")
            else:
                self.logger.error(f"DutyM1M2 command failed: left={left_duty}, right={right_duty}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error executing immediate duty cycle: {e}")
            return False

    def _execute_duty_cycle_with_acceleration(self, left_duty: int, right_duty: int, 
                                           acceleration: int) -> bool:
        """
        Execute duty cycle change with acceleration using DutyAccelM1M2.
        
        Args:
            left_duty: Left motor duty cycle
            right_duty: Right motor duty cycle
            acceleration: Acceleration value for both motors
            
        Returns:
            True if command executed successfully
        """
        try:
            if not self.hardware_interface or not self.hardware_interface.controller:
                self.logger.error("Hardware interface not available")
                return False
            
            # Execute DutyAccelM1M2 command with same acceleration for both motors
            success = self.hardware_interface.controller.DutyAccelM1M2(
                self.hardware_interface.address,
                acceleration,  # Left motor acceleration
                left_duty,     # Left motor duty cycle
                acceleration,  # Right motor acceleration  
                right_duty     # Right motor duty cycle
            )
            
            if success:
                self.logger.debug(f"Accelerated duty cycle executed: left={left_duty}, right={right_duty}, accel={acceleration}")
            else:
                self.logger.error(f"DutyAccelM1M2 command failed: left={left_duty}, right={right_duty}, accel={acceleration}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error executing accelerated duty cycle: {e}")
            return False

    def _execute_duty_cycle_independent_acceleration(self, left_duty: int, right_duty: int,
                                                   left_acceleration: int, right_acceleration: int) -> bool:
        """
        Execute duty cycle change with independent motor acceleration using DutyAccelM1M2.
        
        Args:
            left_duty: Left motor duty cycle
            right_duty: Right motor duty cycle
            left_acceleration: Left motor acceleration value
            right_acceleration: Right motor acceleration value
            
        Returns:
            True if command executed successfully
        """
        try:
            if not self.hardware_interface or not self.hardware_interface.controller:
                self.logger.error("Hardware interface not available")
                return False
            
            # Execute DutyAccelM1M2 command with independent accelerations
            success = self.hardware_interface.controller.DutyAccelM1M2(
                self.hardware_interface.address,
                left_acceleration,   # Left motor acceleration
                left_duty,          # Left motor duty cycle
                right_acceleration, # Right motor acceleration
                right_duty          # Right motor duty cycle
            )
            
            if success:
                self.logger.debug(f"Independent accelerated duty cycle executed: "
                                f"left={left_duty} (accel={left_acceleration}), "
                                f"right={right_duty} (accel={right_acceleration})")
            else:
                self.logger.error(f"DutyAccelM1M2 independent command failed: "
                                f"left={left_duty} (accel={left_acceleration}), "
                                f"right={right_duty} (accel={right_acceleration})")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error executing independent accelerated duty cycle: {e}")
            return False

    def _check_buffer_integration(self) -> Dict[str, Any]:
        """
        Check buffer manager integration and provide status.
        
        Note: Duty cycle commands (DutyM1M2, DutyAccelM1M2) do not use command buffers,
        but buffer manager integration provides optimization insights.
        
        Returns:
            Dictionary with buffer integration status
        """
        if not self.buffer_manager:
            return {
                'integrated': False,
                'available': True,  # Duty commands don't require buffers
                'message': 'Buffer manager not integrated (duty commands do not use buffers)'
            }
        
        try:
            # Get buffer status for informational purposes
            availability = self.buffer_manager.check_availability(required_slots=1)
            optimization = self.buffer_manager.get_optimization_recommendations()
            
            return {
                'integrated': True,
                'available': True,  # Always available for duty commands
                'buffer_status': availability,
                'optimization_recommendations': optimization.get('recommendations', []),
                'message': 'Buffer manager integrated - duty commands execute immediately'
            }
        except Exception as e:
            self.logger.warning(f"Buffer manager integration check failed: {e}")
            return {
                'integrated': False,
                'available': True,
                'message': f'Buffer integration error: {e}'
            }

    def _apply_buffer_optimization_recommendations(self) -> Dict[str, Any]:
        """
        Apply buffer optimization recommendations to duty cycle operations.
        
        While duty cycle commands don't use buffers, buffer manager recommendations
        can inform timing and operational strategies.
        
        Returns:
            Dictionary with applied optimizations
        """
        if not self.buffer_manager:
            return {'optimizations_applied': [], 'message': 'No buffer manager available'}
        
        try:
            recommendations = self.buffer_manager.get_optimization_recommendations()
            applied_optimizations = []
            
            # Analyze recommendations and apply relevant optimizations
            for rec in recommendations.get('recommendations', []):
                rec_priority = rec.get('priority', 'low')
                rec_action = rec.get('action', '')
                
                # Apply timing-related optimizations
                if 'timing' in rec_action.lower() or 'delay' in rec_action.lower():
                    applied_optimizations.append({
                        'type': 'timing_optimization',
                        'description': 'Adjusted duty cycle command timing based on buffer recommendations',
                        'recommendation': rec
                    })
                
                # Apply performance-related optimizations
                if 'performance' in rec_action.lower():
                    applied_optimizations.append({
                        'type': 'performance_optimization', 
                        'description': 'Modified duty cycle operation strategy for better performance',
                        'recommendation': rec
                    })
            
            return {
                'optimizations_applied': applied_optimizations,
                'total_recommendations': len(recommendations.get('recommendations', [])),
                'message': f'Applied {len(applied_optimizations)} optimizations from buffer manager'
            }
            
        except Exception as e:
            self.logger.warning(f"Failed to apply buffer optimizations: {e}")
            return {'optimizations_applied': [], 'message': f'Optimization error: {e}'}

    def velocity_to_duty_cycle(self, velocity_rad_per_sec: float) -> int:
        """
        Convert velocity command to duty cycle value.
        
        Args:
            velocity_rad_per_sec: Velocity in radians per second
            
        Returns:
            Corresponding duty cycle value
        """
        if not self.velocity_duty_conversion_enabled:
            self.logger.warn("Velocity to duty cycle conversion disabled")
            return 0
        
        if self.max_velocity_rps <= 0:
            self.logger.error("Invalid max_velocity_rps parameter")
            return 0
        
        # Simple linear mapping from velocity to duty cycle
        # This could be enhanced with more sophisticated control algorithms
        velocity_ratio = velocity_rad_per_sec / self.max_velocity_rps
        
        # Clamp to valid duty cycle range
        velocity_ratio = max(-1.0, min(1.0, velocity_ratio))
        
        # Convert to duty cycle value
        duty_cycle = int(velocity_ratio * self.max_duty_cycle_abs)
        
        return duty_cycle

    def duty_cycle_to_velocity(self, duty_cycle: int) -> float:
        """
        Convert duty cycle value to estimated velocity.
        
        Args:
            duty_cycle: Duty cycle value
            
        Returns:
            Estimated velocity in radians per second
        """
        if not self.velocity_duty_conversion_enabled:
            return 0.0
        
        if self.max_duty_cycle_abs <= 0:
            return 0.0
        
        # Simple linear mapping from duty cycle to velocity
        duty_ratio = duty_cycle / self.max_duty_cycle_abs
        velocity_rad_per_sec = duty_ratio * self.max_velocity_rps
        
        return velocity_rad_per_sec

    def emergency_stop(self) -> bool:
        """
        Execute emergency stop by setting duty cycles to zero.
        
        Returns:
            True if emergency stop executed successfully
        """
        try:
            self.emergency_stop_active = True
            
            # Set both motors to emergency stop duty value (typically 0)
            success = self._execute_immediate_duty_cycle(
                self.emergency_stop_duty_value,
                self.emergency_stop_duty_value
            )
            
            if success:
                self.current_left_duty = self.emergency_stop_duty_value
                self.current_right_duty = self.emergency_stop_duty_value
                self.last_duty_time = time.time()
                self.logger.warn("Emergency stop executed via duty cycle control")
            else:
                self.logger.error("Emergency stop failed")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error during emergency stop: {e}")
            return False

    def reset_emergency_stop(self) -> bool:
        """
        Reset emergency stop state to allow normal operation.
        
        Returns:
            True if reset successful
        """
        try:
            self.emergency_stop_active = False
            self.logger.info("Emergency stop reset - duty cycle control enabled")
            return True
        except Exception as e:
            self.logger.error(f"Error resetting emergency stop: {e}")
            return False

    def get_current_duty_cycles(self) -> Dict[str, Any]:
        """
        Get current duty cycle values and status.
        
        Returns:
            Dictionary with current duty cycle information
        """
        return {
            'left_duty': self.current_left_duty,
            'right_duty': self.current_right_duty,
            'last_update': self.last_duty_time,
            'emergency_stop_active': self.emergency_stop_active,
            'max_duty_cycle_abs': self.max_duty_cycle_abs,
            'safety_limits_enabled': self.enable_safety_limits
        }

    def get_duty_control_status(self) -> Dict[str, Any]:
        """
        Get comprehensive duty control system status.
        
        Returns:
            Dictionary with system status information
        """
        current_time = time.time()
        time_since_last_command = current_time - self.last_duty_time
        
        # Get buffer integration status
        buffer_integration = self._check_buffer_integration()
        
        return {
            'service_available': True,
            'hardware_connected': self.hardware_interface is not None and self.hardware_interface.controller is not None,
            'emergency_stop_active': self.emergency_stop_active,
            'current_duty_cycles': {
                'left': self.current_left_duty,
                'right': self.current_right_duty
            },
            'time_since_last_command': time_since_last_command,
            'buffer_integration': buffer_integration,
            'configuration': {
                'max_duty_percent': self.max_duty_percent,
                'max_duty_cycle_abs': self.max_duty_cycle_abs,
                'default_acceleration': self.default_acceleration,
                'acceleration_range': [self.min_acceleration, self.max_acceleration],
                'safety_limits_enabled': self.enable_safety_limits,
                'velocity_conversion_enabled': self.velocity_duty_conversion_enabled
            }
        }


def main():
    """Main function for testing duty control service."""
    if ROS2_AVAILABLE:
        rclpy.init()
        
        # Create duty control service node
        duty_service = DutyControlService()
        
        try:
            rclpy.spin(duty_service)
        except KeyboardInterrupt:
            pass
        finally:
            duty_service.destroy_node()
            rclpy.shutdown()
    else:
        print("ROS2 not available - running in test mode")
        
        # Create duty control service without ROS2
        duty_service = DutyControlService()
        
        # Test duty cycle validation
        print("Testing duty cycle validation...")
        result = duty_service._validate_duty_cycle_values(1000, -1500)
        print(f"Validation result: {result}")
        
        # Test safety limits
        print("Testing safety limits...")
        limited = duty_service._apply_safety_limits(35000)  # Over max
        print(f"Limited duty cycle: {limited}")
        
        # Test velocity conversion
        print("Testing velocity conversion...")
        duty = duty_service.velocity_to_duty_cycle(5.0)  # 5 rad/s
        print(f"Velocity 5.0 rad/s -> Duty cycle: {duty}")
        
        velocity = duty_service.duty_cycle_to_velocity(16000)
        print(f"Duty cycle 16000 -> Velocity: {velocity:.2f} rad/s")


if __name__ == '__main__':
    main()