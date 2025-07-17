#!/usr/bin/env python3
"""
Motion Configuration Service for BasicMicro Driver

This module provides ROS2 services for runtime configuration of motion strategies
and parameters for the BasicMicro hardware interface.
"""

from typing import Dict, Any, Optional, Callable

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import SetParametersResult
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    # Mock ROS2 components for testing
    class Node:
        def __init__(self, name):
            self.name = name
        def get_logger(self):
            import logging
            return logging.getLogger(self.name)
        def declare_parameter(self, name, default_value):
            pass
        def create_service(self, srv_type, name, callback):
            pass
        def add_on_set_parameters_callback(self, callback):
            pass
        def set_parameters(self, params):
            pass
        def get_parameter(self, name):
            class MockParam:
                def __init__(self, value):
                    self.value = value
            return MockParam("mock_value")
    
    class Parameter:
        class Type:
            STRING = 'string'
            INTEGER = 'integer'
        def __init__(self, name, type_val, value):
            self.name = name
            self.type = type_val
            self.value = value
    
    class rclpy:
        class parameter:
            class SetParametersResult:
                def __init__(self):
                    self.successful = True
                    self.reason = ""
        @staticmethod
        def init(args=None):
            pass
        @staticmethod
        def shutdown():
            pass
        @staticmethod
        def spin(node):
            pass
        @staticmethod
        def ok():
            return True

try:
    from basicmicro_driver.srv import SetMotionStrategy, SetMotionParameters
except ImportError:
    # Fallback for testing - create mock service types
    class SetMotionStrategy:
        class Request:
            strategy: str = ""
        class Response:
            success: bool = False
            message: str = ""
    
    class SetMotionParameters:
        class Request:
            default_acceleration: int = 0
            max_speed: int = 0
            buffer_depth: int = 0
        class Response:
            success: bool = False
            message: str = ""

try:
    from basicmicro_driver.hardware_interface import MotionStrategy, BasicmicroHardwareInterface
except ImportError:
    # Fallback for relative import in package context
    from .hardware_interface import MotionStrategy, BasicmicroHardwareInterface


class MotionConfigurationService(Node if ROS2_AVAILABLE else object):
    """
    ROS2 service node for runtime motion configuration.
    
    Provides services for changing motion strategies and parameters at runtime,
    with validation and persistence across restarts.
    """
    
    # Valid motion strategy strings
    VALID_STRATEGIES = {
        'duty': MotionStrategy.DUTY,
        'duty_accel': MotionStrategy.DUTY_ACCEL,
        'speed': MotionStrategy.SPEED,
        'speed_accel': MotionStrategy.SPEED_ACCEL
    }
    
    # Parameter validation ranges
    PARAM_LIMITS = {
        'default_acceleration': (100, 100000),  # encoder counts/s²
        'max_speed': (100, 50000),              # encoder counts/s
        'buffer_depth': (1, 32)                 # buffer commands
    }

    def __init__(self, hardware_interface: Optional[BasicmicroHardwareInterface] = None):
        """
        Initialize the motion configuration service.
        
        Args:
            hardware_interface: Reference to the hardware interface for direct parameter updates
        """
        if ROS2_AVAILABLE:
            super().__init__('motion_config_service')
        
        # Store reference to hardware interface
        self.hardware_interface = hardware_interface
        
        # Initialize ROS2 parameters with current values
        if ROS2_AVAILABLE:
            self._initialize_parameters()
        
        # Create service servers
        if ROS2_AVAILABLE:
            self.set_strategy_service = self.create_service(
                SetMotionStrategy,
                'set_motion_strategy',
                self._set_motion_strategy_callback
            )
            
            self.set_parameters_service = self.create_service(
                SetMotionParameters,
                'set_motion_parameters',
                self._set_motion_parameters_callback
            )
            
            # Parameter change callback
            self.add_on_set_parameters_callback(self._parameters_callback)
            
            self.get_logger().info("Motion configuration service initialized")
            self.get_logger().info(f"Available strategies: {list(self.VALID_STRATEGIES.keys())}")

    def _initialize_parameters(self) -> None:
        """Initialize ROS2 parameters with default values"""
        try:
            # Motion strategy parameter
            self.declare_parameter('motion_strategy', 'speed_accel')
            
            # Motion control parameters
            self.declare_parameter('default_acceleration', 1000)
            self.declare_parameter('max_speed', 5000)
            self.declare_parameter('buffer_depth', 4)
            
            # If hardware interface is available, sync with its current values
            if self.hardware_interface:
                self._sync_parameters_from_hardware()
                
        except Exception as e:
            self.get_logger().error(f"Failed to initialize parameters: {e}")

    def _sync_parameters_from_hardware(self) -> None:
        """Sync ROS2 parameters with current hardware interface values"""
        try:
            # Update parameters to match hardware interface
            self.set_parameters([
                Parameter('motion_strategy', Parameter.Type.STRING, 
                         self.hardware_interface.motion_strategy.value),
                Parameter('default_acceleration', Parameter.Type.INTEGER,
                         self.hardware_interface.default_acceleration),
                Parameter('buffer_depth', Parameter.Type.INTEGER,
                         self.hardware_interface.buffer_depth)
            ])
            
            self.get_logger().info("Parameters synced with hardware interface")
            
        except Exception as e:
            self.get_logger().error(f"Failed to sync parameters: {e}")

    def _set_motion_strategy_callback(self, request: SetMotionStrategy.Request, 
                                    response: SetMotionStrategy.Response) -> SetMotionStrategy.Response:
        """
        Service callback for setting motion strategy.
        
        Args:
            request: Service request with strategy name
            response: Service response to populate
            
        Returns:
            Service response with success status and message
        """
        try:
            strategy_name = request.strategy.lower().strip()
            
            # Validate strategy
            if strategy_name not in self.VALID_STRATEGIES:
                response.success = False
                response.message = f"Invalid strategy '{strategy_name}'. Valid strategies: {list(self.VALID_STRATEGIES.keys())}"
                self.get_logger().warning(response.message)
                return response
            
            # Get strategy enum
            new_strategy = self.VALID_STRATEGIES[strategy_name]
            
            # Update hardware interface if available
            if self.hardware_interface:
                old_strategy = self.hardware_interface.motion_strategy
                self.hardware_interface.motion_strategy = new_strategy
                self.get_logger().info(f"Hardware motion strategy changed: {old_strategy.value} -> {new_strategy.value}")
            
            # Update ROS2 parameter
            self.set_parameters([
                Parameter('motion_strategy', Parameter.Type.STRING, strategy_name)
            ])
            
            response.success = True
            response.message = f"Motion strategy set to '{strategy_name}'"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to set motion strategy: {e}"
            self.get_logger().error(response.message)
        
        return response

    def _set_motion_parameters_callback(self, request: SetMotionParameters.Request,
                                      response: SetMotionParameters.Response) -> SetMotionParameters.Response:
        """
        Service callback for setting motion parameters.
        
        Args:
            request: Service request with parameter values
            response: Service response to populate
            
        Returns:
            Service response with success status and message
        """
        try:
            # Validate parameters
            validation_result = self._validate_motion_parameters(
                request.default_acceleration,
                request.max_speed,
                request.buffer_depth
            )
            
            if not validation_result['valid']:
                response.success = False
                response.message = validation_result['message']
                self.get_logger().warning(response.message)
                return response
            
            # Update hardware interface if available
            if self.hardware_interface:
                old_accel = self.hardware_interface.default_acceleration
                old_depth = self.hardware_interface.buffer_depth
                
                self.hardware_interface.default_acceleration = request.default_acceleration
                self.hardware_interface.buffer_depth = request.buffer_depth
                
                self.get_logger().info(f"Hardware parameters updated: accel {old_accel} -> {request.default_acceleration}, "
                                     f"buffer_depth {old_depth} -> {request.buffer_depth}")
            
            # Update ROS2 parameters
            self.set_parameters([
                Parameter('default_acceleration', Parameter.Type.INTEGER, request.default_acceleration),
                Parameter('max_speed', Parameter.Type.INTEGER, request.max_speed),
                Parameter('buffer_depth', Parameter.Type.INTEGER, request.buffer_depth)
            ])
            
            response.success = True
            response.message = (f"Motion parameters updated: acceleration={request.default_acceleration}, "
                              f"max_speed={request.max_speed}, buffer_depth={request.buffer_depth}")
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to set motion parameters: {e}"
            self.get_logger().error(response.message)
        
        return response

    def _validate_motion_parameters(self, acceleration: int, max_speed: int, 
                                  buffer_depth: int) -> Dict[str, Any]:
        """
        Validate motion parameter values.
        
        Args:
            acceleration: Default acceleration value
            max_speed: Maximum speed value  
            buffer_depth: Buffer depth value
            
        Returns:
            Dictionary with validation result and message
        """
        # Check acceleration limits
        accel_min, accel_max = self.PARAM_LIMITS['default_acceleration']
        if not (accel_min <= acceleration <= accel_max):
            return {
                'valid': False,
                'message': f"Invalid acceleration {acceleration}. Must be between {accel_min} and {accel_max}"
            }
        
        # Check max speed limits
        speed_min, speed_max = self.PARAM_LIMITS['max_speed']
        if not (speed_min <= max_speed <= speed_max):
            return {
                'valid': False,
                'message': f"Invalid max_speed {max_speed}. Must be between {speed_min} and {speed_max}"
            }
        
        # Check buffer depth limits
        depth_min, depth_max = self.PARAM_LIMITS['buffer_depth']
        if not (depth_min <= buffer_depth <= depth_max):
            return {
                'valid': False,
                'message': f"Invalid buffer_depth {buffer_depth}. Must be between {depth_min} and {depth_max}"
            }
        
        return {
            'valid': True,
            'message': "All parameters valid"
        }

    def _parameters_callback(self, params: list) -> SetParametersResult:
        """
        Handle ROS2 parameter changes.
        
        Args:
            params: List of parameter changes
            
        Returns:
            Result indicating success/failure of parameter changes
        """
        result = SetParametersResult()
        result.successful = True
        
        try:
            for param in params:
                if param.name == 'motion_strategy':
                    # Validate and update motion strategy
                    strategy_name = param.value
                    if strategy_name not in self.VALID_STRATEGIES:
                        result.successful = False
                        result.reason = f"Invalid motion strategy: {strategy_name}"
                        break
                    
                    if self.hardware_interface:
                        new_strategy = self.VALID_STRATEGIES[strategy_name]
                        self.hardware_interface.motion_strategy = new_strategy
                        self.get_logger().info(f"Motion strategy updated via parameter: {strategy_name}")
                
                elif param.name in ['default_acceleration', 'max_speed', 'buffer_depth']:
                    # Validate parameter ranges
                    if param.name in self.PARAM_LIMITS:
                        min_val, max_val = self.PARAM_LIMITS[param.name]
                        if not (min_val <= param.value <= max_val):
                            result.successful = False
                            result.reason = f"Parameter {param.name} value {param.value} out of range [{min_val}, {max_val}]"
                            break
                    
                    # Update hardware interface
                    if self.hardware_interface:
                        if param.name == 'default_acceleration':
                            self.hardware_interface.default_acceleration = param.value
                        elif param.name == 'buffer_depth':
                            self.hardware_interface.buffer_depth = param.value
                        
                        self.get_logger().info(f"Parameter {param.name} updated via parameter server: {param.value}")
                
        except Exception as e:
            result.successful = False
            result.reason = f"Parameter update failed: {e}"
            self.get_logger().error(result.reason)
        
        return result

    def get_current_configuration(self) -> Dict[str, Any]:
        """
        Get current motion configuration.
        
        Returns:
            Dictionary with current configuration values
        """
        try:
            config = {
                'motion_strategy': self.get_parameter('motion_strategy').value,
                'default_acceleration': self.get_parameter('default_acceleration').value,
                'max_speed': self.get_parameter('max_speed').value,
                'buffer_depth': self.get_parameter('buffer_depth').value
            }
            
            # Add hardware interface values if available
            if self.hardware_interface:
                config['hardware_motion_strategy'] = self.hardware_interface.motion_strategy.value
                config['hardware_default_acceleration'] = self.hardware_interface.default_acceleration
                config['hardware_buffer_depth'] = self.hardware_interface.buffer_depth
            
            return config
            
        except Exception as e:
            self.get_logger().error(f"Failed to get current configuration: {e}")
            return {}


def main(args=None):
    """Main entry point for the motion configuration service node"""
    rclpy.init(args=args)
    
    try:
        # Create motion configuration service node
        motion_config_service = MotionConfigurationService()
        
        motion_config_service.get_logger().info("Motion configuration service started")
        
        # Spin the node
        rclpy.spin(motion_config_service)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Motion configuration service failed: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()