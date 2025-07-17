#!/usr/bin/env python3
"""
Trajectory Execution Service for Basicmicro Driver

Implements advanced trajectory execution using buffered commands for smooth motion profiles.
Supports both relative distance and absolute position trajectories with mixed execution.

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
            
        def destroy_node(self):
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
from typing import Tuple, Optional, Dict, Any, List
import time

# ROS2 service and message imports
try:
    from basicmicro_driver.srv import ExecuteTrajectory, ExecutePositionSequence
    from basicmicro_driver.msg import TrajectoryPoint, PositionPoint
except ImportError:
    # Mock service types for testing without ROS2
    class ExecuteTrajectory:
        class Request:
            def __init__(self):
                self.trajectory_type = "distance"
                self.trajectory_points = []
        
        class Response:
            def __init__(self):
                self.success = False
                self.message = ""
                self.total_commands_sent = 0
    
    class ExecutePositionSequence:
        class Request:
            def __init__(self):
                self.position_points = []
        
        class Response:
            def __init__(self):
                self.success = False
                self.message = ""
                self.total_commands_sent = 0
    
    class TrajectoryPoint:
        def __init__(self):
            self.command_type = "distance"
            self.left_distance = 0.0
            self.right_distance = 0.0
            self.left_position = 0.0
            self.right_position = 0.0
            self.speed = 1.0
            self.acceleration = 1.0
            self.deceleration = 1.0
            self.duration = 0.0
    
    class PositionPoint:
        def __init__(self):
            self.left_position = 0.0
            self.right_position = 0.0
            self.max_speed = 1.0
            self.acceleration = 1.0
            self.deceleration = 1.0

# Import existing components
try:
    from basicmicro_driver.unit_converter import UnitConverter
except ImportError:
    # Fallback for relative import in package context
    from .unit_converter import UnitConverter


class TrajectoryService(Node if ROS2_AVAILABLE else object):
    """
    Service for advanced trajectory execution with buffered commands.
    
    Provides services for:
    - Mixed trajectory execution (distance and position commands)
    - Position sequence execution for servo applications
    - Trajectory validation and optimization
    - Buffer management for smooth motion profiles
    """
    
    def __init__(self, hardware_interface=None):
        """
        Initialize trajectory execution service.
        
        Args:
            hardware_interface: Reference to hardware interface for direct control
        """
        if ROS2_AVAILABLE:
            super().__init__('trajectory_service')
        
        # Hardware interface reference
        self.hardware_interface = hardware_interface
        
        # Initialize unit converter (will get parameters from hardware interface)
        self._init_unit_converter()
        
        # Trajectory execution configuration
        self.max_buffer_size = 32  # Maximum commands per motor channel
        self.min_buffer_available = 4  # Minimum buffer slots to keep available
        self.trajectory_timeout = 30.0  # Maximum trajectory execution time (seconds)
        
        # Trajectory optimization parameters
        self.min_segment_distance = 0.001  # Minimum segment distance (meters)
        self.min_segment_angle = 0.001  # Minimum segment angle (radians)
        self.velocity_smoothing_factor = 0.9  # For velocity profile smoothing
        
        # Declare parameters for trajectory configuration
        self._declare_trajectory_parameters()
        
        # Create service servers
        self.execute_trajectory_service = self.create_service(
            ExecuteTrajectory,
            '~/execute_trajectory',
            self._execute_trajectory_callback
        )
        
        self.execute_position_sequence_service = self.create_service(
            ExecutePositionSequence,
            '~/execute_position_sequence', 
            self._execute_position_sequence_callback
        )
        
        # Initialize buffer monitoring
        self.last_buffer_status = {'left': 0xFF, 'right': 0xFF}  # 0xFF = empty
        
        if ROS2_AVAILABLE:
            self.get_logger().info("Trajectory execution service initialized")
        else:
            print("Trajectory execution service initialized (ROS2 mock mode)")
    
    def _init_unit_converter(self):
        """Initialize unit converter with hardware interface parameters."""
        if self.hardware_interface:
            # Get physical parameters from hardware interface
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
    
    def _declare_trajectory_parameters(self):
        """Declare ROS2 parameters for trajectory configuration."""
        self.declare_parameter('trajectory.max_buffer_size', self.max_buffer_size)
        self.declare_parameter('trajectory.min_buffer_available', self.min_buffer_available)
        self.declare_parameter('trajectory.timeout', self.trajectory_timeout)
        self.declare_parameter('trajectory.min_segment_distance', self.min_segment_distance)
        self.declare_parameter('trajectory.min_segment_angle', self.min_segment_angle)
        self.declare_parameter('trajectory.velocity_smoothing', self.velocity_smoothing_factor)
        
        # Motion limits for trajectory validation
        self.declare_parameter('trajectory.max_speed', 10.0)  # m/s
        self.declare_parameter('trajectory.max_acceleration', 50.0)  # m/s²
        self.declare_parameter('trajectory.max_distance_per_segment', 100.0)  # meters
        self.declare_parameter('trajectory.max_angle_per_segment', 100.0)  # radians
        
        # Load current parameter values
        self._load_trajectory_parameters()
    
    def _load_trajectory_parameters(self):
        """Load trajectory parameters from ROS2 parameter server."""
        self.max_buffer_size = self.get_parameter('trajectory.max_buffer_size').value
        self.min_buffer_available = self.get_parameter('trajectory.min_buffer_available').value
        self.trajectory_timeout = self.get_parameter('trajectory.timeout').value
        self.min_segment_distance = self.get_parameter('trajectory.min_segment_distance').value
        self.min_segment_angle = self.get_parameter('trajectory.min_segment_angle').value
        self.velocity_smoothing_factor = self.get_parameter('trajectory.velocity_smoothing').value
        
        # Motion limits
        self.max_speed = self.get_parameter('trajectory.max_speed').value
        self.max_acceleration = self.get_parameter('trajectory.max_acceleration').value
        self.max_distance_per_segment = self.get_parameter('trajectory.max_distance_per_segment').value
        self.max_angle_per_segment = self.get_parameter('trajectory.max_angle_per_segment').value
    
    def _execute_trajectory_callback(self, request, response):
        """
        Handle trajectory execution service requests.
        
        Args:
            request: ExecuteTrajectory request
            response: ExecuteTrajectory response
            
        Returns:
            ExecuteTrajectory response with result
        """
        try:
            # Validate trajectory request
            validation_result = self._validate_trajectory_request(request)
            if not validation_result['valid']:
                response.success = False
                response.message = validation_result['message']
                response.total_commands_sent = 0
                return response
            
            # Check buffer availability
            buffer_check = self._check_buffer_availability(len(request.trajectory_points))
            if not buffer_check['available']:
                response.success = False
                response.message = f"Insufficient buffer space: {buffer_check['message']}"
                response.total_commands_sent = 0
                return response
            
            # Optimize trajectory for smooth execution
            optimized_points = self._optimize_trajectory(request.trajectory_points, request.trajectory_type)
            
            # Execute trajectory based on type
            if request.trajectory_type == 'distance':
                execution_result = self._execute_distance_trajectory(optimized_points)
            elif request.trajectory_type == 'position':
                execution_result = self._execute_position_trajectory(optimized_points)
            elif request.trajectory_type == 'mixed':
                execution_result = self._execute_mixed_trajectory(optimized_points)
            else:
                response.success = False
                response.message = f"Unsupported trajectory type: {request.trajectory_type}"
                response.total_commands_sent = 0
                return response
            
            # Return execution results
            response.success = execution_result['success']
            response.message = execution_result['message']
            response.total_commands_sent = execution_result['commands_sent']
            
            if response.success:
                self.get_logger().info(f"Trajectory executed: {response.total_commands_sent} commands sent")
            else:
                self.get_logger().error(f"Trajectory execution failed: {response.message}")
                
            return response
            
        except Exception as e:
            self.get_logger().error(f"Trajectory execution error: {str(e)}")
            response.success = False
            response.message = f"Trajectory execution error: {str(e)}"
            response.total_commands_sent = 0
            return response
    
    def _execute_position_sequence_callback(self, request, response):
        """
        Handle position sequence execution service requests.
        
        Args:
            request: ExecutePositionSequence request
            response: ExecutePositionSequence response
            
        Returns:
            ExecutePositionSequence response with result
        """
        try:
            # Validate position sequence request
            validation_result = self._validate_position_sequence_request(request)
            if not validation_result['valid']:
                response.success = False
                response.message = validation_result['message']
                response.total_commands_sent = 0
                return response
            
            # Check buffer availability
            buffer_check = self._check_buffer_availability(len(request.position_points))
            if not buffer_check['available']:
                response.success = False
                response.message = f"Insufficient buffer space: {buffer_check['message']}"
                response.total_commands_sent = 0
                return response
            
            # Convert position points to trajectory points for unified processing
            trajectory_points = self._convert_position_points_to_trajectory(request.position_points)
            
            # Optimize position sequence
            optimized_points = self._optimize_trajectory(trajectory_points, 'position')
            
            # Execute position sequence
            execution_result = self._execute_position_trajectory(optimized_points)
            
            # Return execution results
            response.success = execution_result['success']
            response.message = execution_result['message']
            response.total_commands_sent = execution_result['commands_sent']
            
            if response.success:
                self.get_logger().info(f"Position sequence executed: {response.total_commands_sent} commands sent")
            else:
                self.get_logger().error(f"Position sequence execution failed: {response.message}")
                
            return response
            
        except Exception as e:
            self.get_logger().error(f"Position sequence execution error: {str(e)}")
            response.success = False
            response.message = f"Position sequence execution error: {str(e)}"
            response.total_commands_sent = 0
            return response
    
    def _validate_trajectory_request(self, request) -> Dict[str, Any]:
        """
        Validate trajectory execution request parameters.
        
        Args:
            request: ExecuteTrajectory request
            
        Returns:
            Dictionary with validation result
        """
        # Check trajectory type
        valid_types = ['distance', 'position', 'mixed']
        if request.trajectory_type not in valid_types:
            return {
                'valid': False,
                'message': f"Invalid trajectory type '{request.trajectory_type}'. Must be one of: {valid_types}"
            }
        
        # Check trajectory has points
        if not request.trajectory_points:
            return {
                'valid': False,
                'message': "Trajectory must contain at least one point"
            }
        
        # Check trajectory length
        if len(request.trajectory_points) > self.max_buffer_size:
            return {
                'valid': False,
                'message': f"Trajectory too long: {len(request.trajectory_points)} points exceeds buffer size {self.max_buffer_size}"
            }
        
        # Validate individual trajectory points
        for i, point in enumerate(request.trajectory_points):
            point_validation = self._validate_trajectory_point(point, request.trajectory_type)
            if not point_validation['valid']:
                return {
                    'valid': False,
                    'message': f"Point {i}: {point_validation['message']}"
                }
        
        return {'valid': True, 'message': 'Trajectory validation passed'}
    
    def _validate_trajectory_point(self, point: TrajectoryPoint, trajectory_type: str) -> Dict[str, Any]:
        """
        Validate individual trajectory point parameters.
        
        Args:
            point: TrajectoryPoint to validate
            trajectory_type: Expected trajectory type for validation context
            
        Returns:
            Dictionary with validation result
        """
        # Check command type
        valid_command_types = ['distance', 'position']
        if point.command_type not in valid_command_types:
            return {
                'valid': False,
                'message': f"Invalid command type '{point.command_type}'. Must be 'distance' or 'position'"
            }
        
        # For strict trajectory types, verify command type matches
        if trajectory_type != 'mixed' and point.command_type != trajectory_type:
            return {
                'valid': False,
                'message': f"Command type '{point.command_type}' doesn't match trajectory type '{trajectory_type}'"
            }
        
        # Validate common parameters
        if point.speed <= 0 or point.speed > self.max_speed:
            return {
                'valid': False,
                'message': f"Invalid speed {point.speed}. Must be > 0 and <= {self.max_speed} m/s"
            }
        
        if point.acceleration <= 0 or point.acceleration > self.max_acceleration:
            return {
                'valid': False,
                'message': f"Invalid acceleration {point.acceleration}. Must be > 0 and <= {self.max_acceleration} m/s²"
            }
        
        if point.duration < 0:
            return {
                'valid': False,
                'message': f"Invalid duration {point.duration}. Must be >= 0 seconds"
            }
        
        # Validate distance command specific parameters
        if point.command_type == 'distance':
            if abs(point.left_distance) > self.max_distance_per_segment:
                return {
                    'valid': False,
                    'message': f"Left distance {point.left_distance} exceeds maximum {self.max_distance_per_segment} m"
                }
            
            if abs(point.right_distance) > self.max_distance_per_segment:
                return {
                    'valid': False,
                    'message': f"Right distance {point.right_distance} exceeds maximum {self.max_distance_per_segment} m"
                }
        
        # Validate position command specific parameters
        elif point.command_type == 'position':
            if abs(point.left_position) > self.max_angle_per_segment:
                return {
                    'valid': False,
                    'message': f"Left position {point.left_position} exceeds maximum {self.max_angle_per_segment} rad"
                }
            
            if abs(point.right_position) > self.max_angle_per_segment:
                return {
                    'valid': False,
                    'message': f"Right position {point.right_position} exceeds maximum {self.max_angle_per_segment} rad"
                }
            
            if point.deceleration <= 0 or point.deceleration > self.max_acceleration:
                return {
                    'valid': False,
                    'message': f"Invalid deceleration {point.deceleration}. Must be > 0 and <= {self.max_acceleration} m/s²"
                }
        
        return {'valid': True, 'message': 'Point validation passed'}
    
    def _validate_position_sequence_request(self, request) -> Dict[str, Any]:
        """
        Validate position sequence execution request parameters.
        
        Args:
            request: ExecutePositionSequence request
            
        Returns:
            Dictionary with validation result
        """
        # Check sequence has points
        if not request.position_points:
            return {
                'valid': False,
                'message': "Position sequence must contain at least one point"
            }
        
        # Check sequence length
        if len(request.position_points) > self.max_buffer_size:
            return {
                'valid': False,
                'message': f"Sequence too long: {len(request.position_points)} points exceeds buffer size {self.max_buffer_size}"
            }
        
        # Validate individual position points
        for i, point in enumerate(request.position_points):
            point_validation = self._validate_position_point(point)
            if not point_validation['valid']:
                return {
                    'valid': False,
                    'message': f"Point {i}: {point_validation['message']}"
                }
        
        return {'valid': True, 'message': 'Position sequence validation passed'}
    
    def _validate_position_point(self, point: PositionPoint) -> Dict[str, Any]:
        """
        Validate individual position point parameters.
        
        Args:
            point: PositionPoint to validate
            
        Returns:
            Dictionary with validation result
        """
        # Validate position values
        if abs(point.left_position) > self.max_angle_per_segment:
            return {
                'valid': False,
                'message': f"Left position {point.left_position} exceeds maximum {self.max_angle_per_segment} rad"
            }
        
        if abs(point.right_position) > self.max_angle_per_segment:
            return {
                'valid': False,
                'message': f"Right position {point.right_position} exceeds maximum {self.max_angle_per_segment} rad"
            }
        
        # Validate motion parameters
        if point.max_speed <= 0 or point.max_speed > self.max_speed:
            return {
                'valid': False,
                'message': f"Invalid max speed {point.max_speed}. Must be > 0 and <= {self.max_speed} m/s"
            }
        
        if point.acceleration <= 0 or point.acceleration > self.max_acceleration:
            return {
                'valid': False,
                'message': f"Invalid acceleration {point.acceleration}. Must be > 0 and <= {self.max_acceleration} m/s²"
            }
        
        if point.deceleration <= 0 or point.deceleration > self.max_acceleration:
            return {
                'valid': False,
                'message': f"Invalid deceleration {point.deceleration}. Must be > 0 and <= {self.max_acceleration} m/s²"
            }
        
        return {'valid': True, 'message': 'Position point validation passed'}
    
    def _check_buffer_availability(self, required_slots: int) -> Dict[str, Any]:
        """
        Check if sufficient buffer space is available for trajectory.
        
        Args:
            required_slots: Number of buffer slots required
            
        Returns:
            Dictionary with availability status
        """
        if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
            # For testing without hardware interface
            return {
                'available': True,
                'message': 'Buffer availability check skipped (no hardware interface)'
            }
        
        try:
            # Read current buffer status
            buffer_result = self.hardware_interface.controller.ReadBuffers(
                self.hardware_interface.address
            )
            
            if not buffer_result[0]:
                return {
                    'available': False,
                    'message': 'Failed to read buffer status from controller'
                }
            
            # Interpret buffer status
            raw_status = buffer_result[1]
            if raw_status == 0xFF:
                # Buffer empty
                available_slots = self.max_buffer_size
            elif raw_status == 0:
                # Executing last command, buffer empty
                available_slots = self.max_buffer_size
            else:
                # raw_status = number of commands in buffer (1-32)
                available_slots = self.max_buffer_size - raw_status
            
            # Check if we have enough slots with safety margin
            required_with_margin = required_slots + self.min_buffer_available
            if available_slots < required_with_margin:
                return {
                    'available': False,
                    'message': f"Need {required_with_margin} slots, only {available_slots} available"
                }
            
            return {
                'available': True,
                'message': f"Buffer available: {available_slots} slots free"
            }
            
        except Exception as e:
            return {
                'available': False,
                'message': f"Buffer status check failed: {str(e)}"
            }
    
    def _optimize_trajectory(self, points: List[TrajectoryPoint], trajectory_type: str) -> List[TrajectoryPoint]:
        """
        Optimize trajectory for smooth execution.
        
        Args:
            points: List of trajectory points to optimize
            trajectory_type: Type of trajectory for optimization context
            
        Returns:
            List of optimized trajectory points
        """
        if len(points) <= 1:
            return points
        
        optimized = []
        
        for i, point in enumerate(points):
            # Create optimized point (copy original)
            opt_point = TrajectoryPoint()
            opt_point.command_type = point.command_type
            opt_point.left_distance = point.left_distance
            opt_point.right_distance = point.right_distance
            opt_point.left_position = point.left_position
            opt_point.right_position = point.right_position
            opt_point.deceleration = point.deceleration
            opt_point.speed = point.speed
            opt_point.acceleration = point.acceleration
            opt_point.duration = point.duration
            
            # Apply velocity smoothing for multi-segment trajectories
            if i > 0 and i < len(points) - 1:
                # Smooth velocity based on adjacent segments
                prev_speed = points[i-1].speed
                next_speed = points[i+1].speed
                
                # Apply smoothing factor
                smoothed_speed = (
                    (1 - self.velocity_smoothing_factor) * point.speed +
                    self.velocity_smoothing_factor * (prev_speed + next_speed) / 2
                )
                
                # Ensure smoothed speed doesn't exceed limits
                opt_point.speed = min(smoothed_speed, self.max_speed)
            
            # Remove tiny segments that could cause control issues
            if point.command_type == 'distance':
                if (abs(point.left_distance) < self.min_segment_distance and 
                    abs(point.right_distance) < self.min_segment_distance):
                    # Skip tiny distance segments
                    continue
            elif point.command_type == 'position':
                if (abs(point.left_position) < self.min_segment_angle and 
                    abs(point.right_position) < self.min_segment_angle):
                    # Skip tiny position segments
                    continue
            
            optimized.append(opt_point)
        
        return optimized
    
    def _convert_position_points_to_trajectory(self, position_points: List[PositionPoint]) -> List[TrajectoryPoint]:
        """
        Convert PositionPoint sequence to TrajectoryPoint sequence.
        
        Args:
            position_points: List of position points to convert
            
        Returns:
            List of trajectory points
        """
        trajectory_points = []
        
        for point in position_points:
            traj_point = TrajectoryPoint()
            traj_point.command_type = 'position'
            traj_point.left_position = point.left_position
            traj_point.right_position = point.right_position
            traj_point.speed = point.max_speed
            traj_point.acceleration = point.acceleration
            traj_point.deceleration = point.deceleration
            traj_point.duration = 0.0  # No timing constraint from position points
            
            trajectory_points.append(traj_point)
        
        return trajectory_points
    
    def _execute_distance_trajectory(self, points: List[TrajectoryPoint]) -> Dict[str, Any]:
        """
        Execute distance-based trajectory using SpeedAccelDistanceM1M2.
        
        Args:
            points: List of distance trajectory points
            
        Returns:
            Dictionary with execution result
        """
        if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
            # For testing without hardware interface
            return {
                'success': True,
                'message': f'Distance trajectory simulated: {len(points)} points',
                'commands_sent': len(points)
            }
        
        commands_sent = 0
        
        try:
            for i, point in enumerate(points):
                # Convert units to encoder counts
                left_distance_counts = self.unit_converter.distance_to_counts(point.left_distance)
                right_distance_counts = self.unit_converter.distance_to_counts(point.right_distance)
                speed_counts = self.unit_converter.speed_to_counts_per_sec(point.speed)
                accel_counts = self.unit_converter.acceleration_to_counts_per_sec2(point.acceleration)
                
                # Determine if this command should be buffered
                buffer_flag = 1 if i < len(points) - 1 else 0  # Buffer all except last
                
                # Execute distance command
                result = self.hardware_interface.controller.SpeedAccelDistanceM1M2(
                    self.hardware_interface.address,
                    accel_counts,
                    speed_counts, left_distance_counts,
                    speed_counts, right_distance_counts,
                    buffer_flag
                )
                
                if result:
                    commands_sent += 1
                    self.get_logger().debug(f"Distance command {i} sent: left={point.left_distance:.3f}m, right={point.right_distance:.3f}m")
                else:
                    self.get_logger().error(f"Distance command {i} failed")
                    return {
                        'success': False,
                        'message': f'Command {i} failed to execute',
                        'commands_sent': commands_sent
                    }
            
            return {
                'success': True,
                'message': f'Distance trajectory completed: {commands_sent} commands sent',
                'commands_sent': commands_sent
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'Distance trajectory execution failed: {str(e)}',
                'commands_sent': commands_sent
            }
    
    def _execute_position_trajectory(self, points: List[TrajectoryPoint]) -> Dict[str, Any]:
        """
        Execute position-based trajectory using SpeedAccelDeccelPositionM1M2.
        
        Args:
            points: List of position trajectory points
            
        Returns:
            Dictionary with execution result
        """
        if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
            # For testing without hardware interface
            return {
                'success': True,
                'message': f'Position trajectory simulated: {len(points)} points',
                'commands_sent': len(points)
            }
        
        commands_sent = 0
        
        try:
            for i, point in enumerate(points):
                # Convert units to encoder counts
                left_position_counts = self.unit_converter.radians_to_counts(point.left_position)
                right_position_counts = self.unit_converter.radians_to_counts(point.right_position)
                speed_counts = self.unit_converter.speed_to_counts_per_sec(point.speed)
                accel_counts = self.unit_converter.acceleration_to_counts_per_sec2(point.acceleration)
                decel_counts = self.unit_converter.acceleration_to_counts_per_sec2(point.deceleration)
                
                # Determine if this command should be buffered
                buffer_flag = 1 if i < len(points) - 1 else 0  # Buffer all except last
                
                # Execute position command
                result = self.hardware_interface.controller.SpeedAccelDeccelPositionM1M2(
                    self.hardware_interface.address,
                    accel_counts, speed_counts, decel_counts, left_position_counts,
                    accel_counts, speed_counts, decel_counts, right_position_counts,
                    buffer_flag
                )
                
                if result:
                    commands_sent += 1
                    self.get_logger().debug(f"Position command {i} sent: left={point.left_position:.3f}rad, right={point.right_position:.3f}rad")
                else:
                    self.get_logger().error(f"Position command {i} failed")
                    return {
                        'success': False,
                        'message': f'Command {i} failed to execute',
                        'commands_sent': commands_sent
                    }
            
            return {
                'success': True,
                'message': f'Position trajectory completed: {commands_sent} commands sent',
                'commands_sent': commands_sent
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'Position trajectory execution failed: {str(e)}',
                'commands_sent': commands_sent
            }
    
    def _execute_mixed_trajectory(self, points: List[TrajectoryPoint]) -> Dict[str, Any]:
        """
        Execute mixed trajectory with both distance and position commands.
        
        Args:
            points: List of mixed trajectory points
            
        Returns:
            Dictionary with execution result
        """
        if not self.hardware_interface or not hasattr(self.hardware_interface, 'controller'):
            # For testing without hardware interface
            return {
                'success': True,
                'message': f'Mixed trajectory simulated: {len(points)} points',
                'commands_sent': len(points)
            }
        
        commands_sent = 0
        
        try:
            for i, point in enumerate(points):
                # Determine if this command should be buffered
                buffer_flag = 1 if i < len(points) - 1 else 0  # Buffer all except last
                
                if point.command_type == 'distance':
                    # Execute distance command
                    left_distance_counts = self.unit_converter.distance_to_counts(point.left_distance)
                    right_distance_counts = self.unit_converter.distance_to_counts(point.right_distance)
                    speed_counts = self.unit_converter.speed_to_counts_per_sec(point.speed)
                    accel_counts = self.unit_converter.acceleration_to_counts_per_sec2(point.acceleration)
                    
                    result = self.hardware_interface.controller.SpeedAccelDistanceM1M2(
                        self.hardware_interface.address,
                        accel_counts,
                        speed_counts, left_distance_counts,
                        speed_counts, right_distance_counts,
                        buffer_flag
                    )
                    
                elif point.command_type == 'position':
                    # Execute position command
                    left_position_counts = self.unit_converter.radians_to_counts(point.left_position)
                    right_position_counts = self.unit_converter.radians_to_counts(point.right_position)
                    speed_counts = self.unit_converter.speed_to_counts_per_sec(point.speed)
                    accel_counts = self.unit_converter.acceleration_to_counts_per_sec2(point.acceleration)
                    decel_counts = self.unit_converter.acceleration_to_counts_per_sec2(point.deceleration)
                    
                    result = self.hardware_interface.controller.SpeedAccelDeccelPositionM1M2(
                        self.hardware_interface.address,
                        accel_counts, speed_counts, decel_counts, left_position_counts,
                        accel_counts, speed_counts, decel_counts, right_position_counts,
                        buffer_flag
                    )
                
                else:
                    self.get_logger().error(f"Unknown command type in point {i}: {point.command_type}")
                    return {
                        'success': False,
                        'message': f'Unknown command type in point {i}: {point.command_type}',
                        'commands_sent': commands_sent
                    }
                
                if result:
                    commands_sent += 1
                    self.get_logger().debug(f"Mixed command {i} ({point.command_type}) sent successfully")
                else:
                    self.get_logger().error(f"Mixed command {i} ({point.command_type}) failed")
                    return {
                        'success': False,
                        'message': f'Command {i} ({point.command_type}) failed to execute',
                        'commands_sent': commands_sent
                    }
            
            return {
                'success': True,
                'message': f'Mixed trajectory completed: {commands_sent} commands sent',
                'commands_sent': commands_sent
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'Mixed trajectory execution failed: {str(e)}',
                'commands_sent': commands_sent
            }
    
    def get_current_configuration(self) -> Dict[str, Any]:
        """
        Get current trajectory service configuration.
        
        Returns:
            Dictionary with current configuration
        """
        return {
            'max_buffer_size': self.max_buffer_size,
            'min_buffer_available': self.min_buffer_available,
            'trajectory_timeout': self.trajectory_timeout,
            'min_segment_distance': self.min_segment_distance,
            'min_segment_angle': self.min_segment_angle,
            'velocity_smoothing_factor': self.velocity_smoothing_factor,
            'max_speed': self.max_speed,
            'max_acceleration': self.max_acceleration,
            'max_distance_per_segment': self.max_distance_per_segment,
            'max_angle_per_segment': self.max_angle_per_segment
        }


def main(args=None):
    """Main entry point for trajectory service."""
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        
        try:
            # Create trajectory service node
            trajectory_service = TrajectoryService()
            
            # Spin the node
            rclpy.spin(trajectory_service)
            
        except KeyboardInterrupt:
            pass
        finally:
            if 'trajectory_service' in locals():
                trajectory_service.destroy_node()
            rclpy.shutdown()
    else:
        print("ROS2 not available - running in test mode")
        trajectory_service = TrajectoryService()
        print("Trajectory Service created successfully in test mode")


if __name__ == '__main__':
    main()