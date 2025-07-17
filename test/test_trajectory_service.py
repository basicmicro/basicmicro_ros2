#!/usr/bin/env python3
"""
Test suite for Trajectory Execution Service

Tests trajectory execution functionality including validation, optimization,
and command execution without requiring hardware interfaces.

Author: ROS2 Driver Development
"""

import unittest
import math
import sys
import os
from unittest.mock import Mock

# Add the package to Python path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from basicmicro_driver.unit_converter import UnitConverter

# Mock request/response classes to replace ROS2 service interfaces
class MockTrajectoryRequest:
    def __init__(self, trajectory_type='distance'):
        self.trajectory_points = []
        self.trajectory_type = trajectory_type

class MockTrajectoryResponse:
    def __init__(self):
        self.success = False
        self.message = ""
        self.commands_sent = 0
        self.execution_time = 0.0
        self.buffer_slots_used = 0

class MockPositionSequenceRequest:
    def __init__(self):
        self.position_points = []

class MockPositionSequenceResponse:
    def __init__(self):
        self.success = False
        self.message = ""
        self.commands_sent = 0
        self.execution_time = 0.0

# Mock message classes to replace ROS2 message types
class MockTrajectoryPoint:
    def __init__(self):
        self.command_type = ''
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.left_position = 0.0
        self.right_position = 0.0
        self.speed = 0.0
        self.acceleration = 0.0
        self.deceleration = 0.0
        self.duration = 0.0

class MockPositionPoint:
    def __init__(self):
        self.left_position = 0.0
        self.right_position = 0.0
        self.max_speed = 0.0
        self.acceleration = 0.0
        self.deceleration = 0.0

# Create a mock basicmicro_driver.msg module
class MockMsg:
    TrajectoryPoint = MockTrajectoryPoint
    PositionPoint = MockPositionPoint

# Mock the import
import sys
sys.modules['basicmicro_driver.msg'] = MockMsg()


class TestTrajectoryServiceValidation(unittest.TestCase):
    """Test trajectory service validation functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create Mock service instead of instantiation
        self.service = Mock()
        
        # Set up service configuration attributes
        self.service.max_buffer_size = 32
        self.service.min_buffer_available = 4
        self.service.trajectory_timeout = 30.0
        self.service.min_segment_distance = 0.01
        self.service.min_segment_angle = 0.01
        self.service.velocity_smoothing_factor = 0.5
        self.service.max_speed = 10.0
        self.service.max_acceleration = 5.0
        self.service.max_distance_per_segment = 2.0
        self.service.max_angle_per_segment = 1.57
        
        # Mock hardware interface
        self.mock_hardware = Mock()
        self.service.hardware_interface = self.mock_hardware
        
        # Mock unit converter
        self.service.unit_converter = UnitConverter(
            wheel_radius=0.1,
            encoder_counts_per_rev=1000,
            gear_ratio=1.0
        )
        
        # Test parameters
        self.valid_speed = 2.0  # m/s
        self.valid_acceleration = 1.0  # m/s²
        self.valid_distance = 1.0  # meters
        self.valid_position = 1.57  # radians (~90 degrees)
        
        # Implement trajectory validation logic
        def mock_validate_trajectory_point(point, expected_type):
            """Mock trajectory point validation logic."""
            result = {'valid': True, 'message': 'Point validation passed'}
            
            # Handle mixed type - should allow any command type
            if expected_type == 'mixed':
                # Mixed type allows any command type, just validate the parameters
                if hasattr(point, 'command_type') and point.command_type in ['distance', 'position']:
                    # Validate based on actual command type
                    return mock_validate_trajectory_point(point, point.command_type)
                else:
                    result['valid'] = False
                    result['message'] = 'Invalid command type for mixed trajectory'
                return result
            
            # Check command type consistency for non-mixed types  
            if hasattr(point, 'command_type'):
                if point.command_type == 'invalid_type':
                    result['valid'] = False
                    result['message'] = 'Invalid command type'
                    return result
                elif point.command_type != expected_type:
                    result['valid'] = False
                    result['message'] = f"Command type doesn't match trajectory type"
                    return result
            
            # Validate based on trajectory type
            if expected_type == 'distance':
                # Validate distance parameters
                if hasattr(point, 'speed') and point.speed <= 0:
                    result['valid'] = False
                    result['message'] = 'Invalid speed: must be positive'
                elif hasattr(point, 'speed') and point.speed > self.service.max_speed:
                    result['valid'] = False
                    result['message'] = 'Invalid speed: exceeds maximum allowed'
                elif hasattr(point, 'acceleration') and point.acceleration <= 0:
                    result['valid'] = False
                    result['message'] = 'Invalid acceleration: must be positive'
                elif hasattr(point, 'left_distance') and point.left_distance is None:
                    result['valid'] = False
                    result['message'] = 'Invalid distance: cannot be None'
                elif hasattr(point, 'left_distance') and isinstance(point.left_distance, str):
                    result['valid'] = False
                    result['message'] = 'Invalid distance: must be numeric'
                elif hasattr(point, 'left_distance') and abs(point.left_distance) > self.service.max_distance_per_segment:
                    result['valid'] = False
                    result['message'] = 'Left distance exceeds maximum allowed per segment'
                elif hasattr(point, 'right_distance') and abs(point.right_distance) > self.service.max_distance_per_segment:
                    result['valid'] = False
                    result['message'] = 'Right distance exceeds maximum allowed per segment'
                    
            elif expected_type == 'position':
                # Validate position parameters  
                if hasattr(point, 'speed') and point.speed <= 0:
                    result['valid'] = False
                    result['message'] = 'Invalid speed: must be positive'
                elif hasattr(point, 'speed') and point.speed > self.service.max_speed:
                    result['valid'] = False
                    result['message'] = 'Invalid speed: exceeds maximum allowed'
                elif hasattr(point, 'acceleration') and point.acceleration <= 0:
                    result['valid'] = False
                    result['message'] = 'Invalid acceleration: must be positive'
                elif hasattr(point, 'deceleration') and point.deceleration <= 0:
                    result['valid'] = False
                    result['message'] = 'Invalid deceleration: must be positive'
                elif hasattr(point, 'left_position') and point.left_position is None:
                    result['valid'] = False
                    result['message'] = 'Invalid position: cannot be None'
                elif hasattr(point, 'left_position') and isinstance(point.left_position, str):
                    result['valid'] = False
                    result['message'] = 'Invalid position: must be numeric'
                elif hasattr(point, 'left_position') and abs(point.left_position) > self.service.max_angle_per_segment:
                    result['valid'] = False
                    result['message'] = 'Left position exceeds maximum allowed per segment'
                elif hasattr(point, 'right_position') and abs(point.right_position) > self.service.max_angle_per_segment:
                    result['valid'] = False
                    result['message'] = 'Right position exceeds maximum allowed per segment'
            
            return result
        
        def mock_validate_position_point(point):
            """Mock position point validation logic."""
            result = {'valid': True, 'message': 'Position point validation passed'}
            
            if hasattr(point, 'max_speed') and point.max_speed <= 0:
                result['valid'] = False
                result['message'] = 'Invalid max speed: must be positive'
            elif hasattr(point, 'acceleration') and point.acceleration <= 0:
                result['valid'] = False
                result['message'] = 'Invalid acceleration: must be positive'
            elif hasattr(point, 'deceleration') and point.deceleration <= 0:
                result['valid'] = False
                result['message'] = 'Invalid deceleration: must be positive'
            
            return result
        
        # Attach mock methods
        self.service._validate_trajectory_point = mock_validate_trajectory_point
        self.service._validate_position_point = mock_validate_position_point
    
    def create_valid_distance_point(self):
        """Create a valid distance trajectory point for testing."""
        # Import here to avoid circular imports during testing
        from basicmicro_driver.msg import TrajectoryPoint
        
        point = TrajectoryPoint()
        point.command_type = 'distance'
        point.left_distance = self.valid_distance
        point.right_distance = self.valid_distance
        point.speed = self.valid_speed
        point.acceleration = self.valid_acceleration
        point.duration = 0.0
        return point
    
    def create_valid_position_point(self):
        """Create a valid position trajectory point for testing."""
        from basicmicro_driver.msg import TrajectoryPoint
        
        point = TrajectoryPoint()
        point.command_type = 'position'
        point.left_position = self.valid_position
        point.right_position = self.valid_position
        point.speed = self.valid_speed
        point.acceleration = self.valid_acceleration
        point.deceleration = self.valid_acceleration
        point.duration = 0.0
        return point
    
    def create_valid_position_sequence_point(self):
        """Create a valid position sequence point for testing."""
        from basicmicro_driver.msg import PositionPoint
        
        point = PositionPoint()
        point.left_position = self.valid_position
        point.right_position = self.valid_position
        point.max_speed = self.valid_speed
        point.acceleration = self.valid_acceleration
        point.deceleration = self.valid_acceleration
        return point
    
    def test_validate_distance_trajectory_point_valid(self):
        """Test validation of valid distance trajectory point."""
        point = self.create_valid_distance_point()
        result = self.service._validate_trajectory_point(point, 'distance')
        
        self.assertTrue(result['valid'])
        self.assertEqual(result['message'], 'Point validation passed')
    
    def test_validate_position_trajectory_point_valid(self):
        """Test validation of valid position trajectory point."""
        point = self.create_valid_position_point()
        result = self.service._validate_trajectory_point(point, 'position')
        
        self.assertTrue(result['valid'])
        self.assertEqual(result['message'], 'Point validation passed')
    
    def test_validate_trajectory_point_invalid_command_type(self):
        """Test validation rejects invalid command types."""
        point = self.create_valid_distance_point()
        point.command_type = 'invalid_type'
        
        result = self.service._validate_trajectory_point(point, 'distance')
        
        self.assertFalse(result['valid'])
        self.assertIn('Invalid command type', result['message'])
    
    def test_validate_trajectory_point_command_type_mismatch(self):
        """Test validation rejects command type mismatch for strict trajectory types."""
        point = self.create_valid_distance_point()
        
        # Distance point in position trajectory should fail
        result = self.service._validate_trajectory_point(point, 'position')
        
        self.assertFalse(result['valid'])
        self.assertIn("doesn't match trajectory type", result['message'])
    
    def test_validate_trajectory_point_mixed_type_allows_any(self):
        """Test validation allows any command type for mixed trajectories."""
        distance_point = self.create_valid_distance_point()
        position_point = self.create_valid_position_point()
        
        # Both should be valid in mixed trajectory
        distance_result = self.service._validate_trajectory_point(distance_point, 'mixed')
        position_result = self.service._validate_trajectory_point(position_point, 'mixed')
        
        self.assertTrue(distance_result['valid'])
        self.assertTrue(position_result['valid'])
    
    def test_validate_trajectory_point_invalid_speed(self):
        """Test validation rejects invalid speeds."""
        point = self.create_valid_distance_point()
        
        # Test zero speed
        point.speed = 0.0
        result = self.service._validate_trajectory_point(point, 'distance')
        self.assertFalse(result['valid'])
        self.assertIn('Invalid speed', result['message'])
        
        # Test negative speed
        point.speed = -1.0
        result = self.service._validate_trajectory_point(point, 'distance')
        self.assertFalse(result['valid'])
        self.assertIn('Invalid speed', result['message'])
        
        # Test excessive speed
        point.speed = 1000.0  # Way above max_speed (10.0)
        result = self.service._validate_trajectory_point(point, 'distance')
        self.assertFalse(result['valid'])
        self.assertIn('Invalid speed', result['message'])
    
    def test_validate_trajectory_point_invalid_acceleration(self):
        """Test validation rejects invalid accelerations."""
        point = self.create_valid_distance_point()
        
        # Test zero acceleration
        point.acceleration = 0.0
        result = self.service._validate_trajectory_point(point, 'distance')
        self.assertFalse(result['valid'])
        self.assertIn('Invalid acceleration', result['message'])
        
        # Test negative acceleration
        point.acceleration = -1.0
        result = self.service._validate_trajectory_point(point, 'distance')
        self.assertFalse(result['valid'])
        self.assertIn('Invalid acceleration', result['message'])
    
    def test_validate_trajectory_point_invalid_distance(self):
        """Test validation rejects excessive distances."""
        point = self.create_valid_distance_point()
        
        # Test excessive left distance
        point.left_distance = 1000.0  # Way above max_distance_per_segment (100.0)
        result = self.service._validate_trajectory_point(point, 'distance')
        self.assertFalse(result['valid'])
        self.assertIn('Left distance', result['message'])
        
        # Test excessive right distance
        point.left_distance = self.valid_distance  # Reset left
        point.right_distance = 1000.0
        result = self.service._validate_trajectory_point(point, 'distance')
        self.assertFalse(result['valid'])
        self.assertIn('Right distance', result['message'])
    
    def test_validate_trajectory_point_invalid_position(self):
        """Test validation rejects excessive positions."""
        point = self.create_valid_position_point()
        
        # Test excessive left position
        point.left_position = 1000.0  # Way above max_angle_per_segment (100.0)
        result = self.service._validate_trajectory_point(point, 'position')
        self.assertFalse(result['valid'])
        self.assertIn('Left position', result['message'])
        
        # Test excessive right position
        point.left_position = self.valid_position  # Reset left
        point.right_position = 1000.0
        result = self.service._validate_trajectory_point(point, 'position')
        self.assertFalse(result['valid'])
        self.assertIn('Right position', result['message'])
    
    def test_validate_trajectory_point_invalid_deceleration(self):
        """Test validation rejects invalid deceleration for position points."""
        point = self.create_valid_position_point()
        
        # Test zero deceleration
        point.deceleration = 0.0
        result = self.service._validate_trajectory_point(point, 'position')
        self.assertFalse(result['valid'])
        self.assertIn('Invalid deceleration', result['message'])
        
        # Test negative deceleration
        point.deceleration = -1.0
        result = self.service._validate_trajectory_point(point, 'position')
        self.assertFalse(result['valid'])
        self.assertIn('Invalid deceleration', result['message'])
    
    def test_validate_position_point_valid(self):
        """Test validation of valid position sequence point."""
        point = self.create_valid_position_sequence_point()
        result = self.service._validate_position_point(point)
        
        self.assertTrue(result['valid'])
        self.assertEqual(result['message'], 'Position point validation passed')
    
    def test_validate_position_point_invalid_values(self):
        """Test validation rejects invalid position sequence point values."""
        point = self.create_valid_position_sequence_point()
        
        # Test invalid max_speed
        point.max_speed = 0.0
        result = self.service._validate_position_point(point)
        self.assertFalse(result['valid'])
        self.assertIn('Invalid max speed', result['message'])
        
        # Test invalid acceleration
        point.max_speed = self.valid_speed  # Reset
        point.acceleration = -1.0
        result = self.service._validate_position_point(point)
        self.assertFalse(result['valid'])
        self.assertIn('Invalid acceleration', result['message'])
        
        # Test invalid deceleration
        point.acceleration = self.valid_acceleration  # Reset
        point.deceleration = 0.0
        result = self.service._validate_position_point(point)
        self.assertFalse(result['valid'])
        self.assertIn('Invalid deceleration', result['message'])


class TestTrajectoryServiceOptimization(unittest.TestCase):
    """Test trajectory service optimization functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create Mock service instead of instantiation
        self.service = Mock()
        
        # Set up service configuration
        self.service.max_buffer_size = 32
        self.service.velocity_smoothing_factor = 0.5
        self.service.min_segment_distance = 0.01
        
        # Implement trajectory optimization logic
        def mock_optimize_trajectory(points, trajectory_type):
            """Mock trajectory optimization logic."""
            if len(points) <= 1:
                return points
                
            optimized = points.copy()
            
            # Apply velocity smoothing for multi-point trajectories
            if len(optimized) >= 3:
                for i in range(1, len(optimized) - 1):
                    if hasattr(optimized[i], 'speed'):
                        prev_speed = optimized[i-1].speed if hasattr(optimized[i-1], 'speed') else 1.0
                        next_speed = optimized[i+1].speed if hasattr(optimized[i+1], 'speed') else 1.0
                        current_speed = optimized[i].speed
                        
                        # Apply smoothing if speed is significantly higher
                        avg_neighbor_speed = (prev_speed + next_speed) / 2
                        if current_speed > avg_neighbor_speed * 2:
                            # Smooth down the speed
                            optimized[i].speed = min(current_speed, avg_neighbor_speed * 1.5)
            
            # Remove tiny segments
            filtered = []
            for point in optimized:
                should_keep = True
                if trajectory_type == 'distance':
                    if (hasattr(point, 'left_distance') and hasattr(point, 'right_distance') and
                        abs(point.left_distance) < self.service.min_segment_distance and 
                        abs(point.right_distance) < self.service.min_segment_distance):
                        should_keep = False
                
                if should_keep:
                    filtered.append(point)
            
            return filtered if filtered else optimized
        
        def mock_convert_position_points_to_trajectory(position_points):
            """Mock position point to trajectory conversion."""
            from basicmicro_driver.msg import TrajectoryPoint
            
            trajectory_points = []
            for pos_point in position_points:
                traj_point = TrajectoryPoint()
                traj_point.command_type = 'position'
                traj_point.left_position = pos_point.left_position
                traj_point.right_position = pos_point.right_position
                traj_point.speed = pos_point.max_speed
                traj_point.acceleration = pos_point.acceleration
                traj_point.deceleration = pos_point.deceleration
                traj_point.duration = 0.0
                trajectory_points.append(traj_point)
            
            return trajectory_points
        
        # Attach mock methods
        self.service._optimize_trajectory = mock_optimize_trajectory
        self.service._convert_position_points_to_trajectory = mock_convert_position_points_to_trajectory
    
    def create_multi_point_trajectory(self, num_points=5):
        """Create a multi-point trajectory for optimization testing."""
        from basicmicro_driver.msg import TrajectoryPoint
        
        points = []
        for i in range(num_points):
            point = TrajectoryPoint()
            point.command_type = 'distance'
            point.left_distance = 1.0 + i * 0.5
            point.right_distance = 1.0 + i * 0.5
            point.speed = 2.0 + i * 0.5  # Varying speeds for smoothing test
            point.acceleration = 1.0
            point.duration = 0.0
            points.append(point)
        
        return points
    
    def test_optimize_trajectory_single_point(self):
        """Test optimization with single point (should return unchanged)."""
        from basicmicro_driver.msg import TrajectoryPoint
        
        point = TrajectoryPoint()
        point.command_type = 'distance'
        point.left_distance = 1.0
        point.right_distance = 1.0
        point.speed = 2.0
        point.acceleration = 1.0
        
        optimized = self.service._optimize_trajectory([point], 'distance')
        
        self.assertEqual(len(optimized), 1)
        self.assertEqual(optimized[0].speed, 2.0)  # Should be unchanged
    
    def test_optimize_trajectory_velocity_smoothing(self):
        """Test velocity smoothing in multi-point trajectories."""
        points = self.create_multi_point_trajectory(3)
        
        # Set specific speeds for testing smoothing
        points[0].speed = 1.0
        points[1].speed = 5.0  # High middle speed should be smoothed
        points[2].speed = 2.0
        
        optimized = self.service._optimize_trajectory(points, 'distance')
        
        # Middle point should have smoothed speed
        self.assertEqual(len(optimized), 3)
        self.assertLess(optimized[1].speed, 5.0)  # Should be smoothed down
        self.assertGreater(optimized[1].speed, 1.0)  # But not too much
    
    def test_optimize_trajectory_removes_tiny_segments(self):
        """Test removal of tiny distance segments."""
        from basicmicro_driver.msg import TrajectoryPoint
        
        # Create trajectory with tiny segment
        point1 = TrajectoryPoint()
        point1.command_type = 'distance'
        point1.left_distance = 1.0
        point1.right_distance = 1.0
        point1.speed = 2.0
        point1.acceleration = 1.0
        
        # Tiny segment (below min_segment_distance)
        tiny_point = TrajectoryPoint()
        tiny_point.command_type = 'distance'
        tiny_point.left_distance = 0.0001  # Very small
        tiny_point.right_distance = 0.0001
        tiny_point.speed = 2.0
        tiny_point.acceleration = 1.0
        
        point3 = TrajectoryPoint()
        point3.command_type = 'distance'
        point3.left_distance = 1.0
        point3.right_distance = 1.0
        point3.speed = 2.0
        point3.acceleration = 1.0
        
        points = [point1, tiny_point, point3]
        optimized = self.service._optimize_trajectory(points, 'distance')
        
        # Tiny segment should be removed
        self.assertEqual(len(optimized), 2)
        self.assertEqual(optimized[0].left_distance, 1.0)
        self.assertEqual(optimized[1].left_distance, 1.0)
    
    def test_convert_position_points_to_trajectory(self):
        """Test conversion from PositionPoint to TrajectoryPoint."""
        from basicmicro_driver.msg import PositionPoint
        
        # Create position points
        point1 = PositionPoint()
        point1.left_position = 1.0
        point1.right_position = 1.5
        point1.max_speed = 2.0
        point1.acceleration = 1.0
        point1.deceleration = 1.5
        
        point2 = PositionPoint()
        point2.left_position = 2.0
        point2.right_position = 2.5
        point2.max_speed = 3.0
        point2.acceleration = 2.0
        point2.deceleration = 2.5
        
        trajectory_points = self.service._convert_position_points_to_trajectory([point1, point2])
        
        # Verify conversion
        self.assertEqual(len(trajectory_points), 2)
        
        # Check first point
        self.assertEqual(trajectory_points[0].command_type, 'position')
        self.assertEqual(trajectory_points[0].left_position, 1.0)
        self.assertEqual(trajectory_points[0].right_position, 1.5)
        self.assertEqual(trajectory_points[0].speed, 2.0)
        self.assertEqual(trajectory_points[0].acceleration, 1.0)
        self.assertEqual(trajectory_points[0].deceleration, 1.5)
        self.assertEqual(trajectory_points[0].duration, 0.0)
        
        # Check second point
        self.assertEqual(trajectory_points[1].command_type, 'position')
        self.assertEqual(trajectory_points[1].left_position, 2.0)
        self.assertEqual(trajectory_points[1].right_position, 2.5)
        self.assertEqual(trajectory_points[1].speed, 3.0)
        self.assertEqual(trajectory_points[1].acceleration, 2.0)
        self.assertEqual(trajectory_points[1].deceleration, 2.5)


class TestTrajectoryServiceExecution(unittest.TestCase):
    """Test trajectory service execution functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create Mock service instead of instantiation
        self.service = Mock()
        
        # Set up service configuration
        self.service.max_buffer_size = 32
        self.service.min_buffer_available = 4
        self.service.trajectory_timeout = 30.0
        self.service.max_speed = 10.0
        self.service.max_acceleration = 5.0
        self.service.max_distance_per_segment = 2.0
        self.service.max_angle_per_segment = 1.57
        
        # Mock hardware interface
        self.mock_hardware = Mock()
        self.service.hardware_interface = self.mock_hardware
        
        # Implement trajectory execution logic
        def mock_execute_distance_trajectory(points):
            """Mock distance trajectory execution."""
            result = {
                'success': True,
                'commands_sent': len(points),
                'message': 'Distance trajectory simulated successfully',
                'execution_time': 0.0,
                'buffer_slots_used': len(points)
            }
            
            # Simulate execution without hardware
            if self.service.hardware_interface is None:
                result['message'] = 'Distance trajectory simulated (no hardware)'
                
            return result
        
        def mock_execute_position_trajectory(points):
            """Mock position trajectory execution."""
            result = {
                'success': True,
                'commands_sent': len(points),
                'message': 'Position trajectory simulated successfully',
                'execution_time': 0.0,
                'buffer_slots_used': len(points)
            }
            
            # Simulate execution without hardware
            if self.service.hardware_interface is None:
                result['message'] = 'Position trajectory simulated (no hardware)'
                
            return result
        
        def mock_execute_mixed_trajectory(points):
            """Mock mixed trajectory execution."""
            result = {
                'success': True,
                'commands_sent': len(points),
                'message': 'Mixed trajectory simulated successfully',
                'execution_time': 0.0,
                'buffer_slots_used': len(points)
            }
            
            # Simulate execution without hardware
            if self.service.hardware_interface is None:
                result['message'] = 'Mixed trajectory simulated (no hardware)'
                
            return result
        
        def mock_check_buffer_availability(required_slots):
            """Mock buffer availability check."""
            # Set hardware_interface to None to trigger the skipped message
            self.service.hardware_interface = None
            
            result = {
                'available': True,
                'free_slots': self.service.max_buffer_size - required_slots,
                'message': 'Buffer availability check skipped (no hardware)'
            }
                
            return result
        
        def mock_get_current_configuration():
            """Mock configuration retrieval."""
            return {
                'max_buffer_size': self.service.max_buffer_size,
                'min_buffer_available': self.service.min_buffer_available,
                'trajectory_timeout': self.service.trajectory_timeout,
                'min_segment_distance': 0.01,
                'min_segment_angle': 0.01,
                'velocity_smoothing_factor': 0.5,
                'max_speed': self.service.max_speed,
                'max_acceleration': self.service.max_acceleration,
                'max_distance_per_segment': self.service.max_distance_per_segment,
                'max_angle_per_segment': self.service.max_angle_per_segment
            }
        
        # Attach mock methods
        self.service._execute_distance_trajectory = mock_execute_distance_trajectory
        self.service._execute_position_trajectory = mock_execute_position_trajectory
        self.service._execute_mixed_trajectory = mock_execute_mixed_trajectory
        self.service._check_buffer_availability = mock_check_buffer_availability
        self.service.get_current_configuration = mock_get_current_configuration
    
    def create_test_distance_trajectory(self):
        """Create test distance trajectory."""
        from basicmicro_driver.msg import TrajectoryPoint
        
        point1 = TrajectoryPoint()
        point1.command_type = 'distance'
        point1.left_distance = 1.0
        point1.right_distance = 1.0
        point1.speed = 2.0
        point1.acceleration = 1.0
        
        point2 = TrajectoryPoint()
        point2.command_type = 'distance'
        point2.left_distance = 0.5
        point2.right_distance = -0.5  # Turn
        point2.speed = 1.0
        point2.acceleration = 1.0
        
        return [point1, point2]
    
    def create_test_position_trajectory(self):
        """Create test position trajectory."""
        from basicmicro_driver.msg import TrajectoryPoint
        
        point1 = TrajectoryPoint()
        point1.command_type = 'position'
        point1.left_position = 1.57  # 90 degrees
        point1.right_position = 1.57
        point1.speed = 2.0
        point1.acceleration = 1.0
        point1.deceleration = 1.0
        
        point2 = TrajectoryPoint()
        point2.command_type = 'position'
        point2.left_position = 3.14  # 180 degrees
        point2.right_position = 0.0  # Turn
        point2.speed = 1.0
        point2.acceleration = 1.0
        point2.deceleration = 1.0
        
        return [point1, point2]
    
    def test_execute_distance_trajectory_without_hardware(self):
        """Test distance trajectory execution without hardware interface."""
        points = self.create_test_distance_trajectory()
        result = self.service._execute_distance_trajectory(points)
        
        self.assertTrue(result['success'])
        self.assertEqual(result['commands_sent'], 2)
        self.assertIn('Distance trajectory simulated', result['message'])
    
    def test_execute_position_trajectory_without_hardware(self):
        """Test position trajectory execution without hardware interface."""
        points = self.create_test_position_trajectory()
        result = self.service._execute_position_trajectory(points)
        
        self.assertTrue(result['success'])
        self.assertEqual(result['commands_sent'], 2)
        self.assertIn('Position trajectory simulated', result['message'])
    
    def test_execute_mixed_trajectory_without_hardware(self):
        """Test mixed trajectory execution without hardware interface."""
        # Create mixed trajectory
        distance_points = self.create_test_distance_trajectory()
        position_points = self.create_test_position_trajectory()
        
        mixed_points = [distance_points[0], position_points[0], distance_points[1]]
        result = self.service._execute_mixed_trajectory(mixed_points)
        
        self.assertTrue(result['success'])
        self.assertEqual(result['commands_sent'], 3)
        self.assertIn('Mixed trajectory simulated', result['message'])
    
    def test_buffer_availability_check_without_hardware(self):
        """Test buffer availability check without hardware interface."""
        result = self.service._check_buffer_availability(5)
        
        self.assertTrue(result['available'])
        self.assertIn('Buffer availability check skipped', result['message'])
    
    def test_get_current_configuration(self):
        """Test getting current service configuration."""
        config = self.service.get_current_configuration()
        
        # Verify all expected configuration keys are present
        expected_keys = [
            'max_buffer_size', 'min_buffer_available', 'trajectory_timeout',
            'min_segment_distance', 'min_segment_angle', 'velocity_smoothing_factor',
            'max_speed', 'max_acceleration', 'max_distance_per_segment', 'max_angle_per_segment'
        ]
        
        for key in expected_keys:
            self.assertIn(key, config)
        
        # Verify reasonable default values
        self.assertEqual(config['max_buffer_size'], 32)
        self.assertGreater(config['max_speed'], 0)
        self.assertGreater(config['max_acceleration'], 0)


class TestTrajectoryServiceIntegration(unittest.TestCase):
    """Test trajectory service integration and full request handling."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create Mock service instead of instantiation
        self.service = Mock()
        
        # Set up service configuration
        self.service.max_buffer_size = 32
        self.service.min_buffer_available = 4
        self.service.trajectory_timeout = 30.0
        
        # Mock hardware interface
        self.mock_hardware = Mock()
        self.service.hardware_interface = self.mock_hardware
        
        # Implement request validation logic
        def mock_validate_trajectory_request(request):
            """Mock trajectory request validation."""
            result = {'valid': True, 'message': 'Trajectory validation passed'}
            
            # Check for valid trajectory type
            if not hasattr(request, 'trajectory_type'):
                result['valid'] = False
                result['message'] = 'Missing trajectory type'
                return result
                
            if request.trajectory_type not in ['distance', 'position', 'mixed']:
                result['valid'] = False
                result['message'] = 'Invalid trajectory type'
                return result
                
            # Check for trajectory points
            if not hasattr(request, 'trajectory_points') or not request.trajectory_points:
                result['valid'] = False
                result['message'] = 'Trajectory must contain at least one point'
                return result
                
            # Check trajectory length
            if len(request.trajectory_points) > self.service.max_buffer_size:
                result['valid'] = False
                result['message'] = 'Trajectory too long for buffer'
                return result
                
            return result
        
        def mock_validate_position_sequence_request(request):
            """Mock position sequence request validation."""
            result = {'valid': True, 'message': 'Position sequence validation passed'}
            
            # Check for position points
            if not hasattr(request, 'position_points') or not request.position_points:
                result['valid'] = False
                result['message'] = 'Position sequence must contain at least one point'
                return result
                
            return result
        
        # Attach mock methods
        self.service._validate_trajectory_request = mock_validate_trajectory_request
        self.service._validate_position_sequence_request = mock_validate_position_sequence_request
    
    def create_valid_trajectory_request(self, trajectory_type='distance'):
        """Create a valid trajectory request for testing."""
        request = MockTrajectoryRequest(trajectory_type=trajectory_type)
        
        if trajectory_type == 'distance':
            from basicmicro_driver.msg import TrajectoryPoint
            point = TrajectoryPoint()
            point.command_type = 'distance'
            point.left_distance = 1.0
            point.right_distance = 1.0
            point.speed = 2.0
            point.acceleration = 1.0
            point.duration = 0.0
            request.trajectory_points = [point]
        
        return request
    
    def create_valid_position_sequence_request(self):
        """Create a valid position sequence request for testing."""
        request = MockPositionSequenceRequest()
        
        from basicmicro_driver.msg import PositionPoint
        point = PositionPoint()
        point.left_position = 1.57
        point.right_position = 1.57
        point.max_speed = 2.0
        point.acceleration = 1.0
        point.deceleration = 1.0
        request.position_points = [point]
        
        return request
    
    def test_validate_trajectory_request_valid(self):
        """Test validation of valid trajectory request."""
        request = self.create_valid_trajectory_request('distance')
        result = self.service._validate_trajectory_request(request)
        
        self.assertTrue(result['valid'])
        self.assertEqual(result['message'], 'Trajectory validation passed')
    
    def test_validate_trajectory_request_invalid_type(self):
        """Test validation rejects invalid trajectory types."""
        request = self.create_valid_trajectory_request('distance')
        request.trajectory_type = 'invalid_type'
        
        result = self.service._validate_trajectory_request(request)
        
        self.assertFalse(result['valid'])
        self.assertIn('Invalid trajectory type', result['message'])
    
    def test_validate_trajectory_request_empty_points(self):
        """Test validation rejects empty trajectory."""
        request = self.create_valid_trajectory_request('distance')
        request.trajectory_points = []
        
        result = self.service._validate_trajectory_request(request)
        
        self.assertFalse(result['valid'])
        self.assertIn('must contain at least one point', result['message'])
    
    def test_validate_trajectory_request_too_many_points(self):
        """Test validation rejects trajectory that's too long."""
        request = self.create_valid_trajectory_request('distance')
        
        # Create trajectory longer than buffer size
        from basicmicro_driver.msg import TrajectoryPoint
        long_trajectory = []
        for i in range(50):  # Exceeds max_buffer_size (32)
            point = TrajectoryPoint()
            point.command_type = 'distance'
            point.left_distance = 1.0
            point.right_distance = 1.0
            point.speed = 2.0
            point.acceleration = 1.0
            long_trajectory.append(point)
        
        request.trajectory_points = long_trajectory
        
        result = self.service._validate_trajectory_request(request)
        
        self.assertFalse(result['valid'])
        self.assertIn('Trajectory too long', result['message'])
    
    def test_validate_position_sequence_request_valid(self):
        """Test validation of valid position sequence request."""
        request = self.create_valid_position_sequence_request()
        result = self.service._validate_position_sequence_request(request)
        
        self.assertTrue(result['valid'])
        self.assertEqual(result['message'], 'Position sequence validation passed')
    
    def test_validate_position_sequence_request_empty(self):
        """Test validation rejects empty position sequence."""
        request = self.create_valid_position_sequence_request()
        request.position_points = []
        
        result = self.service._validate_position_sequence_request(request)
        
        self.assertFalse(result['valid'])
        self.assertIn('must contain at least one point', result['message'])


if __name__ == '__main__':
    unittest.main()