#!/usr/bin/env python3
"""
Hardware-free test suite for Trajectory Service Core Logic

Tests trajectory service validation and optimization logic without ROS2 dependencies.

Author: ROS2 Driver Development
"""

import unittest
import math
import sys
import os

# Add the package to Python path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import only the unit converter for testing
from basicmicro_driver.unit_converter import UnitConverter


class MockTrajectoryPoint:
    """Mock TrajectoryPoint for testing without ROS2 message dependencies."""
    
    def __init__(self):
        self.command_type = 'distance'
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.left_position = 0.0
        self.right_position = 0.0
        self.deceleration = 1.0
        self.speed = 1.0
        self.acceleration = 1.0
        self.duration = 0.0


class MockPositionPoint:
    """Mock PositionPoint for testing without ROS2 message dependencies."""
    
    def __init__(self):
        self.left_position = 0.0
        self.right_position = 0.0
        self.max_speed = 1.0
        self.acceleration = 1.0
        self.deceleration = 1.0


class TrajectoryServiceCoreTester:
    """Core trajectory service logic without ROS2 dependencies."""
    
    def __init__(self):
        # Configuration parameters (from trajectory service)
        self.max_buffer_size = 32
        self.min_buffer_available = 4
        self.trajectory_timeout = 30.0
        self.min_segment_distance = 0.001
        self.min_segment_angle = 0.001
        self.velocity_smoothing_factor = 0.9
        self.max_speed = 10.0
        self.max_acceleration = 50.0
        self.max_distance_per_segment = 100.0
        self.max_angle_per_segment = 100.0
        
        # Unit converter for testing
        self.unit_converter = UnitConverter(
            wheel_radius=0.1,
            encoder_counts_per_rev=1000,
            gear_ratio=1.0
        )
    
    def _validate_trajectory_point(self, point, trajectory_type):
        """
        Validate individual trajectory point parameters.
        Core logic extracted from trajectory service.
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
    
    def _validate_position_point(self, point):
        """
        Validate individual position point parameters.
        Core logic extracted from trajectory service.
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
    
    def _optimize_trajectory(self, points, trajectory_type):
        """
        Optimize trajectory for smooth execution.
        Core logic extracted from trajectory service.
        """
        if len(points) <= 1:
            return points
        
        optimized = []
        
        for i, point in enumerate(points):
            # Create optimized point (copy original)
            opt_point = MockTrajectoryPoint()
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


class TestTrajectoryServiceCoreLogic(unittest.TestCase):
    """Test trajectory service core logic without ROS2 dependencies."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.service = TrajectoryServiceCoreTester()
        
        # Test parameters
        self.valid_speed = 2.0  # m/s
        self.valid_acceleration = 1.0  # m/s²
        self.valid_distance = 1.0  # meters
        self.valid_position = 1.57  # radians (~90 degrees)
    
    def create_valid_distance_point(self):
        """Create a valid distance trajectory point for testing."""
        point = MockTrajectoryPoint()
        point.command_type = 'distance'
        point.left_distance = self.valid_distance
        point.right_distance = self.valid_distance
        point.speed = self.valid_speed
        point.acceleration = self.valid_acceleration
        point.duration = 0.0
        return point
    
    def create_valid_position_point(self):
        """Create a valid position trajectory point for testing."""
        point = MockTrajectoryPoint()
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
        point = MockPositionPoint()
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
    
    def test_optimize_trajectory_single_point(self):
        """Test optimization with single point (should return unchanged)."""
        point = self.create_valid_distance_point()
        point.speed = 2.0
        
        optimized = self.service._optimize_trajectory([point], 'distance')
        
        self.assertEqual(len(optimized), 1)
        self.assertEqual(optimized[0].speed, 2.0)  # Should be unchanged
    
    def test_optimize_trajectory_velocity_smoothing(self):
        """Test velocity smoothing in multi-point trajectories."""
        point1 = self.create_valid_distance_point()
        point1.speed = 1.0
        
        point2 = self.create_valid_distance_point()
        point2.speed = 5.0  # High middle speed should be smoothed
        
        point3 = self.create_valid_distance_point()
        point3.speed = 2.0
        
        optimized = self.service._optimize_trajectory([point1, point2, point3], 'distance')
        
        # Middle point should have smoothed speed
        self.assertEqual(len(optimized), 3)
        self.assertLess(optimized[1].speed, 5.0)  # Should be smoothed down
        self.assertGreater(optimized[1].speed, 1.0)  # But not too much
    
    def test_optimize_trajectory_removes_tiny_segments(self):
        """Test removal of tiny distance segments."""
        point1 = self.create_valid_distance_point()
        point1.left_distance = 1.0
        point1.right_distance = 1.0
        
        # Tiny segment (below min_segment_distance)
        tiny_point = self.create_valid_distance_point()
        tiny_point.left_distance = 0.0001  # Very small
        tiny_point.right_distance = 0.0001
        
        point3 = self.create_valid_distance_point()
        point3.left_distance = 1.0
        point3.right_distance = 1.0
        
        points = [point1, tiny_point, point3]
        optimized = self.service._optimize_trajectory(points, 'distance')
        
        # Tiny segment should be removed
        self.assertEqual(len(optimized), 2)
        self.assertEqual(optimized[0].left_distance, 1.0)
        self.assertEqual(optimized[1].left_distance, 1.0)


class TestTrajectoryUnitConversions(unittest.TestCase):
    """Test trajectory-related unit conversions."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.service = TrajectoryServiceCoreTester()
    
    def test_distance_to_counts_conversion(self):
        """Test distance to encoder counts conversion."""
        distance_m = 1.0  # 1 meter
        counts = self.service.unit_converter.distance_to_counts(distance_m)
        
        # With wheel_radius=0.1m, circumference = 2*pi*0.1 = 0.628m
        # 1 meter = 1/0.628 = 1.59 revolutions
        # 1.59 revolutions * 1000 counts/rev = 1592 counts
        expected_counts = int(1000 / (2 * math.pi * 0.1))
        
        self.assertAlmostEqual(counts, expected_counts, delta=1)
    
    def test_speed_to_counts_per_sec_conversion(self):
        """Test linear speed to angular velocity conversion."""
        speed_mps = 1.0  # 1 m/s
        counts_per_sec = self.service.unit_converter.speed_to_counts_per_sec(speed_mps)
        
        # 1 m/s = 1/0.628 = 1.59 rev/s = 1592 counts/sec
        expected_counts_per_sec = int(1000 / (2 * math.pi * 0.1))
        
        self.assertAlmostEqual(counts_per_sec, expected_counts_per_sec, delta=1)
    
    def test_acceleration_to_counts_per_sec2_conversion(self):
        """Test linear acceleration to angular acceleration conversion."""
        accel_mps2 = 1.0  # 1 m/s²
        counts_per_sec2 = self.service.unit_converter.acceleration_to_counts_per_sec2(accel_mps2)
        
        # 1 m/s² = 1592 counts/s²
        expected_counts_per_sec2 = int(1000 / (2 * math.pi * 0.1))
        
        self.assertAlmostEqual(counts_per_sec2, expected_counts_per_sec2, delta=1)
    
    def test_round_trip_conversions(self):
        """Test round-trip conversion accuracy."""
        original_distance = 5.0  # meters
        
        # Convert to counts and back
        counts = self.service.unit_converter.distance_to_counts(original_distance)
        radians = self.service.unit_converter.counts_to_radians(counts)
        back_to_distance = self.service.unit_converter.radians_to_distance(radians)
        
        # Should be very close to original
        self.assertAlmostEqual(original_distance, back_to_distance, places=3)


if __name__ == '__main__':
    unittest.main()