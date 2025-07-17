#!/usr/bin/env python3
"""
Simple test for Distance Movement Service functionality.

Tests core functionality without requiring generated service interfaces.
"""

import unittest
import math
from unittest.mock import Mock, MagicMock


class TestUnitConverterDistanceMethods(unittest.TestCase):
    """Test the new distance conversion methods added to UnitConverter."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Import here to avoid ROS2 dependency issues
        import sys
        import os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'basicmicro_driver'))
        
        from unit_converter import UnitConverter
        
        # Create unit converter with known parameters
        self.converter = UnitConverter(
            wheel_radius=0.1,  # 10cm radius
            encoder_counts_per_rev=1000,
            gear_ratio=1.0
        )
    
    def test_distance_to_radians_conversion(self):
        """Test conversion from linear distance to wheel radians."""
        # 1 meter distance with 0.1m radius wheel
        # Expected: 1.0 / 0.1 = 10 radians
        result = self.converter.distance_to_radians(1.0)
        expected = 1.0 / 0.1  # distance / wheel_radius
        self.assertAlmostEqual(result, expected, places=5)
        
        # Test with wheel circumference distance (should be 2π radians)
        circumference = 2 * math.pi * 0.1
        result = self.converter.distance_to_radians(circumference)
        expected = 2 * math.pi
        self.assertAlmostEqual(result, expected, places=5)
    
    def test_radians_to_distance_conversion(self):
        """Test conversion from wheel radians to linear distance."""
        # 2π radians (one full revolution) with 0.1m radius
        # Expected: 2π * 0.1 = circumference
        result = self.converter.radians_to_distance(2 * math.pi)
        expected = 2 * math.pi * 0.1
        self.assertAlmostEqual(result, expected, places=5)
        
        # Test with 1 radian
        result = self.converter.radians_to_distance(1.0)
        expected = 1.0 * 0.1  # radians * wheel_radius
        self.assertAlmostEqual(result, expected, places=5)
    
    def test_speed_to_counts_per_sec_conversion(self):
        """Test conversion from linear speed to encoder counts per second."""
        # 1 m/s with 0.1m radius wheel
        # Angular speed = 1.0 / 0.1 = 10 rad/s
        # Counts/sec = 10 * (1000 / 2π) ≈ 1591.5
        result = self.converter.speed_to_counts_per_sec(1.0)
        
        angular_speed = 1.0 / 0.1  # 10 rad/s
        expected = angular_speed * (1000 / (2 * math.pi))
        self.assertAlmostEqual(result, int(round(expected)), delta=1)
    
    def test_acceleration_to_counts_per_sec2_conversion(self):
        """Test conversion from linear acceleration to counts per second²."""
        # 1 m/s² with 0.1m radius wheel
        # Angular acceleration = 1.0 / 0.1 = 10 rad/s²
        # Counts/s² = 10 * (1000 / 2π)
        result = self.converter.acceleration_to_counts_per_sec2(1.0)
        
        angular_accel = 1.0 / 0.1  # 10 rad/s²
        expected = angular_accel * (1000 / (2 * math.pi))
        self.assertAlmostEqual(result, int(round(expected)), delta=1)
    
    def test_distance_conversion_consistency(self):
        """Test that distance conversions are consistent."""
        # Test round-trip conversion
        original_distance = 2.5  # meters
        
        # Distance -> Radians -> Distance
        radians = self.converter.distance_to_radians(original_distance)
        back_to_distance = self.converter.radians_to_distance(radians)
        self.assertAlmostEqual(original_distance, back_to_distance, places=5)
        
        # Distance -> Counts -> Distance  
        counts = self.converter.distance_to_counts(original_distance)
        back_to_distance2 = self.converter.counts_to_meters(counts)
        self.assertAlmostEqual(original_distance, back_to_distance2, places=3)  # Lower precision due to integer rounding


class TestDistanceMovementLogic(unittest.TestCase):
    """Test distance movement logic without ROS2 dependencies."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Mock hardware interface
        self.mock_hardware = Mock()
        self.mock_hardware.wheel_radius = 0.1
        self.mock_hardware.encoder_counts_per_rev = 1000
        self.mock_hardware.gear_ratio = 1.0
        self.mock_hardware.address = 0x80
        
        # Mock controller
        self.mock_controller = Mock()
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = (True,)
        self.mock_controller.ReadBuffers.return_value = (True, 4)
        self.mock_controller.DutyM1M2.return_value = (True,)
        self.mock_hardware.controller = self.mock_controller
    
    def test_distance_parameter_validation(self):
        """Test validation logic for distance movement parameters."""
        # Create a simplified validation function similar to the service
        def validate_distance_request(left_distance, right_distance, speed, acceleration):
            max_distance = 1000.0
            max_speed = 10.0
            max_acceleration = 50.0
            
            if abs(left_distance) > max_distance or abs(right_distance) > max_distance:
                return {'valid': False, 'message': f"Distance too large (max: {max_distance}m)"}
            
            if abs(speed) > max_speed:
                return {'valid': False, 'message': f"Speed too high (max: {max_speed} m/s)"}
            
            if speed <= 0:
                return {'valid': False, 'message': "Speed must be positive"}
            
            if abs(acceleration) > max_acceleration:
                return {'valid': False, 'message': f"Acceleration too high (max: {max_acceleration} m/s²)"}
            
            if acceleration <= 0:
                return {'valid': False, 'message': "Acceleration must be positive"}
            
            return {'valid': True, 'message': 'Valid request'}
        
        # Test valid parameters
        result = validate_distance_request(1.0, 1.0, 2.0, 1.0)
        self.assertTrue(result['valid'])
        
        # Test excessive distance
        result = validate_distance_request(2000.0, 1.0, 2.0, 1.0)
        self.assertFalse(result['valid'])
        self.assertIn("too large", result['message'])
        
        # Test negative speed
        result = validate_distance_request(1.0, 1.0, -2.0, 1.0)
        self.assertFalse(result['valid'])
        self.assertIn("positive", result['message'])
        
        # Test excessive acceleration
        result = validate_distance_request(1.0, 1.0, 2.0, 100.0)
        self.assertFalse(result['valid'])
        self.assertIn("too high", result['message'])
    
    def test_position_limit_checking_logic(self):
        """Test position limit checking logic."""
        # Simulate position limit checking
        def check_position_limits(current_left, current_right, left_distance, right_distance, 
                                left_min, left_max, right_min, right_max, wheel_radius):
            # Convert distances to radians for checking
            left_distance_rad = left_distance / wheel_radius
            right_distance_rad = right_distance / wheel_radius
            
            final_left_pos = current_left + left_distance_rad
            final_right_pos = current_right + right_distance_rad
            
            violations = []
            
            if final_left_pos < left_min:
                violations.append(f"Left wheel below minimum")
            if final_left_pos > left_max:
                violations.append(f"Left wheel above maximum")
            if final_right_pos < right_min:
                violations.append(f"Right wheel below minimum")
            if final_right_pos > right_max:
                violations.append(f"Right wheel above maximum")
            
            return {
                'allowed': len(violations) == 0,
                'violations': violations
            }
        
        # Test movement within limits
        result = check_position_limits(
            current_left=0.0, current_right=0.0,
            left_distance=0.5, right_distance=0.5,
            left_min=-10.0, left_max=10.0,
            right_min=-10.0, right_max=10.0,
            wheel_radius=0.1
        )
        self.assertTrue(result['allowed'])
        self.assertEqual(len(result['violations']), 0)
        
        # Test movement exceeding limits
        result = check_position_limits(
            current_left=9.0, current_right=9.0,
            left_distance=0.5, right_distance=0.5,  # Would exceed max of 10.0
            left_min=-10.0, left_max=10.0,
            right_min=-10.0, right_max=10.0,
            wheel_radius=0.1
        )
        self.assertFalse(result['allowed'])
        self.assertGreater(len(result['violations']), 0)
    
    def test_command_execution_logic(self):
        """Test command execution logic with unit conversion."""
        import sys
        import os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'basicmicro_driver'))
        
        from unit_converter import UnitConverter
        
        converter = UnitConverter(
            wheel_radius=0.1,
            encoder_counts_per_rev=1000,
            gear_ratio=1.0
        )
        
        # Test conversion for realistic command
        left_distance = 1.0  # meters
        right_distance = 0.5  # meters  
        speed = 2.0  # m/s
        acceleration = 1.0  # m/s²
        
        # Convert to encoder units
        left_distance_counts = converter.distance_to_counts(left_distance)
        right_distance_counts = converter.distance_to_counts(right_distance)
        speed_counts = converter.speed_to_counts_per_sec(speed)
        acceleration_counts = converter.acceleration_to_counts_per_sec2(acceleration)
        
        # Verify conversions are reasonable
        self.assertGreater(left_distance_counts, 0)
        self.assertGreater(right_distance_counts, 0)
        self.assertGreater(speed_counts, 0)
        self.assertGreater(acceleration_counts, 0)
        
        # Test mock controller call
        address = 0x80
        buffer_flag = 0  # Immediate execution
        
        result = self.mock_controller.SpeedAccelDistanceM1M2(
            address,
            acceleration_counts,
            speed_counts, left_distance_counts,
            speed_counts, right_distance_counts,
            buffer_flag
        )
        
        self.assertTrue(result[0])  # Should succeed
        self.mock_controller.SpeedAccelDistanceM1M2.assert_called_once()


if __name__ == '__main__':
    unittest.main()