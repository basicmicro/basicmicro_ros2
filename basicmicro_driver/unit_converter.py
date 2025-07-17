"""
Unit conversion utilities for BasicMicro ROS2 driver.

This module provides comprehensive unit conversion between ROS2 standard units
(radians, m/s) and BasicMicro encoder counts for wheel-based robots.

Author: ROS2 BasicMicro Driver
License: MIT
"""

import math
from typing import Union


class UnitConverter:
    """
    Comprehensive unit conversion for BasicMicro motor controllers.
    
    Handles conversion between:
    - ROS2 standard units: radians for position, rad/s for velocity  
    - BasicMicro units: encoder counts and counts/sec
    
    Supports configurable wheel radius, encoder resolution, and gear ratios.
    """
    
    def __init__(self, wheel_radius: float, encoder_counts_per_rev: int, gear_ratio: float = 1.0):
        """
        Initialize unit converter with robot physical parameters.
        
        Args:
            wheel_radius: Wheel radius in meters (must be positive)
            encoder_counts_per_rev: Encoder counts per wheel revolution (must be positive)
            gear_ratio: Gear ratio between encoder and wheel (must be positive)
            
        Raises:
            ValueError: If any parameter is invalid
        """
        self._validate_parameters(wheel_radius, encoder_counts_per_rev, gear_ratio)
        
        self.wheel_radius = wheel_radius
        self.encoder_counts_per_rev = encoder_counts_per_rev
        self.gear_ratio = gear_ratio
        
        # Pre-calculate conversion factors for efficiency
        self._radians_per_count = (2.0 * math.pi) / (encoder_counts_per_rev * gear_ratio)
        self._counts_per_radian = (encoder_counts_per_rev * gear_ratio) / (2.0 * math.pi)
        
        # Wheel circumference for distance conversions
        self.wheel_circumference = 2.0 * math.pi * wheel_radius
        
        # Counts per meter of wheel travel
        self._counts_per_meter = (encoder_counts_per_rev * gear_ratio) / self.wheel_circumference
        
    def _validate_parameters(self, wheel_radius: float, encoder_counts_per_rev: int, gear_ratio: float) -> None:
        """
        Validate constructor parameters.
        
        Raises:
            ValueError: If any parameter is invalid
        """
        if wheel_radius <= 0:
            raise ValueError(f"wheel_radius must be positive, got {wheel_radius}")
            
        if encoder_counts_per_rev <= 0:
            raise ValueError(f"encoder_counts_per_rev must be positive, got {encoder_counts_per_rev}")
            
        if gear_ratio <= 0:
            raise ValueError(f"gear_ratio must be positive, got {gear_ratio}")
    
    def counts_to_radians(self, counts: Union[int, float]) -> float:
        """
        Convert encoder counts to radians.
        
        Args:
            counts: Encoder counts (can be negative for reverse rotation)
            
        Returns:
            Wheel position in radians
            
        Examples:
            >>> converter = UnitConverter(0.1, 1000, 1.0)
            >>> converter.counts_to_radians(1000)  # One full revolution
            6.283185307179586
        """
        return float(counts) * self._radians_per_count
    
    def radians_to_counts(self, radians: float) -> int:
        """
        Convert radians to encoder counts.
        
        Args:
            radians: Wheel position in radians
            
        Returns:
            Encoder counts (rounded to nearest integer)
            
        Examples:
            >>> converter = UnitConverter(0.1, 1000, 1.0)
            >>> converter.radians_to_counts(2 * math.pi)  # One full revolution
            1000
        """
        counts = radians * self._counts_per_radian
        
        # Handle potential overflow for very large values
        if abs(counts) > 2**31 - 1:
            raise OverflowError(f"Counts value {counts} exceeds 32-bit signed integer range")
            
        return int(round(counts))
    
    def counts_per_sec_to_rad_per_sec(self, counts_per_sec: Union[int, float]) -> float:
        """
        Convert counts/sec to rad/sec.
        
        Args:
            counts_per_sec: Encoder velocity in counts per second
            
        Returns:
            Wheel velocity in radians per second
            
        Examples:
            >>> converter = UnitConverter(0.1, 1000, 1.0)
            >>> converter.counts_per_sec_to_rad_per_sec(1000)  # 1 rev/sec
            6.283185307179586
        """
        return float(counts_per_sec) * self._radians_per_count
    
    def rad_per_sec_to_counts_per_sec(self, rad_per_sec: float) -> int:
        """
        Convert rad/sec to counts/sec.
        
        Args:
            rad_per_sec: Wheel velocity in radians per second
            
        Returns:
            Encoder velocity in counts per second (rounded to nearest integer)
            
        Examples:
            >>> converter = UnitConverter(0.1, 1000, 1.0)
            >>> converter.rad_per_sec_to_counts_per_sec(2 * math.pi)  # 1 rev/sec
            1000
        """
        counts_per_sec = rad_per_sec * self._counts_per_radian
        
        # Handle potential overflow for very large velocities
        if abs(counts_per_sec) > 2**31 - 1:
            raise OverflowError(f"Counts/sec value {counts_per_sec} exceeds 32-bit signed integer range")
            
        return int(round(counts_per_sec))
    
    def rad_per_sec_to_duty(self, rad_per_sec: float, max_rad_per_sec: float = 10.0) -> int:
        """
        Convert rad/sec to duty cycle for open-loop control.
        
        Args:
            rad_per_sec: Wheel velocity in radians per second
            max_rad_per_sec: Maximum achievable velocity for scaling (configurable)
            
        Returns:
            Duty cycle value (-32767 to +32767)
            
        Note:
            This is a simplified linear conversion. Actual implementation may need
            motor-specific calibration for accurate velocity control.
            
        Examples:
            >>> converter = UnitConverter(0.1, 1000, 1.0)
            >>> converter.rad_per_sec_to_duty(5.0, 10.0)  # 50% of max speed
            16383
        """
        if max_rad_per_sec <= 0:
            raise ValueError(f"max_rad_per_sec must be positive, got {max_rad_per_sec}")
            
        duty_ratio = rad_per_sec / max_rad_per_sec
        duty_ratio = max(-1.0, min(1.0, duty_ratio))  # Clamp to [-1, 1]
        return int(duty_ratio * 32767)
    
    def meters_to_counts(self, meters: float) -> int:
        """
        Convert linear distance in meters to encoder counts.
        
        Args:
            meters: Linear distance in meters
            
        Returns:
            Encoder counts for the specified distance
            
        Examples:
            >>> converter = UnitConverter(0.1, 1000, 1.0)  # 0.1m radius wheel
            >>> converter.meters_to_counts(0.628)  # ~1/10 of circumference
            100
        """
        counts = meters * self._counts_per_meter
        
        if abs(counts) > 2**31 - 1:
            raise OverflowError(f"Distance {meters}m converts to {counts} counts, exceeding 32-bit range")
            
        return int(round(counts))
    
    def counts_to_meters(self, counts: Union[int, float]) -> float:
        """
        Convert encoder counts to linear distance in meters.
        
        Args:
            counts: Encoder counts
            
        Returns:
            Linear distance in meters
            
        Examples:
            >>> converter = UnitConverter(0.1, 1000, 1.0)  # 0.1m radius wheel
            >>> converter.counts_to_meters(1000)  # One revolution
            0.6283185307179587
        """
        return float(counts) / self._counts_per_meter
    
    def distance_to_counts(self, distance_meters: float) -> int:
        """
        Convert linear distance in meters to encoder counts.
        
        Args:
            distance_meters: Linear distance in meters
            
        Returns:
            Encoder counts for the specified distance
        """
        return self.meters_to_counts(distance_meters)
    
    def distance_to_radians(self, distance_meters: float) -> float:
        """
        Convert linear distance in meters to wheel rotation in radians.
        
        Args:
            distance_meters: Linear distance in meters
            
        Returns:
            Wheel rotation in radians
        """
        # Distance = wheel_radius * angle_radians
        # Therefore: angle_radians = distance / wheel_radius
        return distance_meters / self.wheel_radius
    
    def radians_to_distance(self, radians: float) -> float:
        """
        Convert wheel rotation in radians to linear distance in meters.
        
        Args:
            radians: Wheel rotation in radians
            
        Returns:
            Linear distance in meters
        """
        # Distance = wheel_radius * angle_radians
        return radians * self.wheel_radius
    
    def speed_to_counts_per_sec(self, speed_m_per_s: float) -> int:
        """
        Convert linear speed in m/s to encoder counts per second.
        
        Args:
            speed_m_per_s: Linear speed in meters per second
            
        Returns:
            Encoder velocity in counts per second
        """
        # Convert linear speed to angular speed, then to counts/sec
        angular_speed = speed_m_per_s / self.wheel_radius  # rad/s
        return self.rad_per_sec_to_counts_per_sec(angular_speed)
    
    def acceleration_to_counts_per_sec2(self, accel_m_per_s2: float) -> int:
        """
        Convert linear acceleration in m/s² to encoder counts per second².
        
        Args:
            accel_m_per_s2: Linear acceleration in meters per second²
            
        Returns:
            Encoder acceleration in counts per second²
        """
        # Convert linear acceleration to angular acceleration, then to counts/s²
        angular_accel = accel_m_per_s2 / self.wheel_radius  # rad/s²
        counts_per_sec2 = angular_accel * self._counts_per_radian
        
        if abs(counts_per_sec2) > 2**31 - 1:
            raise OverflowError(f"Acceleration {accel_m_per_s2} m/s² exceeds 32-bit signed integer range")
            
        return int(round(counts_per_sec2))
    
    def validate_conversion_range(self, value: float, value_type: str) -> None:
        """
        Validate that a value is within safe conversion range.
        
        Args:
            value: Value to validate
            value_type: Type description for error messages
            
        Raises:
            OverflowError: If value would cause integer overflow
        """
        if value_type in ["radians", "rad_per_sec"]:
            counts_equivalent = abs(value * self._counts_per_radian)
        elif value_type == "meters":
            counts_equivalent = abs(value * self._counts_per_meter)
        else:
            return  # Unknown type, skip validation
            
        if counts_equivalent > 2**31 - 1:
            raise OverflowError(f"{value_type} value {value} would exceed 32-bit integer range")
    
    def get_conversion_info(self) -> dict:
        """
        Get conversion factor information for debugging and validation.
        
        Returns:
            Dictionary with conversion factors and robot parameters
        """
        return {
            "wheel_radius": self.wheel_radius,
            "encoder_counts_per_rev": self.encoder_counts_per_rev,
            "gear_ratio": self.gear_ratio,
            "wheel_circumference": self.wheel_circumference,
            "radians_per_count": self._radians_per_count,
            "counts_per_radian": self._counts_per_radian,
            "counts_per_meter": self._counts_per_meter
        }
    
    def __str__(self) -> str:
        """String representation of converter configuration."""
        return (f"UnitConverter(wheel_radius={self.wheel_radius}m, "
                f"encoder_counts_per_rev={self.encoder_counts_per_rev}, "
                f"gear_ratio={self.gear_ratio})")
    
    def __repr__(self) -> str:
        """Detailed representation of converter configuration."""
        return (f"UnitConverter(wheel_radius={self.wheel_radius}, "
                f"encoder_counts_per_rev={self.encoder_counts_per_rev}, "
                f"gear_ratio={self.gear_ratio})")