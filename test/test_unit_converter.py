"""
Unit tests for UnitConverter class.

Tests comprehensive unit conversion functionality including edge cases,
error conditions, and accuracy validation.

Author: ROS2 BasicMicro Driver
License: MIT
"""

import math
import pytest
from basicmicro_driver.unit_converter import UnitConverter


class TestUnitConverterInitialization:
    """Test UnitConverter initialization and parameter validation."""
    
    def test_valid_initialization(self):
        """Test valid initialization with typical parameters."""
        converter = UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=1.0)
        
        assert converter.wheel_radius == 0.1
        assert converter.encoder_counts_per_rev == 1000
        assert converter.gear_ratio == 1.0
        assert converter.wheel_circumference == pytest.approx(2 * math.pi * 0.1, rel=1e-6)
    
    def test_initialization_with_gear_ratio(self):
        """Test initialization with different gear ratios."""
        # Test gear reduction
        converter = UnitConverter(wheel_radius=0.05, encoder_counts_per_rev=500, gear_ratio=2.0)
        assert converter.gear_ratio == 2.0
        
        # Test gear multiplication  
        converter = UnitConverter(wheel_radius=0.2, encoder_counts_per_rev=2000, gear_ratio=0.5)
        assert converter.gear_ratio == 0.5
    
    def test_invalid_wheel_radius(self):
        """Test initialization with invalid wheel radius values."""
        with pytest.raises(ValueError, match="wheel_radius must be positive"):
            UnitConverter(wheel_radius=0.0, encoder_counts_per_rev=1000, gear_ratio=1.0)
            
        with pytest.raises(ValueError, match="wheel_radius must be positive"):
            UnitConverter(wheel_radius=-0.1, encoder_counts_per_rev=1000, gear_ratio=1.0)
    
    def test_invalid_encoder_counts(self):
        """Test initialization with invalid encoder counts."""
        with pytest.raises(ValueError, match="encoder_counts_per_rev must be positive"):
            UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=0, gear_ratio=1.0)
            
        with pytest.raises(ValueError, match="encoder_counts_per_rev must be positive"):
            UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=-1000, gear_ratio=1.0)
    
    def test_invalid_gear_ratio(self):
        """Test initialization with invalid gear ratios."""
        with pytest.raises(ValueError, match="gear_ratio must be positive"):
            UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=0.0)
            
        with pytest.raises(ValueError, match="gear_ratio must be positive"):
            UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=-1.0)


class TestBasicConversions:
    """Test basic conversion methods with known values."""
    
    @pytest.fixture
    def standard_converter(self):
        """Standard converter for testing: 10cm radius, 1000 counts/rev, 1:1 gear ratio."""
        return UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=1.0)
    
    @pytest.fixture
    def geared_converter(self):
        """Geared converter for testing: 5cm radius, 500 counts/rev, 2:1 gear ratio."""
        return UnitConverter(wheel_radius=0.05, encoder_counts_per_rev=500, gear_ratio=2.0)
    
    def test_counts_to_radians_basic(self, standard_converter):
        """Test basic counts to radians conversion."""
        # One full revolution (1000 counts) = 2π radians
        result = standard_converter.counts_to_radians(1000)
        assert result == pytest.approx(2 * math.pi, rel=1e-6)
        
        # Half revolution
        result = standard_converter.counts_to_radians(500)
        assert result == pytest.approx(math.pi, rel=1e-6)
        
        # Zero counts
        result = standard_converter.counts_to_radians(0)
        assert result == 0.0
        
        # Negative counts (reverse rotation)
        result = standard_converter.counts_to_radians(-1000)
        assert result == pytest.approx(-2 * math.pi, rel=1e-6)
    
    def test_radians_to_counts_basic(self, standard_converter):
        """Test basic radians to counts conversion."""
        # One full revolution (2π radians) = 1000 counts
        result = standard_converter.radians_to_counts(2 * math.pi)
        assert result == 1000
        
        # Half revolution
        result = standard_converter.radians_to_counts(math.pi)
        assert result == 500
        
        # Zero radians
        result = standard_converter.radians_to_counts(0.0)
        assert result == 0
        
        # Negative radians (reverse rotation)
        result = standard_converter.radians_to_counts(-2 * math.pi)
        assert result == -1000
    
    def test_counts_per_sec_to_rad_per_sec_basic(self, standard_converter):
        """Test basic velocity conversion from counts/sec to rad/sec."""
        # 1000 counts/sec = 2π rad/sec (1 revolution per second)
        result = standard_converter.counts_per_sec_to_rad_per_sec(1000)
        assert result == pytest.approx(2 * math.pi, rel=1e-6)
        
        # 500 counts/sec = π rad/sec (0.5 revolution per second)
        result = standard_converter.counts_per_sec_to_rad_per_sec(500)
        assert result == pytest.approx(math.pi, rel=1e-6)
        
        # Zero velocity
        result = standard_converter.counts_per_sec_to_rad_per_sec(0)
        assert result == 0.0
    
    def test_rad_per_sec_to_counts_per_sec_basic(self, standard_converter):
        """Test basic velocity conversion from rad/sec to counts/sec."""
        # 2π rad/sec = 1000 counts/sec (1 revolution per second)
        result = standard_converter.rad_per_sec_to_counts_per_sec(2 * math.pi)
        assert result == 1000
        
        # π rad/sec = 500 counts/sec (0.5 revolution per second)
        result = standard_converter.rad_per_sec_to_counts_per_sec(math.pi)
        assert result == 500
        
        # Zero velocity
        result = standard_converter.rad_per_sec_to_counts_per_sec(0.0)
        assert result == 0


class TestGearRatioConversions:
    """Test conversions with different gear ratios."""
    
    def test_gear_reduction_conversions(self):
        """Test conversions with gear reduction (gear_ratio > 1)."""
        # 2:1 gear reduction - encoder turns twice for each wheel revolution
        converter = UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=2.0)
        
        # One wheel revolution = 2000 encoder counts = 2π radians
        result = converter.counts_to_radians(2000)
        assert result == pytest.approx(2 * math.pi, rel=1e-6)
        
        result = converter.radians_to_counts(2 * math.pi)
        assert result == 2000
    
    def test_gear_multiplication_conversions(self):
        """Test conversions with gear multiplication (gear_ratio < 1)."""
        # 1:2 gear multiplication - encoder turns half for each wheel revolution
        converter = UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=0.5)
        
        # One wheel revolution = 500 encoder counts = 2π radians
        result = converter.counts_to_radians(500)
        assert result == pytest.approx(2 * math.pi, rel=1e-6)
        
        result = converter.radians_to_counts(2 * math.pi)
        assert result == 500


class TestDutyCycleConversions:
    """Test duty cycle conversion functionality."""
    
    @pytest.fixture
    def converter(self):
        """Standard converter for duty cycle testing."""
        return UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=1.0)
    
    def test_rad_per_sec_to_duty_basic(self, converter):
        """Test basic rad/sec to duty cycle conversion."""
        # Maximum positive velocity
        result = converter.rad_per_sec_to_duty(10.0, max_rad_per_sec=10.0)
        assert result == 32767
        
        # Maximum negative velocity
        result = converter.rad_per_sec_to_duty(-10.0, max_rad_per_sec=10.0)
        assert result == -32767
        
        # Half maximum velocity
        result = converter.rad_per_sec_to_duty(5.0, max_rad_per_sec=10.0)
        assert result == 16383  # 32767 / 2
        
        # Zero velocity
        result = converter.rad_per_sec_to_duty(0.0, max_rad_per_sec=10.0)
        assert result == 0
    
    def test_rad_per_sec_to_duty_clamping(self, converter):
        """Test duty cycle clamping for velocities exceeding maximum."""
        # Velocity exceeding maximum should clamp to max duty
        result = converter.rad_per_sec_to_duty(15.0, max_rad_per_sec=10.0)
        assert result == 32767
        
        # Negative velocity exceeding maximum should clamp to min duty
        result = converter.rad_per_sec_to_duty(-15.0, max_rad_per_sec=10.0)
        assert result == -32767
    
    def test_rad_per_sec_to_duty_invalid_max(self, converter):
        """Test duty cycle conversion with invalid maximum velocity."""
        with pytest.raises(ValueError, match="max_rad_per_sec must be positive"):
            converter.rad_per_sec_to_duty(5.0, max_rad_per_sec=0.0)
            
        with pytest.raises(ValueError, match="max_rad_per_sec must be positive"):
            converter.rad_per_sec_to_duty(5.0, max_rad_per_sec=-1.0)


class TestDistanceConversions:
    """Test linear distance conversion functionality."""
    
    @pytest.fixture
    def converter(self):
        """Standard converter: 10cm radius wheel, 1000 counts/rev."""
        return UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=1.0)
    
    def test_meters_to_counts_basic(self, converter):
        """Test basic meters to counts conversion."""
        # One wheel circumference = 2π * 0.1 = ~0.628m = 1000 counts
        circumference = 2 * math.pi * 0.1
        result = converter.meters_to_counts(circumference)
        assert result == 1000
        
        # Half circumference
        result = converter.meters_to_counts(circumference / 2)
        assert result == 500
        
        # Zero distance
        result = converter.meters_to_counts(0.0)
        assert result == 0
    
    def test_counts_to_meters_basic(self, converter):
        """Test basic counts to meters conversion."""
        # 1000 counts = one wheel circumference = 2π * 0.1 m
        result = converter.counts_to_meters(1000)
        expected_circumference = 2 * math.pi * 0.1
        assert result == pytest.approx(expected_circumference, rel=1e-6)
        
        # 500 counts = half circumference
        result = converter.counts_to_meters(500)
        assert result == pytest.approx(expected_circumference / 2, rel=1e-6)
        
        # Zero counts
        result = converter.counts_to_meters(0)
        assert result == 0.0
    
    def test_distance_conversion_consistency(self, converter):
        """Test round-trip consistency of distance conversions."""
        test_distances = [0.1, 0.5, 1.0, 2.5]
        
        for distance in test_distances:
            counts = converter.meters_to_counts(distance)
            recovered_distance = converter.counts_to_meters(counts)
            assert recovered_distance == pytest.approx(distance, rel=1e-3)


class TestConversionConsistency:
    """Test round-trip conversion consistency and accuracy."""
    
    @pytest.fixture
    def converter(self):
        """Standard converter for consistency testing."""
        return UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=1.0)
    
    def test_position_conversion_round_trip(self, converter):
        """Test round-trip position conversion consistency."""
        test_positions = [0, 100, 500, 1000, -500, 2000]
        
        for counts in test_positions:
            radians = converter.counts_to_radians(counts)
            recovered_counts = converter.radians_to_counts(radians)
            assert recovered_counts == counts
    
    def test_velocity_conversion_round_trip(self, converter):
        """Test round-trip velocity conversion consistency."""
        test_velocities = [0, 100, 500, 1000, -500, 2000]
        
        for counts_per_sec in test_velocities:
            rad_per_sec = converter.counts_per_sec_to_rad_per_sec(counts_per_sec)
            recovered_counts_per_sec = converter.rad_per_sec_to_counts_per_sec(rad_per_sec)
            assert recovered_counts_per_sec == counts_per_sec
    
    def test_floating_point_precision(self, converter):
        """Test handling of floating-point precision issues."""
        # Test very small values
        small_radians = 1e-10
        counts = converter.radians_to_counts(small_radians)
        recovered_radians = converter.counts_to_radians(counts)
        # Should be zero due to integer rounding
        assert counts == 0
        assert recovered_radians == 0.0
        
        # Test values that should round consistently
        # Use a larger value that won't round to zero
        precise_radians = math.pi / 100  # Larger value that converts to multiple counts
        counts = converter.radians_to_counts(precise_radians)
        recovered_radians = converter.counts_to_radians(counts)
        # Allow for rounding error due to integer conversion
        assert recovered_radians == pytest.approx(precise_radians, rel=1e-2)


class TestOverflowHandling:
    """Test integer overflow and underflow handling."""
    
    @pytest.fixture
    def converter(self):
        """Standard converter for overflow testing."""
        return UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=1.0)
    
    def test_radians_to_counts_overflow(self, converter):
        """Test overflow detection in radians to counts conversion."""
        # Very large radian value that would overflow 32-bit integer
        large_radians = 1e10
        
        with pytest.raises(OverflowError, match="exceeds 32-bit signed integer range"):
            converter.radians_to_counts(large_radians)
    
    def test_rad_per_sec_to_counts_per_sec_overflow(self, converter):
        """Test overflow detection in velocity conversion."""
        large_velocity = 1e10
        
        with pytest.raises(OverflowError, match="exceeds 32-bit signed integer range"):
            converter.rad_per_sec_to_counts_per_sec(large_velocity)
    
    def test_meters_to_counts_overflow(self, converter):
        """Test overflow detection in distance conversion."""
        large_distance = 1e10
        
        with pytest.raises(OverflowError, match="exceeding 32-bit range"):
            converter.meters_to_counts(large_distance)
    
    def test_validate_conversion_range(self, converter):
        """Test conversion range validation."""
        # Valid values should not raise exceptions
        converter.validate_conversion_range(1.0, "radians")
        converter.validate_conversion_range(1.0, "rad_per_sec")
        converter.validate_conversion_range(1.0, "meters")
        
        # Large values should raise OverflowError
        with pytest.raises(OverflowError):
            converter.validate_conversion_range(1e10, "radians")
        
        with pytest.raises(OverflowError):
            converter.validate_conversion_range(1e10, "rad_per_sec")
        
        with pytest.raises(OverflowError):
            converter.validate_conversion_range(1e10, "meters")


class TestUtilityMethods:
    """Test utility and informational methods."""
    
    @pytest.fixture
    def converter(self):
        """Standard converter for utility testing."""
        return UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=1.0)
    
    def test_get_conversion_info(self, converter):
        """Test conversion information retrieval."""
        info = converter.get_conversion_info()
        
        assert info["wheel_radius"] == 0.1
        assert info["encoder_counts_per_rev"] == 1000
        assert info["gear_ratio"] == 1.0
        assert info["wheel_circumference"] == pytest.approx(2 * math.pi * 0.1, rel=1e-6)
        assert "radians_per_count" in info
        assert "counts_per_radian" in info
        assert "counts_per_meter" in info
    
    def test_string_representations(self, converter):
        """Test string and repr methods."""
        str_repr = str(converter)
        assert "UnitConverter" in str_repr
        assert "wheel_radius=0.1" in str_repr
        assert "encoder_counts_per_rev=1000" in str_repr
        assert "gear_ratio=1.0" in str_repr
        
        repr_str = repr(converter)
        assert "UnitConverter" in repr_str
        assert "0.1" in repr_str
        assert "1000" in repr_str


class TestEdgeCases:
    """Test edge cases and boundary conditions."""
    
    def test_very_small_wheel(self):
        """Test converter with very small wheel radius."""
        converter = UnitConverter(wheel_radius=0.001, encoder_counts_per_rev=100, gear_ratio=1.0)
        
        # Should still work correctly
        result = converter.counts_to_radians(100)
        assert result == pytest.approx(2 * math.pi, rel=1e-6)
    
    def test_very_large_encoder_resolution(self):
        """Test converter with very high encoder resolution."""
        converter = UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000000, gear_ratio=1.0)
        
        # Should still work correctly
        result = converter.counts_to_radians(1000000)
        assert result == pytest.approx(2 * math.pi, rel=1e-6)
    
    def test_fractional_gear_ratio(self):
        """Test converter with fractional gear ratio."""
        converter = UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=1.5)
        
        # 1.5 gear ratio means 1500 counts per wheel revolution
        result = converter.counts_to_radians(1500)
        assert result == pytest.approx(2 * math.pi, rel=1e-6)
    
    def test_floating_point_inputs(self):
        """Test handling of floating-point inputs where integers expected."""
        converter = UnitConverter(wheel_radius=0.1, encoder_counts_per_rev=1000, gear_ratio=1.0)
        
        # counts_to_radians should handle float inputs
        result = converter.counts_to_radians(1000.5)
        expected = converter.counts_to_radians(1000) + converter.counts_to_radians(0.5)
        assert result == pytest.approx(expected, rel=1e-6)
        
        # counts_per_sec_to_rad_per_sec should handle float inputs
        result = converter.counts_per_sec_to_rad_per_sec(1000.5)
        expected = converter.counts_per_sec_to_rad_per_sec(1000) + converter.counts_per_sec_to_rad_per_sec(0.5)
        assert result == pytest.approx(expected, rel=1e-6)


if __name__ == "__main__":
    # Run tests with verbose output
    pytest.main([__file__, "-v"])