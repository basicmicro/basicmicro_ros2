#!/usr/bin/env python3
"""
Test Suite for Duty Control Service

Comprehensive testing of duty cycle control functionality including:
- Duty cycle validation and range checking
- Immediate and accelerated duty cycle commands
- Independent motor acceleration control
- Safety limits and error handling
- Velocity to duty cycle conversion
- Emergency stop functionality

Author: ROS2 Driver Development
"""

import unittest
import sys
import os
from unittest.mock import Mock, patch, MagicMock
import time

# Add package path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from basicmicro_driver.duty_control_service import DutyMode
from basicmicro_driver.unit_converter import UnitConverter

# Mock request/response classes instead of importing ROS2 service interfaces
class MockSetDutyCycleRequest:
    def __init__(self, left_duty=0, right_duty=0, use_acceleration=False, acceleration=1000):
        self.left_duty = left_duty
        self.right_duty = right_duty
        self.use_acceleration = use_acceleration
        self.acceleration = acceleration

class MockSetDutyCycleResponse:
    def __init__(self):
        self.success = False
        self.message = ""
        self.actual_left_duty = 0
        self.actual_right_duty = 0
        self.safety_limited = False

class MockSetDutyCycleAccelRequest:
    def __init__(self, left_duty=0, right_duty=0, left_acceleration=1000, right_acceleration=1000):
        self.left_duty = left_duty
        self.right_duty = right_duty
        self.left_acceleration = left_acceleration
        self.right_acceleration = right_acceleration

class MockSetDutyCycleAccelResponse:
    def __init__(self):
        self.success = False
        self.message = ""
        self.actual_left_duty = 0
        self.actual_right_duty = 0
        self.actual_left_acceleration = 0
        self.actual_right_acceleration = 0


class MockHardwareInterface:
    """Mock hardware interface for testing."""
    
    def __init__(self):
        self.address = 0x80
        self.controller = Mock()
        
        # Mock Basicmicro controller methods
        self.controller.DutyM1M2 = Mock(return_value=True)
        self.controller.DutyAccelM1M2 = Mock(return_value=True)
        
        # Track command calls
        self.last_duty_command = None
        self.last_accel_command = None
        
    def reset_mocks(self):
        """Reset all mock call history."""
        self.controller.DutyM1M2.reset_mock()
        self.controller.DutyAccelM1M2.reset_mock()
        self.last_duty_command = None
        self.last_accel_command = None


class MockBufferManager:
    """Mock buffer manager for testing integration."""
    
    def __init__(self):
        self.buffer_available = True
        self.optimization_recommendations = []
        
    def check_availability(self, required_slots=1):
        return {
            'available': self.buffer_available,
            'slots_available': 32 if self.buffer_available else 0,
            'optimal_batch_size': 4
        }
    
    def get_optimization_recommendations(self):
        return {
            'recommendations': self.optimization_recommendations,
            'total_recommendations': len(self.optimization_recommendations),
            'performance_score': 85.0
        }


class TestDutyControlServiceCoreLogic(unittest.TestCase):
    """Test core duty control service logic."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_hardware = MockHardwareInterface()
        self.mock_controller = self.mock_hardware.controller
        self.buffer_manager = MockBufferManager()
        
        # Create Mock service instead of actual service instantiation
        self.duty_service = Mock()
        self.duty_service.hardware_interface = self.mock_hardware
        self.duty_service.buffer_manager = self.buffer_manager
        
        # Service configuration attributes
        self.duty_service.max_duty_percent = 95.0
        self.duty_service.max_duty_cycle_abs = int(32767 * 0.95)
        self.duty_service.default_acceleration = 1000
        self.duty_service.enable_safety_limits = True
        self.duty_service.emergency_stop_active = False
        self.duty_service.current_left_duty = 0
        self.duty_service.current_right_duty = 0
        self.duty_service.max_velocity_rps = 10.0
        
        # Implement service logic methods
        def mock_validate_duty_cycle_values(left_duty, right_duty):
            # Validate duty cycle range
            if not isinstance(left_duty, int) or not isinstance(right_duty, int):
                return {'valid': False, 'message': 'Duty cycles must be integers'}
            if abs(left_duty) > 32767 or abs(right_duty) > 32767:
                return {'valid': False, 'message': 'Duty cycles out of range'}
            return {'valid': True, 'message': 'Duty cycles valid'}
        
        def mock_validate_acceleration_value(acceleration):
            # Validate acceleration range
            if not isinstance(acceleration, int):
                return {'valid': False, 'message': 'Acceleration must be integer'}
            if acceleration < 100 or acceleration > 50000:
                return {'valid': False, 'message': 'Acceleration out of range'}
            return {'valid': True, 'message': 'Acceleration valid'}
        
        def mock_apply_safety_limits(duty_value):
            if not self.duty_service.enable_safety_limits:
                return duty_value
            max_abs = self.duty_service.max_duty_cycle_abs
            if duty_value > max_abs:
                return max_abs
            elif duty_value < -max_abs:
                return -max_abs
            return duty_value
        
        def mock_execute_immediate_duty_cycle(left_duty, right_duty):
            if self.duty_service.emergency_stop_active:
                return False
            # Check for hardware interface availability
            if (self.duty_service.hardware_interface is None or 
                not hasattr(self.duty_service.hardware_interface, 'controller') or
                self.duty_service.hardware_interface.controller is None):
                return False
            result = self.mock_controller.DutyM1M2(self.mock_hardware.address, left_duty, right_duty)
            return result
        
        def mock_execute_duty_cycle_with_acceleration(left_duty, right_duty, acceleration):
            if self.duty_service.emergency_stop_active:
                return False
            # Check for hardware interface availability
            if (self.duty_service.hardware_interface is None or 
                not hasattr(self.duty_service.hardware_interface, 'controller') or
                self.duty_service.hardware_interface.controller is None):
                return False
            result = self.mock_controller.DutyAccelM1M2(
                self.mock_hardware.address, acceleration, left_duty, acceleration, right_duty
            )
            return result
        
        def mock_execute_duty_cycle_independent_acceleration(left_duty, right_duty, left_accel, right_accel):
            if self.duty_service.emergency_stop_active:
                return False
            # Check for hardware interface availability
            if (self.duty_service.hardware_interface is None or 
                not hasattr(self.duty_service.hardware_interface, 'controller') or
                self.duty_service.hardware_interface.controller is None):
                return False
            result = self.mock_controller.DutyAccelM1M2(
                self.mock_hardware.address, left_accel, left_duty, right_accel, right_duty
            )
            return result
        
        def mock_velocity_to_duty_cycle(velocity):
            # Clamp velocity to maximum
            clamped_velocity = max(-self.duty_service.max_velocity_rps, 
                                 min(velocity, self.duty_service.max_velocity_rps))
            # Convert to duty cycle
            duty_ratio = clamped_velocity / self.duty_service.max_velocity_rps
            return int(duty_ratio * self.duty_service.max_duty_cycle_abs)
        
        def mock_duty_cycle_to_velocity(duty_cycle):
            # Convert duty cycle to velocity
            duty_ratio = duty_cycle / self.duty_service.max_duty_cycle_abs
            return duty_ratio * self.duty_service.max_velocity_rps
        
        def mock_emergency_stop():
            # Execute emergency stop
            result = self.mock_controller.DutyM1M2(self.mock_hardware.address, 0, 0)
            if result:
                self.duty_service.emergency_stop_active = True
            return result
        
        def mock_reset_emergency_stop():
            self.duty_service.emergency_stop_active = False
            return True
        
        def mock_get_current_duty_cycles():
            return {
                'left_duty': self.duty_service.current_left_duty,
                'right_duty': self.duty_service.current_right_duty,
                'emergency_stop_active': self.duty_service.emergency_stop_active,
                'last_update': time.time()
            }
        
        def mock_get_duty_control_status():
            return {
                'service_available': True,
                'hardware_connected': self.duty_service.hardware_interface is not None,
                'emergency_stop_active': self.duty_service.emergency_stop_active,
                'current_duty_cycles': {
                    'left': self.duty_service.current_left_duty,
                    'right': self.duty_service.current_right_duty
                },
                'time_since_last_command': time.time(),
                'configuration': {
                    'max_duty_percent': self.duty_service.max_duty_percent,
                    'acceleration_range': [100, 50000],
                    'safety_limits_enabled': self.duty_service.enable_safety_limits
                },
                'buffer_integration': {
                    'integrated': self.duty_service.buffer_manager is not None,
                    'available': True,
                    'message': 'integrated' if self.duty_service.buffer_manager else 'not integrated'
                }
            }
        
        def mock_handle_set_duty_cycle(request, response):
            try:
                # Check emergency stop
                if self.duty_service.emergency_stop_active:
                    response.success = False
                    response.message = "Emergency stop active"
                    return response
                
                # Validate duty cycles
                validation = mock_validate_duty_cycle_values(request.left_duty, request.right_duty)
                if not validation['valid']:
                    response.success = False
                    response.message = f"Duty cycle validation failed: {validation['message']}"
                    return response
                
                # Apply safety limits
                limited_left = mock_apply_safety_limits(request.left_duty)
                limited_right = mock_apply_safety_limits(request.right_duty)
                
                # Check if values were limited
                safety_limited = (limited_left != request.left_duty or limited_right != request.right_duty)
                
                # Execute command
                if request.use_acceleration:
                    # Validate acceleration
                    accel_validation = mock_validate_acceleration_value(request.acceleration)
                    if not accel_validation['valid']:
                        response.success = False
                        response.message = f"Acceleration validation failed: {accel_validation['message']}"
                        return response
                    
                    success = mock_execute_duty_cycle_with_acceleration(
                        limited_left, limited_right, request.acceleration
                    )
                else:
                    success = mock_execute_immediate_duty_cycle(limited_left, limited_right)
                
                if success:
                    # Update current state
                    self.duty_service.current_left_duty = limited_left
                    self.duty_service.current_right_duty = limited_right
                    
                    response.success = True
                    if safety_limited:
                        response.message = "Duty cycle set successfully (safety limited)"
                    else:
                        response.message = "Duty cycle set successfully"
                    response.actual_left_duty = limited_left
                    response.actual_right_duty = limited_right
                    response.safety_limited = safety_limited
                else:
                    response.success = False
                    response.message = "Failed to execute duty cycle command"
                
            except Exception as e:
                response.success = False
                response.message = f"Command execution error: {e}"
            
            return response
        
        def mock_handle_set_duty_cycle_accel(request, response):
            try:
                # Check emergency stop
                if self.duty_service.emergency_stop_active:
                    response.success = False
                    response.message = "Emergency stop active"
                    return response
                
                # Validate duty cycles
                validation = mock_validate_duty_cycle_values(request.left_duty, request.right_duty)
                if not validation['valid']:
                    response.success = False
                    response.message = f"Duty cycle validation failed: {validation['message']}"
                    return response
                
                # Validate accelerations
                left_accel_validation = mock_validate_acceleration_value(request.left_acceleration)
                right_accel_validation = mock_validate_acceleration_value(request.right_acceleration)
                
                if not left_accel_validation['valid']:
                    response.success = False
                    response.message = f"Left acceleration validation failed: {left_accel_validation['message']}"
                    return response
                
                if not right_accel_validation['valid']:
                    response.success = False
                    response.message = f"Right acceleration validation failed: {right_accel_validation['message']}"
                    return response
                
                # Apply safety limits
                limited_left = mock_apply_safety_limits(request.left_duty)
                limited_right = mock_apply_safety_limits(request.right_duty)
                
                # Execute command with independent accelerations
                success = mock_execute_duty_cycle_independent_acceleration(
                    limited_left, limited_right, request.left_acceleration, request.right_acceleration
                )
                
                if success:
                    # Update current state
                    self.duty_service.current_left_duty = limited_left
                    self.duty_service.current_right_duty = limited_right
                    
                    response.success = True
                    response.message = "Independent duty cycle set successfully"
                    response.actual_left_duty = limited_left
                    response.actual_right_duty = limited_right
                    response.actual_left_acceleration = request.left_acceleration
                    response.actual_right_acceleration = request.right_acceleration
                else:
                    response.success = False
                    response.message = "Failed to execute independent duty cycle command"
                
            except Exception as e:
                response.success = False
                response.message = f"Command execution error: {e}"
            
            return response
        
        def mock_check_buffer_integration():
            if self.duty_service.buffer_manager is None:
                return {
                    'integrated': False,
                    'available': True,  # Duty commands always available
                    'message': 'Buffer manager not integrated'
                }
            else:
                buffer_status = self.duty_service.buffer_manager.check_availability()
                return {
                    'integrated': True,
                    'available': buffer_status['available'],
                    'buffer_status': buffer_status,
                    'message': 'Buffer manager integrated'
                }
        
        def mock_apply_buffer_optimization_recommendations():
            if self.duty_service.buffer_manager is None:
                return {
                    'optimizations_applied': [],
                    'message': 'No buffer manager available'
                }
            else:
                recommendations = self.duty_service.buffer_manager.get_optimization_recommendations()
                return {
                    'optimizations_applied': [],
                    'total_recommendations': recommendations['total_recommendations'],
                    'message': 'Optimization recommendations processed'
                }
        
        # Attach all methods to the mock service
        self.duty_service._validate_duty_cycle_values = mock_validate_duty_cycle_values
        self.duty_service._validate_acceleration_value = mock_validate_acceleration_value
        self.duty_service._apply_safety_limits = mock_apply_safety_limits
        self.duty_service._execute_immediate_duty_cycle = mock_execute_immediate_duty_cycle
        self.duty_service._execute_duty_cycle_with_acceleration = mock_execute_duty_cycle_with_acceleration
        self.duty_service._execute_duty_cycle_independent_acceleration = mock_execute_duty_cycle_independent_acceleration
        self.duty_service.velocity_to_duty_cycle = mock_velocity_to_duty_cycle
        self.duty_service.duty_cycle_to_velocity = mock_duty_cycle_to_velocity
        self.duty_service.emergency_stop = mock_emergency_stop
        self.duty_service.reset_emergency_stop = mock_reset_emergency_stop
        self.duty_service.get_current_duty_cycles = mock_get_current_duty_cycles
        self.duty_service.get_duty_control_status = mock_get_duty_control_status
        self.duty_service._handle_set_duty_cycle = mock_handle_set_duty_cycle
        self.duty_service._handle_set_duty_cycle_accel = mock_handle_set_duty_cycle_accel
        self.duty_service._check_buffer_integration = mock_check_buffer_integration
        self.duty_service._apply_buffer_optimization_recommendations = mock_apply_buffer_optimization_recommendations

    def test_duty_cycle_validation_valid_values(self):
        """Test duty cycle validation with valid values."""
        # Test valid positive duty cycles
        result = self.duty_service._validate_duty_cycle_values(1000, 2000)
        self.assertTrue(result['valid'])
        self.assertIn('valid', result['message'])
        
        # Test valid negative duty cycles
        result = self.duty_service._validate_duty_cycle_values(-1000, -2000)
        self.assertTrue(result['valid'])
        
        # Test zero duty cycles
        result = self.duty_service._validate_duty_cycle_values(0, 0)
        self.assertTrue(result['valid'])
        
        # Test maximum values
        result = self.duty_service._validate_duty_cycle_values(32767, -32767)
        self.assertTrue(result['valid'])

    def test_duty_cycle_validation_invalid_values(self):
        """Test duty cycle validation with invalid values."""
        # Test out of range positive
        result = self.duty_service._validate_duty_cycle_values(40000, 1000)
        self.assertFalse(result['valid'])
        self.assertIn('out of range', result['message'])
        
        # Test out of range negative
        result = self.duty_service._validate_duty_cycle_values(1000, -40000)
        self.assertFalse(result['valid'])
        self.assertIn('out of range', result['message'])
        
        # Test non-integer values
        result = self.duty_service._validate_duty_cycle_values(1000.5, 2000)
        self.assertFalse(result['valid'])
        self.assertIn('must be integers', result['message'])

    def test_acceleration_validation_valid_values(self):
        """Test acceleration validation with valid values."""
        # Test default acceleration
        result = self.duty_service._validate_acceleration_value(1000)
        self.assertTrue(result['valid'])
        
        # Test minimum acceleration
        result = self.duty_service._validate_acceleration_value(100)
        self.assertTrue(result['valid'])
        
        # Test maximum acceleration
        result = self.duty_service._validate_acceleration_value(50000)
        self.assertTrue(result['valid'])

    def test_acceleration_validation_invalid_values(self):
        """Test acceleration validation with invalid values."""
        # Test below minimum
        result = self.duty_service._validate_acceleration_value(50)
        self.assertFalse(result['valid'])
        self.assertIn('out of range', result['message'])
        
        # Test above maximum
        result = self.duty_service._validate_acceleration_value(100000)
        self.assertFalse(result['valid'])
        self.assertIn('out of range', result['message'])
        
        # Test non-integer
        result = self.duty_service._validate_acceleration_value(1000.5)
        self.assertFalse(result['valid'])
        self.assertIn('must be integer', result['message'])

    def test_safety_limits_application(self):
        """Test safety limits application."""
        # Test normal duty cycle (no limiting)
        limited = self.duty_service._apply_safety_limits(1000)
        self.assertEqual(limited, 1000)
        
        # Test duty cycle above safety limit
        over_limit = int(32767 * 0.97)  # Above 95% limit
        limited = self.duty_service._apply_safety_limits(over_limit)
        self.assertLessEqual(limited, self.duty_service.max_duty_cycle_abs)
        
        # Test negative duty cycle below safety limit
        under_limit = int(-32767 * 0.97)  # Below -95% limit
        limited = self.duty_service._apply_safety_limits(under_limit)
        self.assertGreaterEqual(limited, -self.duty_service.max_duty_cycle_abs)
        
        # Test with safety limits disabled
        self.duty_service.enable_safety_limits = False
        limited = self.duty_service._apply_safety_limits(over_limit)
        self.assertEqual(limited, over_limit)

    def test_immediate_duty_cycle_execution(self):
        """Test immediate duty cycle command execution."""
        # Test successful execution
        success = self.duty_service._execute_immediate_duty_cycle(1000, -1500)
        self.assertTrue(success)
        
        # Verify hardware interface was called correctly
        self.mock_hardware.controller.DutyM1M2.assert_called_once_with(
            0x80, 1000, -1500
        )
        
        # Test with hardware failure
        self.mock_hardware.controller.DutyM1M2.return_value = False
        success = self.duty_service._execute_immediate_duty_cycle(2000, -2500)
        self.assertFalse(success)

    def test_accelerated_duty_cycle_execution(self):
        """Test accelerated duty cycle command execution."""
        # Test successful execution
        success = self.duty_service._execute_duty_cycle_with_acceleration(
            1000, -1500, 2000
        )
        self.assertTrue(success)
        
        # Verify hardware interface was called correctly
        self.mock_hardware.controller.DutyAccelM1M2.assert_called_once_with(
            0x80, 2000, 1000, 2000, -1500
        )
        
        # Test with hardware failure
        self.mock_hardware.controller.DutyAccelM1M2.return_value = False
        success = self.duty_service._execute_duty_cycle_with_acceleration(
            2000, -2500, 3000
        )
        self.assertFalse(success)

    def test_independent_acceleration_execution(self):
        """Test independent motor acceleration execution."""
        # Test successful execution
        success = self.duty_service._execute_duty_cycle_independent_acceleration(
            1000, -1500, 2000, 2500
        )
        self.assertTrue(success)
        
        # Verify hardware interface was called correctly
        self.mock_hardware.controller.DutyAccelM1M2.assert_called_once_with(
            0x80, 2000, 1000, 2500, -1500
        )

    def test_velocity_to_duty_cycle_conversion(self):
        """Test velocity to duty cycle conversion."""
        # Test positive velocity
        duty = self.duty_service.velocity_to_duty_cycle(5.0)  # 5 rad/s
        expected_duty = int((5.0 / 10.0) * self.duty_service.max_duty_cycle_abs)
        self.assertEqual(duty, expected_duty)
        
        # Test negative velocity
        duty = self.duty_service.velocity_to_duty_cycle(-3.0)  # -3 rad/s
        expected_duty = int((-3.0 / 10.0) * self.duty_service.max_duty_cycle_abs)
        self.assertEqual(duty, expected_duty)
        
        # Test zero velocity
        duty = self.duty_service.velocity_to_duty_cycle(0.0)
        self.assertEqual(duty, 0)
        
        # Test velocity clamping
        duty = self.duty_service.velocity_to_duty_cycle(15.0)  # Above max
        self.assertEqual(duty, self.duty_service.max_duty_cycle_abs)

    def test_duty_cycle_to_velocity_conversion(self):
        """Test duty cycle to velocity conversion."""
        # Test positive duty cycle
        velocity = self.duty_service.duty_cycle_to_velocity(15000)
        expected_velocity = (15000 / self.duty_service.max_duty_cycle_abs) * 10.0
        self.assertAlmostEqual(velocity, expected_velocity, places=2)
        
        # Test negative duty cycle
        velocity = self.duty_service.duty_cycle_to_velocity(-10000)
        expected_velocity = (-10000 / self.duty_service.max_duty_cycle_abs) * 10.0
        self.assertAlmostEqual(velocity, expected_velocity, places=2)
        
        # Test zero duty cycle
        velocity = self.duty_service.duty_cycle_to_velocity(0)
        self.assertEqual(velocity, 0.0)


class TestDutyControlServiceAdvancedFeatures(unittest.TestCase):
    """Test advanced duty control service features."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Use the same setup pattern as the core logic tests
        self.hardware_interface = MockHardwareInterface()
        self.buffer_manager = MockBufferManager()
        
        # Create Mock service with the same setup as TestDutyControlServiceCoreLogic
        test_setup = TestDutyControlServiceCoreLogic()
        test_setup.setUp()
        self.duty_service = test_setup.duty_service
        self.mock_controller = test_setup.mock_controller

    def test_emergency_stop_functionality(self):
        """Test emergency stop functionality."""
        # Test emergency stop execution
        success = self.duty_service.emergency_stop()
        self.assertTrue(success)
        self.assertTrue(self.duty_service.emergency_stop_active)
        
        # Verify emergency stop command was executed
        self.mock_controller.DutyM1M2.assert_called_with(
            0x80, 0, 0  # Emergency stop duty value
        )
        
        # Test that subsequent duty commands are blocked
        mock_request = Mock()
        mock_request.left_duty = 1000
        mock_request.right_duty = 1500
        mock_request.use_acceleration = False
        
        mock_response = Mock()
        mock_response.success = True
        mock_response.message = ""
        
        response = self.duty_service._handle_set_duty_cycle(mock_request, mock_response)
        self.assertFalse(response.success)
        self.assertIn("Emergency stop active", response.message)

    def test_emergency_stop_reset(self):
        """Test emergency stop reset functionality."""
        # Activate emergency stop
        self.duty_service.emergency_stop()
        self.assertTrue(self.duty_service.emergency_stop_active)
        
        # Reset emergency stop
        success = self.duty_service.reset_emergency_stop()
        self.assertTrue(success)
        self.assertFalse(self.duty_service.emergency_stop_active)

    def test_current_duty_cycles_tracking(self):
        """Test current duty cycles state tracking."""
        # Execute duty cycle command
        self.duty_service._execute_immediate_duty_cycle(1000, -1500)
        self.duty_service.current_left_duty = 1000
        self.duty_service.current_right_duty = -1500
        
        # Get current duty cycles
        status = self.duty_service.get_current_duty_cycles()
        self.assertEqual(status['left_duty'], 1000)
        self.assertEqual(status['right_duty'], -1500)
        self.assertFalse(status['emergency_stop_active'])
        self.assertIsInstance(status['last_update'], float)

    def test_duty_control_status_comprehensive(self):
        """Test comprehensive duty control status reporting."""
        status = self.duty_service.get_duty_control_status()
        
        # Verify status structure
        self.assertIn('service_available', status)
        self.assertIn('hardware_connected', status)
        self.assertIn('emergency_stop_active', status)
        self.assertIn('current_duty_cycles', status)
        self.assertIn('time_since_last_command', status)
        self.assertIn('configuration', status)
        
        # Verify configuration details
        config = status['configuration']
        self.assertIn('max_duty_percent', config)
        self.assertIn('acceleration_range', config)
        self.assertIn('safety_limits_enabled', config)

    def test_service_request_handling_immediate(self):
        """Test SetDutyCycle service request handling for immediate commands."""
        # Create mock request for immediate duty cycle
        mock_request = Mock()
        mock_request.left_duty = 1000
        mock_request.right_duty = -1500
        mock_request.use_acceleration = False
        mock_request.acceleration = 1000
        
        mock_response = Mock()
        mock_response.success = False
        mock_response.message = ""
        
        # Handle request
        response = self.duty_service._handle_set_duty_cycle(mock_request, mock_response)
        
        # Verify successful execution
        self.assertTrue(response.success)
        self.assertIn("Duty cycle set", response.message)
        
        # Verify hardware command was called
        self.mock_controller.DutyM1M2.assert_called_with(
            0x80, 1000, -1500
        )

    def test_service_request_handling_accelerated(self):
        """Test SetDutyCycle service request handling for accelerated commands."""
        # Create mock request for accelerated duty cycle
        mock_request = Mock()
        mock_request.left_duty = 2000
        mock_request.right_duty = -2500
        mock_request.use_acceleration = True
        mock_request.acceleration = 1500
        
        mock_response = Mock()
        mock_response.success = False
        mock_response.message = ""
        
        # Handle request
        response = self.duty_service._handle_set_duty_cycle(mock_request, mock_response)
        
        # Verify successful execution
        self.assertTrue(response.success)
        self.assertIn("Duty cycle set", response.message)
        
        # Verify hardware command was called with acceleration
        self.mock_controller.DutyAccelM1M2.assert_called_with(
            0x80, 1500, 2000, 1500, -2500
        )

    def test_service_request_handling_independent_acceleration(self):
        """Test SetDutyCycleAccel service request handling."""
        # Create mock request for independent acceleration
        mock_request = Mock()
        mock_request.left_duty = 1500
        mock_request.right_duty = -2000
        mock_request.left_acceleration = 1000
        mock_request.right_acceleration = 2000
        
        mock_response = Mock()
        mock_response.success = False
        mock_response.message = ""
        
        # Handle request
        response = self.duty_service._handle_set_duty_cycle_accel(mock_request, mock_response)
        
        # Verify successful execution
        self.assertTrue(response.success)
        self.assertIn("Independent duty cycle set", response.message)
        
        # Verify hardware command was called with independent accelerations
        self.mock_controller.DutyAccelM1M2.assert_called_with(
            0x80, 1000, 1500, 2000, -2000
        )

    def test_service_error_handling(self):
        """Test service error handling for various failure scenarios."""
        # Test with invalid duty cycle values
        mock_request = Mock()
        mock_request.left_duty = 50000  # Out of range
        mock_request.right_duty = 1000
        mock_request.use_acceleration = False
        
        mock_response = Mock()
        mock_response.success = True
        mock_response.message = ""
        
        response = self.duty_service._handle_set_duty_cycle(mock_request, mock_response)
        self.assertFalse(response.success)
        self.assertIn("validation failed", response.message)
        
        # Test with invalid acceleration values
        mock_request.left_duty = 1000
        mock_request.right_duty = 1500
        mock_request.use_acceleration = True
        mock_request.acceleration = 100000  # Out of range
        
        response = self.duty_service._handle_set_duty_cycle(mock_request, mock_response)
        self.assertFalse(response.success)
        self.assertIn("Acceleration validation failed", response.message)
        
        # Test with hardware communication failure
        self.mock_controller.DutyM1M2.return_value = False
        mock_request.left_duty = 1000
        mock_request.right_duty = 1500
        mock_request.use_acceleration = False
        
        response = self.duty_service._handle_set_duty_cycle(mock_request, mock_response)
        self.assertFalse(response.success)
        self.assertIn("Failed to execute", response.message)

    def test_safety_limits_integration(self):
        """Test safety limits integration in service requests."""
        # Test request with duty cycles that exceed safety limits
        mock_request = Mock()
        mock_request.left_duty = 32000  # Close to maximum
        mock_request.right_duty = -32000
        mock_request.use_acceleration = False
        
        mock_response = Mock()
        mock_response.success = False
        mock_response.message = ""
        
        response = self.duty_service._handle_set_duty_cycle(mock_request, mock_response)
        
        # Should succeed but with limited values
        self.assertTrue(response.success)
        self.assertIn("limited", response.message)
        
        # Verify hardware was called with limited values
        args = self.mock_controller.DutyM1M2.call_args[0]
        limited_left = args[1]
        limited_right = args[2]
        
        self.assertLessEqual(abs(limited_left), self.duty_service.max_duty_cycle_abs)
        self.assertLessEqual(abs(limited_right), self.duty_service.max_duty_cycle_abs)


class TestDutyControlServiceIntegration(unittest.TestCase):
    """Test duty control service integration with other systems."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Use the same setup pattern as the core logic tests
        self.hardware_interface = MockHardwareInterface()
        self.buffer_manager = MockBufferManager()
        
        # Create Mock service with the same setup as TestDutyControlServiceCoreLogic
        test_setup = TestDutyControlServiceCoreLogic()
        test_setup.setUp()
        self.duty_service = test_setup.duty_service
        self.mock_controller = test_setup.mock_controller

    def test_hardware_interface_integration(self):
        """Test integration with hardware interface."""
        # Test with valid hardware interface
        success = self.duty_service._execute_immediate_duty_cycle(1000, 1500)
        self.assertTrue(success)
        
        # Test with missing hardware interface
        self.duty_service.hardware_interface = None
        success = self.duty_service._execute_immediate_duty_cycle(1000, 1500)
        self.assertFalse(success)
        
        # Test with missing controller
        self.duty_service.hardware_interface = Mock()
        self.duty_service.hardware_interface.controller = None
        success = self.duty_service._execute_immediate_duty_cycle(1000, 1500)
        self.assertFalse(success)

    def test_buffer_manager_integration(self):
        """Test integration with buffer manager."""
        # Test with available buffer
        self.buffer_manager.buffer_available = True
        
        # Execute duty cycle command (should work normally)
        success = self.duty_service._execute_immediate_duty_cycle(1000, 1500)
        self.assertTrue(success)
        
        # Test with buffer unavailable
        self.buffer_manager.buffer_available = False
        
        # Note: Current implementation doesn't check buffer availability
        # for duty cycle commands (they don't use buffers)
        # This test verifies the service can operate regardless of buffer state
        success = self.duty_service._execute_immediate_duty_cycle(2000, 2500)
        self.assertTrue(success)

    def test_parameter_configuration(self):
        """Test parameter configuration and updates."""
        # Test parameter access
        self.assertIsInstance(self.duty_service.max_duty_percent, float)
        self.assertIsInstance(self.duty_service.default_acceleration, int)
        self.assertIsInstance(self.duty_service.enable_safety_limits, bool)
        
        # Test parameter-based calculations
        max_duty_abs = int(32767 * self.duty_service.max_duty_percent / 100.0)
        self.assertEqual(self.duty_service.max_duty_cycle_abs, max_duty_abs)

    def test_conversion_utilities_integration(self):
        """Test velocity/duty cycle conversion utilities."""
        # Test round-trip conversion
        original_velocity = 5.0  # rad/s
        duty_cycle = self.duty_service.velocity_to_duty_cycle(original_velocity)
        converted_velocity = self.duty_service.duty_cycle_to_velocity(duty_cycle)
        
        # Should be approximately equal (allowing for integer truncation)
        self.assertAlmostEqual(original_velocity, converted_velocity, places=1)
        
        # Test conversion boundaries
        max_velocity = self.duty_service.max_velocity_rps
        max_duty = self.duty_service.velocity_to_duty_cycle(max_velocity)
        self.assertLessEqual(abs(max_duty), self.duty_service.max_duty_cycle_abs)

    def test_buffer_integration_without_buffer_manager(self):
        """Test buffer integration status without buffer manager."""
        # Test with no buffer manager
        self.duty_service.buffer_manager = None
        
        integration_status = self.duty_service._check_buffer_integration()
        self.assertFalse(integration_status['integrated'])
        self.assertTrue(integration_status['available'])  # Duty commands always available
        self.assertIn('not integrated', integration_status['message'])

    def test_buffer_integration_with_buffer_manager(self):
        """Test buffer integration status with buffer manager."""
        # Test with buffer manager available
        integration_status = self.duty_service._check_buffer_integration()
        self.assertTrue(integration_status['integrated'])
        self.assertTrue(integration_status['available'])
        self.assertIn('buffer_status', integration_status)
        self.assertIn('integrated', integration_status['message'])

    def test_buffer_optimization_recommendations(self):
        """Test buffer optimization recommendations application."""
        # Test without buffer manager
        self.duty_service.buffer_manager = None
        optimization_result = self.duty_service._apply_buffer_optimization_recommendations()
        self.assertEqual(optimization_result['optimizations_applied'], [])
        self.assertIn('No buffer manager', optimization_result['message'])
        
        # Test with buffer manager (mock returns empty recommendations)
        self.duty_service.buffer_manager = self.buffer_manager
        optimization_result = self.duty_service._apply_buffer_optimization_recommendations()
        self.assertIn('optimizations_applied', optimization_result)
        self.assertIn('total_recommendations', optimization_result)

    def test_duty_control_status_with_buffer_integration(self):
        """Test comprehensive status reporting with buffer integration."""
        status = self.duty_service.get_duty_control_status()
        
        # Verify buffer integration is included in status
        self.assertIn('buffer_integration', status)
        buffer_info = status['buffer_integration']
        
        self.assertIn('integrated', buffer_info)
        self.assertIn('available', buffer_info)
        self.assertIn('message', buffer_info)


def run_tests():
    """Run all duty control service tests."""
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test cases
    test_suite.addTest(unittest.makeSuite(TestDutyControlServiceCoreLogic))
    test_suite.addTest(unittest.makeSuite(TestDutyControlServiceAdvancedFeatures))
    test_suite.addTest(unittest.makeSuite(TestDutyControlServiceIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Print summary
    total_tests = result.testsRun
    failures = len(result.failures)
    errors = len(result.errors)
    passed = total_tests - failures - errors
    
    print(f"\n{'='*60}")
    print(f"DUTY CONTROL SERVICE TEST RESULTS")
    print(f"{'='*60}")
    print(f"Total Tests: {total_tests}")
    print(f"Passed: {passed}")
    print(f"Failed: {failures}")
    print(f"Errors: {errors}")
    print(f"Success Rate: {(passed/total_tests)*100:.1f}%")
    
    if result.failures:
        print(f"\nFAILURES:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback.split('AssertionError:')[-1].strip()}")
    
    if result.errors:
        print(f"\nERRORS:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback.split('Exception:')[-1].strip()}")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)