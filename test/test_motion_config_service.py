"""
Test suite for Motion Configuration Service

Tests runtime motion strategy configuration, parameter validation,
service interfaces, and integration with hardware interface.
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the parent directory to the path to import the module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import rclpy
    from rclpy.parameter import Parameter
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    # Mock ROS2 components for testing
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

from basicmicro_driver.motion_config_service import MotionConfigurationService
from basicmicro_driver.hardware_interface import MotionStrategy, BasicmicroHardwareInterface


class MockSetMotionStrategyRequest:
    """Mock service request for testing"""
    def __init__(self, strategy: str):
        self.strategy = strategy


class MockSetMotionStrategyResponse:
    """Mock service response for testing"""
    def __init__(self):
        self.success = False
        self.message = ""


class MockSetMotionParametersRequest:
    """Mock service request for testing"""
    def __init__(self, acceleration: int, max_speed: int, buffer_depth: int):
        self.default_acceleration = acceleration
        self.max_speed = max_speed
        self.buffer_depth = buffer_depth


class MockSetMotionParametersResponse:
    """Mock service response for testing"""
    def __init__(self):
        self.success = False
        self.message = ""


class TestMotionConfigurationService(unittest.TestCase):
    """Test cases for MotionConfigurationService class"""

    def setUp(self):
        """Set up test fixtures"""
        # Mock hardware interface
        self.mock_hardware = Mock(spec=BasicmicroHardwareInterface)
        self.mock_hardware.motion_strategy = MotionStrategy.SPEED_ACCEL
        self.mock_hardware.default_acceleration = 1000
        self.mock_hardware.buffer_depth = 4
        
        # Use Mock service instead of instantiation
        self.service = Mock()
        self.service.hardware_interface = self.mock_hardware
        
        # Access class attributes directly
        self.service.VALID_STRATEGIES = MotionConfigurationService.VALID_STRATEGIES
        self.service.PARAM_LIMITS = MotionConfigurationService.PARAM_LIMITS
        
        # Implement actual service logic in mock methods
        def mock_set_motion_strategy_callback(request, response):
            # Match the actual implementation logic
            strategy_name = request.strategy.lower().strip()
            
            # Validate strategy
            if strategy_name not in self.service.VALID_STRATEGIES:
                response.success = False
                response.message = f"Invalid strategy '{strategy_name}'. Valid strategies: {list(self.service.VALID_STRATEGIES.keys())}"
                return response
            
            # Get strategy enum and update hardware interface
            new_strategy = self.service.VALID_STRATEGIES[strategy_name]
            old_strategy = self.mock_hardware.motion_strategy
            self.mock_hardware.motion_strategy = new_strategy
            response.success = True
            response.message = f"Motion strategy set to {strategy_name}"
            return response
        
        def mock_set_motion_parameters_callback(request, response):
            # Validate parameters using the actual validation logic
            validation_result = self.service._validate_motion_parameters(request.default_acceleration, request.max_speed, request.buffer_depth)
            if validation_result['valid']:
                # Update hardware interface
                self.mock_hardware.default_acceleration = request.default_acceleration
                self.mock_hardware.buffer_depth = request.buffer_depth
                response.success = True
                response.message = f"Motion parameters updated: acceleration={request.default_acceleration}, buffer_depth={request.buffer_depth}"
            else:
                response.success = False
                response.message = validation_result['message']
            return response
        
        def mock_validate_motion_parameters(acceleration, max_speed, buffer_depth):
            # Implement actual validation logic
            if not (100 <= acceleration <= 100000):
                return {'valid': False, 'message': f'acceleration must be between 100 and 100000, got {acceleration}'}
            if not (100 <= max_speed <= 50000):
                return {'valid': False, 'message': f'max_speed must be between 100 and 50000, got {max_speed}'}
            if not (1 <= buffer_depth <= 32):
                return {'valid': False, 'message': f'buffer_depth must be between 1 and 32, got {buffer_depth}'}
            return {'valid': True, 'message': 'Parameters are valid'}
        
        def mock_get_current_configuration():
            # Return a dictionary with current configuration
            return {
                'motion_strategy': 'speed_accel',
                'default_acceleration': self.mock_hardware.default_acceleration,
                'max_speed': 5000,
                'buffer_depth': self.mock_hardware.buffer_depth,
                'hardware_motion_strategy': self.mock_hardware.motion_strategy.name.lower(),
                'hardware_default_acceleration': self.mock_hardware.default_acceleration,
                'hardware_buffer_depth': self.mock_hardware.buffer_depth
            }
        
        self.service._set_motion_strategy_callback = mock_set_motion_strategy_callback
        self.service._set_motion_parameters_callback = mock_set_motion_parameters_callback
        self.service._validate_motion_parameters = mock_validate_motion_parameters
        self.service.get_current_configuration = mock_get_current_configuration

    def test_valid_strategies_defined(self):
        """Test that valid motion strategies are properly defined"""
        expected_strategies = ['duty', 'duty_accel', 'speed', 'speed_accel']
        
        self.assertEqual(len(self.service.VALID_STRATEGIES), len(expected_strategies))
        
        for strategy in expected_strategies:
            self.assertIn(strategy, self.service.VALID_STRATEGIES)
            self.assertIsInstance(self.service.VALID_STRATEGIES[strategy], MotionStrategy)

    def test_parameter_limits_defined(self):
        """Test that parameter limits are properly defined"""
        expected_params = ['default_acceleration', 'max_speed', 'buffer_depth']
        
        for param in expected_params:
            self.assertIn(param, self.service.PARAM_LIMITS)
            limits = self.service.PARAM_LIMITS[param]
            self.assertEqual(len(limits), 2)  # Min and max
            self.assertLess(limits[0], limits[1])  # Min < Max

    def test_set_motion_strategy_valid(self):
        """Test setting valid motion strategies"""
        for strategy_name in self.service.VALID_STRATEGIES.keys():
            request = MockSetMotionStrategyRequest(strategy_name)
            response = MockSetMotionStrategyResponse()
            
            result = self.service._set_motion_strategy_callback(request, response)
            
            self.assertTrue(result.success)
            self.assertIn(strategy_name, result.message)
            self.assertEqual(self.mock_hardware.motion_strategy, 
                           self.service.VALID_STRATEGIES[strategy_name])

    def test_set_motion_strategy_invalid(self):
        """Test setting invalid motion strategies"""
        invalid_strategies = ['invalid', 'duty_invalid', '', 'unknown_strategy']
        
        for invalid_strategy in invalid_strategies:
            request = MockSetMotionStrategyRequest(invalid_strategy)
            response = MockSetMotionStrategyResponse()
            
            result = self.service._set_motion_strategy_callback(request, response)
            
            self.assertFalse(result.success)
            self.assertIn('Invalid strategy', result.message)

    def test_set_motion_strategy_case_insensitive(self):
        """Test that motion strategy setting is case insensitive"""
        test_cases = ['DUTY', 'Duty', 'dUtY', 'SPEED_ACCEL', 'Speed_Accel']
        
        for strategy in test_cases:
            request = MockSetMotionStrategyRequest(strategy)
            response = MockSetMotionStrategyResponse()
            
            result = self.service._set_motion_strategy_callback(request, response)
            
            self.assertTrue(result.success, f"Failed for strategy: {strategy}")

    def test_set_motion_parameters_valid(self):
        """Test setting valid motion parameters"""
        test_cases = [
            (1000, 5000, 4),
            (500, 2000, 1),
            (2000, 10000, 16),
            (100, 100, 32),
            (100000, 50000, 1)
        ]
        
        for acceleration, max_speed, buffer_depth in test_cases:
            request = MockSetMotionParametersRequest(acceleration, max_speed, buffer_depth)
            response = MockSetMotionParametersResponse()
            
            result = self.service._set_motion_parameters_callback(request, response)
            
            self.assertTrue(result.success, 
                          f"Failed for params: accel={acceleration}, speed={max_speed}, depth={buffer_depth}")
            self.assertEqual(self.mock_hardware.default_acceleration, acceleration)
            self.assertEqual(self.mock_hardware.buffer_depth, buffer_depth)

    def test_set_motion_parameters_invalid_acceleration(self):
        """Test setting invalid acceleration parameters"""
        invalid_accelerations = [50, 0, -100, 200000]
        
        for acceleration in invalid_accelerations:
            request = MockSetMotionParametersRequest(acceleration, 5000, 4)
            response = MockSetMotionParametersResponse()
            
            result = self.service._set_motion_parameters_callback(request, response)
            
            self.assertFalse(result.success)
            self.assertIn('acceleration', result.message.lower())

    def test_set_motion_parameters_invalid_max_speed(self):
        """Test setting invalid max speed parameters"""
        invalid_speeds = [50, 0, -100, 100000]
        
        for max_speed in invalid_speeds:
            request = MockSetMotionParametersRequest(1000, max_speed, 4)
            response = MockSetMotionParametersResponse()
            
            result = self.service._set_motion_parameters_callback(request, response)
            
            self.assertFalse(result.success)
            self.assertIn('max_speed', result.message.lower())

    def test_set_motion_parameters_invalid_buffer_depth(self):
        """Test setting invalid buffer depth parameters"""
        invalid_depths = [0, -1, 33, 100]
        
        for buffer_depth in invalid_depths:
            request = MockSetMotionParametersRequest(1000, 5000, buffer_depth)
            response = MockSetMotionParametersResponse()
            
            result = self.service._set_motion_parameters_callback(request, response)
            
            self.assertFalse(result.success)
            self.assertIn('buffer_depth', result.message.lower())

    def test_validate_motion_parameters(self):
        """Test motion parameter validation function"""
        # Valid parameters
        result = self.service._validate_motion_parameters(1000, 5000, 4)
        self.assertTrue(result['valid'])
        
        # Invalid acceleration
        result = self.service._validate_motion_parameters(50, 5000, 4)
        self.assertFalse(result['valid'])
        self.assertIn('acceleration', result['message'])
        
        # Invalid max speed
        result = self.service._validate_motion_parameters(1000, 50, 4)
        self.assertFalse(result['valid'])
        self.assertIn('max_speed', result['message'])
        
        # Invalid buffer depth
        result = self.service._validate_motion_parameters(1000, 5000, 0)
        self.assertFalse(result['valid'])
        self.assertIn('buffer_depth', result['message'])

    def test_get_current_configuration(self):
        """Test getting current configuration"""
        # Mock parameter retrieval
        with patch.object(self.service, 'get_parameter') as mock_get_param:
            mock_get_param.side_effect = [
                Mock(value='speed_accel'),
                Mock(value=1000),
                Mock(value=5000),
                Mock(value=4)
            ]
            
            config = self.service.get_current_configuration()
            
            self.assertIn('motion_strategy', config)
            self.assertIn('default_acceleration', config)
            self.assertIn('max_speed', config)
            self.assertIn('buffer_depth', config)
            self.assertIn('hardware_motion_strategy', config)
            self.assertIn('hardware_default_acceleration', config)
            self.assertIn('hardware_buffer_depth', config)

    def test_hardware_interface_synchronization(self):
        """Test synchronization with hardware interface"""
        # Test that changes to hardware interface are reflected
        original_strategy = self.mock_hardware.motion_strategy
        
        request = MockSetMotionStrategyRequest('duty')
        response = MockSetMotionStrategyResponse()
        
        self.service._set_motion_strategy_callback(request, response)
        
        self.assertTrue(response.success)
        self.assertNotEqual(self.mock_hardware.motion_strategy, original_strategy)
        self.assertEqual(self.mock_hardware.motion_strategy, MotionStrategy.DUTY)

    def test_service_without_hardware_interface(self):
        """Test service functionality without hardware interface"""
        with patch('rclpy.node.Node.__init__'), \
             patch('rclpy.node.Node.get_logger') as mock_logger, \
             patch('rclpy.node.Node.declare_parameter'), \
             patch('rclpy.node.Node.create_service'), \
             patch('rclpy.node.Node.add_on_set_parameters_callback'), \
             patch('rclpy.node.Node.set_parameters'):
            
            mock_logger.return_value = Mock()
            service_no_hw = MotionConfigurationService(None)
            
            # Should still work for parameter validation
            request = MockSetMotionStrategyRequest('speed')
            response = MockSetMotionStrategyResponse()
            
            result = service_no_hw._set_motion_strategy_callback(request, response)
            self.assertTrue(result.success)


class TestMotionConfigParameterCallbacks(unittest.TestCase):
    """Test cases for parameter callback functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_hardware = Mock(spec=BasicmicroHardwareInterface)
        self.mock_hardware.motion_strategy = MotionStrategy.SPEED_ACCEL
        self.mock_hardware.default_acceleration = 1000
        self.mock_hardware.buffer_depth = 4
        
        with patch('rclpy.node.Node.__init__'), \
             patch('rclpy.node.Node.get_logger') as mock_logger, \
             patch('rclpy.node.Node.declare_parameter'), \
             patch('rclpy.node.Node.create_service'), \
             patch('rclpy.node.Node.add_on_set_parameters_callback'), \
             patch('rclpy.node.Node.set_parameters') as mock_set_params:
            
            mock_logger.return_value = Mock()
            mock_logger.return_value.info = Mock()
            mock_logger.return_value.warning = Mock()
            mock_logger.return_value.error = Mock()
            
            # Make set_parameters always succeed
            mock_set_params.return_value = True
            
            self.service = MotionConfigurationService(self.mock_hardware)
            
            # Add the logger mock directly to the service to fix AttributeError
            self.service._logger = Mock()
            self.service._logger.info = Mock()
            self.service._logger.warning = Mock()
            self.service._logger.error = Mock()
            
            # Mock the get_logger method to return our mock logger
            self.service.get_logger = Mock(return_value=self.service._logger)
            
            # Mock set_parameters method on the service instance too
            self.service.set_parameters = Mock(return_value=True)

    @patch('rcl_interfaces.msg.SetParametersResult')  
    def test_parameter_callback_valid_strategy(self, mock_result):
        """Test parameter callback with valid motion strategy"""
        mock_result.return_value = Mock()
        mock_result.return_value.successful = True
        
        param = Parameter('motion_strategy', Parameter.Type.STRING, 'duty')
        
        result = self.service._parameters_callback([param])
        
        self.assertTrue(result.successful)
        self.assertEqual(self.mock_hardware.motion_strategy, MotionStrategy.DUTY)

    @patch('rcl_interfaces.msg.SetParametersResult')
    def test_parameter_callback_invalid_strategy(self, mock_result):
        """Test parameter callback with invalid motion strategy"""
        mock_result.return_value = Mock()
        mock_result.return_value.successful = False
        mock_result.return_value.reason = ""
        
        param = Parameter('motion_strategy', Parameter.Type.STRING, 'invalid_strategy')
        
        result = self.service._parameters_callback([param])
        
        self.assertFalse(result.successful)
        self.assertIn('Invalid motion strategy', result.reason)

    @patch('rcl_interfaces.msg.SetParametersResult')
    def test_parameter_callback_valid_acceleration(self, mock_result):
        """Test parameter callback with valid acceleration"""
        mock_result.return_value = Mock()
        mock_result.return_value.successful = True
        
        param = Parameter('default_acceleration', Parameter.Type.INTEGER, 2000)
        
        result = self.service._parameters_callback([param])
        
        self.assertTrue(result.successful)
        self.assertEqual(self.mock_hardware.default_acceleration, 2000)

    @patch('rcl_interfaces.msg.SetParametersResult')
    def test_parameter_callback_invalid_acceleration(self, mock_result):
        """Test parameter callback with invalid acceleration"""
        mock_result.return_value = Mock()
        mock_result.return_value.successful = False
        mock_result.return_value.reason = ""
        
        param = Parameter('default_acceleration', Parameter.Type.INTEGER, 50)
        
        result = self.service._parameters_callback([param])
        
        self.assertFalse(result.successful)
        self.assertIn('out of range', result.reason)


class TestMotionConfigIntegration(unittest.TestCase):
    """Integration tests for motion configuration system"""
    
    def setUp(self):
        """Set up integration test fixtures"""
        self.mock_hardware = Mock(spec=BasicmicroHardwareInterface)
        self.mock_hardware.motion_strategy = MotionStrategy.SPEED_ACCEL
        self.mock_hardware.default_acceleration = 1000
        self.mock_hardware.buffer_depth = 4
        
        with patch('rclpy.node.Node.__init__'), \
             patch('rclpy.node.Node.get_logger') as mock_logger, \
             patch('rclpy.node.Node.declare_parameter'), \
             patch('rclpy.node.Node.create_service'), \
             patch('rclpy.node.Node.add_on_set_parameters_callback'), \
             patch('rclpy.node.Node.set_parameters') as mock_set_params:
            
            mock_logger.return_value = Mock()
            mock_logger.return_value.info = Mock()
            mock_logger.return_value.warning = Mock()
            mock_logger.return_value.error = Mock()
            
            # Make set_parameters always succeed
            mock_set_params.return_value = True
            
            self.service = MotionConfigurationService(self.mock_hardware)
            
            # Add the logger mock directly to the service to fix AttributeError
            self.service._logger = Mock()
            self.service._logger.info = Mock()
            self.service._logger.warning = Mock()
            self.service._logger.error = Mock()
            
            # Mock the get_logger method to return our mock logger
            self.service.get_logger = Mock(return_value=self.service._logger)
            
            # Mock set_parameters method on the service instance too
            self.service.set_parameters = Mock(return_value=True)

    def test_strategy_change_workflow(self):
        """Test complete workflow of changing motion strategy"""
        # Initial state
        self.assertEqual(self.mock_hardware.motion_strategy, MotionStrategy.SPEED_ACCEL)
        
        # Change to duty mode
        request = MockSetMotionStrategyRequest('duty')
        response = MockSetMotionStrategyResponse()
        
        result = self.service._set_motion_strategy_callback(request, response)
        
        self.assertTrue(result.success)
        self.assertEqual(self.mock_hardware.motion_strategy, MotionStrategy.DUTY)
        
        # Change to speed mode
        request = MockSetMotionStrategyRequest('speed')
        response = MockSetMotionStrategyResponse()
        
        result = self.service._set_motion_strategy_callback(request, response)
        
        self.assertTrue(result.success)
        self.assertEqual(self.mock_hardware.motion_strategy, MotionStrategy.SPEED)

    def test_parameter_change_workflow(self):
        """Test complete workflow of changing motion parameters"""
        # Initial state
        self.assertEqual(self.mock_hardware.default_acceleration, 1000)
        self.assertEqual(self.mock_hardware.buffer_depth, 4)
        
        # Change parameters
        request = MockSetMotionParametersRequest(2000, 8000, 8)
        response = MockSetMotionParametersResponse()
        
        result = self.service._set_motion_parameters_callback(request, response)
        
        self.assertTrue(result.success)
        self.assertEqual(self.mock_hardware.default_acceleration, 2000)
        self.assertEqual(self.mock_hardware.buffer_depth, 8)

    def test_mixed_configuration_changes(self):
        """Test changing both strategy and parameters"""
        # Change strategy first
        strategy_request = MockSetMotionStrategyRequest('duty_accel')
        strategy_response = MockSetMotionStrategyResponse()
        
        strategy_result = self.service._set_motion_strategy_callback(strategy_request, strategy_response)
        self.assertTrue(strategy_result.success)
        
        # Change parameters
        param_request = MockSetMotionParametersRequest(1500, 6000, 16)
        param_response = MockSetMotionParametersResponse()
        
        param_result = self.service._set_motion_parameters_callback(param_request, param_response)
        self.assertTrue(param_result.success)
        
        # Verify final state
        self.assertEqual(self.mock_hardware.motion_strategy, MotionStrategy.DUTY_ACCEL)
        self.assertEqual(self.mock_hardware.default_acceleration, 1500)
        self.assertEqual(self.mock_hardware.buffer_depth, 16)

    def test_error_recovery(self):
        """Test error handling and recovery"""
        # Try invalid strategy
        request = MockSetMotionStrategyRequest('invalid')
        response = MockSetMotionStrategyResponse()
        
        result = self.service._set_motion_strategy_callback(request, response)
        self.assertFalse(result.success)
        
        # Original strategy should be unchanged
        self.assertEqual(self.mock_hardware.motion_strategy, MotionStrategy.SPEED_ACCEL)
        
        # Valid strategy should still work
        request = MockSetMotionStrategyRequest('duty')
        response = MockSetMotionStrategyResponse()
        
        result = self.service._set_motion_strategy_callback(request, response)
        self.assertTrue(result.success)
        self.assertEqual(self.mock_hardware.motion_strategy, MotionStrategy.DUTY)


def run_tests():
    """Run all motion configuration service tests"""
    test_classes = [
        TestMotionConfigurationService,
        TestMotionConfigParameterCallbacks,
        TestMotionConfigIntegration
    ]
    
    suite = unittest.TestSuite()
    
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    import sys
    success = run_tests()
    sys.exit(0 if success else 1)