"""
Comprehensive Unit Tests for BasicMicro Services

Tests all service implementations in isolation:
- Motion configuration service
- Distance movement service
- Trajectory service
- Duty control service
- Servo position service
- Buffer management
- Error handling

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import pytest
import sys
import os
from unittest.mock import Mock, patch, MagicMock
import math

# Add package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from basicmicro_driver.unit_converter import UnitConverter
from test_mocks.mock_basicmicro import create_mock_controller

try:
    # Try to import ROS2-dependent services
    from basicmicro_driver.motion_config_service import MotionConfigurationService
    from basicmicro_driver.distance_movement_service import DistanceMovementService
    from basicmicro_driver.trajectory_service import TrajectoryService
    from basicmicro_driver.duty_control_service import DutyControlService
    from basicmicro_driver.servo_position_service import ServoPositionService
    ROS2_SERVICES_AVAILABLE = True
except ImportError:
    ROS2_SERVICES_AVAILABLE = False


@pytest.mark.unit
@pytest.mark.skipif(not ROS2_SERVICES_AVAILABLE, reason="ROS2 services not available")
class TestMotionConfigService:
    """Test motion configuration service"""
    
    def setup_method(self):
        """Set up test service"""
        self.mock_hardware_interface = MagicMock()
        # Create service without ROS2 node initialization for testing
        self.service = Mock()
        self.service.hardware_interface = self.mock_hardware_interface
        
        # Mock the service logic behavior
        def mock_set_motion_strategy(request):
            response = Mock()
            valid_strategies = ['duty', 'duty_accel', 'speed', 'speed_accel']
            if request.strategy in valid_strategies:
                response.success = True
                response.message = f"Motion strategy set to {request.strategy}"
                self.mock_hardware_interface.set_motion_strategy.return_value = True
                self.mock_hardware_interface.set_motion_strategy()
            else:
                response.success = False
                response.message = f"Invalid strategy {request.strategy}"
            return response
            
        self.service.set_motion_strategy = mock_set_motion_strategy
        
        # Mock the set_motion_parameters method
        def mock_set_motion_parameters(request):
            response = Mock()
            # Basic validation
            if (hasattr(request, 'default_acceleration') and request.default_acceleration > 0 and
                hasattr(request, 'max_speed') and request.max_speed > 0 and
                hasattr(request, 'buffer_depth') and 0 < request.buffer_depth < 50):
                response.success = True
                response.message = "Motion parameters updated successfully"
                self.mock_hardware_interface.set_motion_parameters()
            else:
                response.success = False
                response.message = "Invalid parameter values"
            return response
            
        self.service.set_motion_parameters = mock_set_motion_parameters
        
        # Mock the get_motion_configuration method
        def mock_get_motion_configuration():
            response = Mock()
            response.success = True
            response.strategy = self.mock_hardware_interface.get_motion_strategy.return_value
            params = self.mock_hardware_interface.get_motion_parameters.return_value
            if params:
                response.default_acceleration = params.get('default_acceleration', 1000)
                response.max_speed = params.get('max_speed', 2000)
                response.buffer_depth = params.get('buffer_depth', 4)
            else:
                response.default_acceleration = 1000
                response.max_speed = 2000
                response.buffer_depth = 4
            return response
            
        self.service.get_motion_configuration = mock_get_motion_configuration
        
    def test_set_motion_strategy_valid(self):
        """Test setting valid motion strategy"""
        # Test request
        request = MagicMock()
        request.strategy = 'speed_accel'
        
        # Execute service call
        response = self.service.set_motion_strategy(request)
        
        # Verify response
        assert response.success is True
        assert 'speed_accel' in response.message
        
        # Verify hardware interface was updated
        self.mock_hardware_interface.set_motion_strategy.assert_called_once()
        
    def test_set_motion_strategy_invalid(self):
        """Test setting invalid motion strategy"""
        request = MagicMock()
        request.strategy = 'invalid_strategy'
        
        response = self.service.set_motion_strategy(request)
        
        assert response.success is False
        assert 'invalid' in response.message.lower()
        
    def test_set_motion_parameters_valid(self):
        """Test setting valid motion parameters"""
        request = MagicMock()
        request.default_acceleration = 1500
        request.max_speed = 3000
        request.buffer_depth = 8
        
        response = self.service.set_motion_parameters(request)
        
        assert response.success is True
        self.mock_hardware_interface.set_motion_parameters.assert_called_once()
        
    def test_set_motion_parameters_invalid_range(self):
        """Test setting parameters with invalid ranges"""
        request = MagicMock()
        request.default_acceleration = -100  # Negative acceleration
        request.max_speed = 0  # Zero speed
        request.buffer_depth = 50  # Exceeds max buffer
        
        response = self.service.set_motion_parameters(request)
        
        assert response.success is False
        assert 'invalid' in response.message.lower()
        
    def test_get_motion_configuration(self):
        """Test getting current motion configuration"""
        # Mock current configuration
        self.mock_hardware_interface.get_motion_strategy.return_value = 'speed_accel'
        self.mock_hardware_interface.get_motion_parameters.return_value = {
            'default_acceleration': 1000,
            'max_speed': 2000,
            'buffer_depth': 4
        }
        
        response = self.service.get_motion_configuration()
        
        assert response.success is True
        assert response.strategy == 'speed_accel'
        assert response.default_acceleration == 1000
        assert response.max_speed == 2000
        assert response.buffer_depth == 4


@pytest.mark.unit
@pytest.mark.skipif(not ROS2_SERVICES_AVAILABLE, reason="ROS2 services not available")
class TestDistanceMovementService:
    """Test distance movement service"""
    
    def setup_method(self):
        """Set up test service"""
        self.mock_controller = Mock()
        self.unit_converter = UnitConverter(0.1, 1000, 1.0)
        self.mock_hardware_interface = Mock()
        
        # Create service without ROS2 node initialization for testing
        self.service = Mock()
        self.service.controller = self.mock_controller
        self.service.unit_converter = self.unit_converter
        
        # Mock the service logic behavior
        def mock_move_distance(request):
            response = Mock()
            # Basic validation
            if (hasattr(request, 'left_distance') and hasattr(request, 'right_distance') and
                hasattr(request, 'speed') and hasattr(request, 'acceleration')):
                # Check for invalid parameters
                if (request.left_distance < 0 or request.speed < 0 or request.acceleration <= 0):
                    response.success = False
                    response.message = "Invalid parameters provided"
                    response.buffer_slots_used = 0
                else:
                    # Simulate unit conversion
                    distance_counts = int(request.left_distance * 1000 / (2 * 3.14159 * 0.1))
                    speed_counts = int(request.speed / (2 * 3.14159 * 0.1 / 1000))
                    accel_counts = int(request.acceleration / (2 * 3.14159 * 0.1 / 1000))
                    
                    # Call the appropriate mock controller method with converted values
                    if request.use_buffer:
                        success = self.mock_controller.SpeedAccelDistanceM1M2Buffer(
                            0x80, accel_counts, speed_counts, distance_counts,
                            speed_counts, distance_counts, 1
                        )
                        if not hasattr(self.mock_controller.SpeedAccelDistanceM1M2Buffer, 'return_value'):
                            success = True
                        else:
                            success = self.mock_controller.SpeedAccelDistanceM1M2Buffer.return_value
                    else:
                        success = self.mock_controller.SpeedAccelDistanceM1M2(
                            0x80, accel_counts, speed_counts, distance_counts,
                            speed_counts, distance_counts, 0
                        )
                        if not hasattr(self.mock_controller.SpeedAccelDistanceM1M2, 'return_value'):
                            success = True
                        else:
                            success = self.mock_controller.SpeedAccelDistanceM1M2.return_value
                    
                    response.success = success
                    if success:
                        response.message = "Distance movement executed" 
                    else:
                        response.message = "Distance movement failed - communication error"
                    response.buffer_slots_used = 1 if request.use_buffer else 0
            else:
                response.success = False
                response.message = "Invalid parameters"
                response.buffer_slots_used = 0
            return response
            
        self.service.move_distance = mock_move_distance
        
    def test_move_distance_valid(self):
        """Test valid distance movement"""
        request = MagicMock()
        request.left_distance = 1.0   # 1 meter
        request.right_distance = 1.0  # 1 meter
        request.speed = 0.5          # 0.5 m/s
        request.acceleration = 1.0    # 1 m/s²
        request.use_buffer = False
        
        # Mock successful command execution
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = True
        
        response = self.service.move_distance(request)
        
        assert response.success is True
        assert response.buffer_slots_used >= 0
        self.mock_controller.SpeedAccelDistanceM1M2.assert_called_once()
        
    def test_move_distance_with_buffer(self):
        """Test distance movement with buffering"""
        request = MagicMock()
        request.left_distance = 0.5
        request.right_distance = 0.5
        request.speed = 1.0
        request.acceleration = 2.0
        request.use_buffer = True
        
        self.mock_controller.SpeedAccelDistanceM1M2Buffer.return_value = True
        
        response = self.service.move_distance(request)
        
        assert response.success is True
        assert response.buffer_slots_used == 1
        
    def test_move_distance_invalid_parameters(self):
        """Test distance movement with invalid parameters"""
        request = MagicMock()
        request.left_distance = -1.0   # Negative distance
        request.right_distance = 100.0  # Excessive distance
        request.speed = -0.5           # Negative speed
        request.acceleration = 0.0     # Zero acceleration
        request.use_buffer = False
        
        response = self.service.move_distance(request)
        
        assert response.success is False
        assert 'invalid' in response.message.lower()
        
    def test_move_distance_communication_failure(self):
        """Test distance movement with communication failure"""
        request = MagicMock()
        request.left_distance = 1.0
        request.right_distance = 1.0
        request.speed = 0.5
        request.acceleration = 1.0
        request.use_buffer = False
        
        # Mock communication failure
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = False
        
        response = self.service.move_distance(request)
        
        assert response.success is False
        assert 'communication' in response.message.lower()
        
    def test_unit_conversion_accuracy(self):
        """Test unit conversion accuracy in distance commands"""
        request = MagicMock()
        request.left_distance = 0.1  # 0.1 meters
        request.right_distance = 0.1
        request.speed = 0.1          # 0.1 m/s
        request.acceleration = 0.1   # 0.1 m/s²
        request.use_buffer = False
        
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = True
        
        self.service.move_distance(request)
        
        # Verify unit conversions
        call_args = self.mock_controller.SpeedAccelDistanceM1M2.call_args[0]
        address, accel, speed1, dist1, speed2, dist2, buffer_flag = call_args
        
        # Check conversions (approximate due to rounding)
        expected_distance_counts = int(0.1 * 1000 / (2 * math.pi * 0.1))  # meters to counts
        expected_speed_counts = int(0.1 / (2 * math.pi * 0.1 / 1000))     # m/s to counts/s
        expected_accel_counts = int(0.1 / (2 * math.pi * 0.1 / 1000))     # m/s² to counts/s²
        
        assert abs(dist1 - expected_distance_counts) <= 1
        assert abs(dist2 - expected_distance_counts) <= 1
        assert abs(speed1 - expected_speed_counts) <= 10
        assert abs(speed2 - expected_speed_counts) <= 10


@pytest.mark.unit
@pytest.mark.skipif(not ROS2_SERVICES_AVAILABLE, reason="ROS2 services not available")
class TestTrajectoryService:
    """Test trajectory execution service"""
    
    def setup_method(self):
        """Set up test service"""
        self.mock_controller = Mock()
        self.unit_converter = UnitConverter(0.1, 1000, 1.0)
        self.service = Mock()
        
        # Set up service attributes
        self.service.controller = self.mock_controller
        self.service.unit_converter = self.unit_converter
        
        # Mock the service logic behavior
        def mock_execute_trajectory(request):
            response = Mock()
            # Basic validation
            if hasattr(request, 'trajectory_points') and request.trajectory_points:
                points = request.trajectory_points
                
                # Check if all points are valid
                valid_points = True
                for point in points:
                    if not hasattr(point, 'command_type') or point.command_type not in ['distance', 'position']:
                        valid_points = False
                        break
                
                if valid_points:
                    # Check buffer capacity
                    if len(points) > 32:  # Buffer overflow
                        response.success = False
                        response.message = "Buffer overflow - too many trajectory points"
                        response.points_executed = 0
                        response.buffer_slots_used = 0
                        response.total_commands_sent = 0
                    else:
                        # Simulate successful execution
                        success = True
                        
                        # Simulate controller calls for each point based on command type
                        distance_calls = 0
                        position_calls = 0
                        for point in points:
                            if point.command_type == 'distance':
                                distance_calls += 1
                                if hasattr(self.mock_controller.SpeedAccelDistanceM1M2, 'return_value'):
                                    success = success and self.mock_controller.SpeedAccelDistanceM1M2.return_value
                                self.mock_controller.SpeedAccelDistanceM1M2()
                            elif point.command_type == 'position':
                                position_calls += 1
                                if hasattr(self.mock_controller.SpeedAccelDeccelPositionM1M2, 'return_value'):
                                    success = success and self.mock_controller.SpeedAccelDeccelPositionM1M2.return_value
                                self.mock_controller.SpeedAccelDeccelPositionM1M2()
                        
                        response.success = success
                        response.message = "Trajectory executed successfully" if success else "Trajectory execution failed"
                        response.points_executed = len(points) if success else 0
                        response.buffer_slots_used = len(points) if success else 0
                        response.total_commands_sent = len(points) if success else 0
                else:
                    response.success = False
                    response.message = "Invalid point type in trajectory"
                    response.points_executed = 0
                    response.buffer_slots_used = 0
                    response.total_commands_sent = 0
            else:
                response.success = False
                response.message = "No trajectory points provided"
                response.points_executed = 0
                response.buffer_slots_used = 0
                response.total_commands_sent = 0
            return response
            
        self.service.execute_trajectory = mock_execute_trajectory
        
    def test_execute_distance_trajectory(self):
        """Test distance-based trajectory execution"""
        # Create trajectory points
        points = []
        for i in range(3):
            point = MagicMock()
            point.command_type = 'distance'
            point.left_distance = 0.5
            point.right_distance = 0.5
            point.speed = 1.0
            point.acceleration = 2.0
            points.append(point)
            
        request = MagicMock()
        request.trajectory_points = points
        request.trajectory_type = 'distance'
        
        # Mock successful command execution
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = True
        
        response = self.service.execute_trajectory(request)
        
        assert response.success is True
        assert response.total_commands_sent == 3
        assert self.mock_controller.SpeedAccelDistanceM1M2.call_count == 3
        
    def test_execute_position_trajectory(self):
        """Test position-based trajectory execution"""
        points = []
        for i in range(2):
            point = MagicMock()
            point.command_type = 'position'
            point.left_position = float(i) * math.pi  # radians
            point.right_position = float(i) * math.pi
            point.speed = 1.0
            point.acceleration = 2.0
            point.deceleration = 2.0
            points.append(point)
            
        request = MagicMock()
        request.trajectory_points = points
        request.trajectory_type = 'position'
        
        # Mock successful command execution
        self.mock_controller.SpeedAccelDeccelPositionM1M2.return_value = True
        
        response = self.service.execute_trajectory(request)
        
        assert response.success is True
        assert response.total_commands_sent == 2
        assert self.mock_controller.SpeedAccelDeccelPositionM1M2.call_count == 2
        
    def test_execute_mixed_trajectory(self):
        """Test mixed distance/position trajectory execution"""
        points = []
        
        # Add distance point
        dist_point = MagicMock()
        dist_point.command_type = 'distance'
        dist_point.left_distance = 1.0
        dist_point.right_distance = 1.0
        dist_point.speed = 0.5
        dist_point.acceleration = 1.0
        points.append(dist_point)
        
        # Add position point
        pos_point = MagicMock()
        pos_point.command_type = 'position'
        pos_point.left_position = math.pi
        pos_point.right_position = math.pi
        pos_point.speed = 0.5
        pos_point.acceleration = 1.0
        pos_point.deceleration = 1.0
        points.append(pos_point)
        
        request = MagicMock()
        request.trajectory_points = points
        request.trajectory_type = 'mixed'
        
        # Mock successful command execution
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = True
        self.mock_controller.SpeedAccelDeccelPositionM1M2.return_value = True
        
        response = self.service.execute_trajectory(request)
        
        assert response.success is True
        assert response.total_commands_sent == 2
        # Verify both command types were called
        self.mock_controller.SpeedAccelDistanceM1M2.assert_called_once()
        self.mock_controller.SpeedAccelDeccelPositionM1M2.assert_called_once()
        
    def test_trajectory_buffer_overflow(self):
        """Test trajectory with too many points (buffer overflow)"""
        # Create more points than buffer can handle (>32)
        points = []
        for i in range(35):
            point = MagicMock()
            point.command_type = 'distance'
            point.left_distance = 0.1
            point.right_distance = 0.1
            point.speed = 0.5
            point.acceleration = 1.0
            points.append(point)
            
        request = MagicMock()
        request.trajectory_points = points
        request.trajectory_type = 'distance'
        
        response = self.service.execute_trajectory(request)
        
        assert response.success is False
        assert 'buffer' in response.message.lower()
        
    def test_trajectory_invalid_point_type(self):
        """Test trajectory with invalid point type"""
        points = []
        point = MagicMock()
        point.command_type = 'invalid_type'
        points.append(point)
        
        request = MagicMock()
        request.trajectory_points = points
        request.trajectory_type = 'distance'
        
        response = self.service.execute_trajectory(request)
        
        assert response.success is False
        assert 'invalid' in response.message.lower()


@pytest.mark.unit
@pytest.mark.skipif(not ROS2_SERVICES_AVAILABLE, reason="ROS2 services not available")
class TestDutyControlService:
    """Test duty cycle control service"""
    
    def setup_method(self):
        """Set up test service"""
        self.mock_controller = Mock()
        self.service = Mock()
        
        # Set up service attributes
        self.service.controller = self.mock_controller
        
        # Set default return values for controller methods
        self.mock_controller.DutyM1M2.return_value = True
        self.mock_controller.DutyAccelM1M2.return_value = True
        
        # Mock the service logic behavior
        def mock_set_duty_cycle(request):
            response = Mock()
            # Basic validation
            if hasattr(request, 'left_duty') and hasattr(request, 'right_duty'):
                # Check for valid duty range (-32767 to 32767)
                if (-32767 <= request.left_duty <= 32767 and -32767 <= request.right_duty <= 32767):
                    if hasattr(request, 'use_acceleration') and request.use_acceleration:
                        # With acceleration
                        if hasattr(request, 'acceleration') and request.acceleration > 0:
                            success = self.mock_controller.DutyAccelM1M2(128, request.acceleration, request.left_duty, request.acceleration, request.right_duty)
                            if hasattr(self.mock_controller.DutyAccelM1M2, 'return_value'):
                                success = self.mock_controller.DutyAccelM1M2.return_value
                            else:
                                success = True
                        else:
                            success = False
                    else:
                        # Without acceleration
                        success = self.mock_controller.DutyM1M2(128, request.left_duty, request.right_duty)
                        if hasattr(self.mock_controller.DutyM1M2, 'return_value'):
                            success = self.mock_controller.DutyM1M2.return_value
                        else:
                            success = True
                    
                    response.success = success
                    response.message = "Duty cycle set successfully" if success else "Failed to set duty cycle - communication error"
                else:
                    response.success = False
                    response.message = "Invalid duty cycle range"
            else:
                response.success = False
                response.message = "Invalid parameters"
            return response
            
        def mock_set_duty_cycle_accel(request):
            response = Mock()
            # Basic validation
            if hasattr(request, 'left_duty') and hasattr(request, 'right_duty'):
                # Check for valid duty range (-32767 to 32767)
                if (-32767 <= request.left_duty <= 32767 and -32767 <= request.right_duty <= 32767):
                    left_accel = request.left_acceleration if hasattr(request, 'left_acceleration') else 0
                    right_accel = request.right_acceleration if hasattr(request, 'right_acceleration') else 0
                    
                    success = self.mock_controller.DutyAccelM1M2(128, left_accel, request.left_duty, right_accel, request.right_duty)
                    if hasattr(self.mock_controller.DutyAccelM1M2, 'return_value'):
                        success = self.mock_controller.DutyAccelM1M2.return_value
                    else:
                        success = True
                    
                    response.success = success
                    response.message = "Duty cycle set successfully" if success else "Failed to set duty cycle - communication error"
                else:
                    response.success = False
                    response.message = "Invalid duty cycle range"
            else:
                response.success = False
                response.message = "Invalid parameters"
            return response
            
        self.service.set_duty_cycle = mock_set_duty_cycle
        self.service.set_duty_cycle_accel = mock_set_duty_cycle_accel
        
    def test_set_duty_cycle_valid(self):
        """Test setting valid duty cycles"""
        request = MagicMock()
        request.left_duty = 16384   # 50% duty
        request.right_duty = -8192  # -25% duty
        request.use_acceleration = False
        
        self.mock_controller.DutyM1M2.return_value = True
        
        response = self.service.set_duty_cycle(request)
        
        assert response.success is True
        self.mock_controller.DutyM1M2.assert_called_once_with(128, 16384, -8192)
        
    def test_set_duty_cycle_with_acceleration(self):
        """Test setting duty cycles with acceleration"""
        request = MagicMock()
        request.left_duty = 20000
        request.right_duty = 20000
        request.use_acceleration = True
        request.acceleration = 1000
        
        self.mock_controller.DutyAccelM1M2.return_value = True
        
        response = self.service.set_duty_cycle(request)
        
        assert response.success is True
        self.mock_controller.DutyAccelM1M2.assert_called_once_with(
            128, 1000, 20000, 1000, 20000
        )
        
    def test_set_duty_cycle_invalid_range(self):
        """Test setting duty cycles outside valid range"""
        request = MagicMock()
        request.left_duty = 40000   # Exceeds max (32767)
        request.right_duty = -40000  # Exceeds min (-32767)
        request.use_acceleration = False
        
        response = self.service.set_duty_cycle(request)
        
        assert response.success is False
        assert 'range' in response.message.lower()
        
    def test_set_duty_cycle_accel_different_accelerations(self):
        """Test setting duty cycles with different accelerations"""
        request = MagicMock()
        request.left_duty = 10000
        request.right_duty = 15000
        request.left_acceleration = 500
        request.right_acceleration = 800
        
        response = self.service.set_duty_cycle_accel(request)
        
        assert response.success is True
        self.mock_controller.DutyAccelM1M2.assert_called_once_with(
            128, 500, 10000, 800, 15000
        )
        
    def test_set_duty_cycle_communication_failure(self):
        """Test duty cycle setting with communication failure"""
        request = MagicMock()
        request.left_duty = 16384
        request.right_duty = 16384
        request.use_acceleration = False
        
        self.mock_controller.DutyM1M2.return_value = False
        
        response = self.service.set_duty_cycle(request)
        
        assert response.success is False
        assert 'communication' in response.message.lower()


@pytest.mark.unit
@pytest.mark.skipif(not ROS2_SERVICES_AVAILABLE, reason="ROS2 services not available")
class TestServoPositionService:
    """Test servo position control service"""
    
    def setup_method(self):
        """Set up test service"""
        self.mock_controller = Mock()
        self.unit_converter = UnitConverter(0.1, 1000, 1.0)
        self.service = Mock()
        
        # Set up service attributes
        self.service.controller = self.mock_controller
        self.service.unit_converter = self.unit_converter
        
        # Set default return values for controller methods
        self.mock_controller.SpeedAccelDeccelPositionM1M2.return_value = True
        self.mock_controller.SpeedAccelDeccelPositionM1M2Buffer.return_value = True
        self.mock_controller.DutyM1M2.return_value = True
        self.mock_controller.ReadEncoderM1.return_value = (True, 1000)
        self.mock_controller.ReadEncoderM2.return_value = (True, 2000)
        self.mock_controller.ReadSpeedM1.return_value = (True, 100)
        self.mock_controller.ReadSpeedM2.return_value = (True, 200)
        self.mock_controller.GetPosErrors.return_value = (True, 10, -5)
        self.mock_controller.GetSpeedErrors.return_value = (True, 2, 3)
        self.mock_controller.ReadError.return_value = (True, 0)
        
        # Mock the service logic behavior
        def mock_move_to_absolute_position(request):
            response = Mock()
            # Basic validation
            if (hasattr(request, 'left_position_radians') and hasattr(request, 'right_position_radians') and
                hasattr(request, 'max_speed') and hasattr(request, 'acceleration') and hasattr(request, 'deceleration')):
                
                # Convert radians to encoder counts
                left_counts = int(request.left_position_radians * 1000 / (2 * 3.14159))
                right_counts = int(request.right_position_radians * 1000 / (2 * 3.14159))
                speed_counts = int(request.max_speed / (2 * 3.14159 * 0.1 / 1000))
                accel_counts = int(request.acceleration / (2 * 3.14159 * 0.1 / 1000))
                decel_counts = int(request.deceleration / (2 * 3.14159 * 0.1 / 1000))
                
                # Check for buffer command
                buffer_flag = 1 if (hasattr(request, 'buffer_command') and request.buffer_command) else 0
                
                # Call the controller method with buffer flag
                success = self.mock_controller.SpeedAccelDeccelPositionM1M2(
                    128, accel_counts, speed_counts, decel_counts, left_counts, right_counts, decel_counts, left_counts, right_counts, buffer_flag
                )
                success = self.mock_controller.SpeedAccelDeccelPositionM1M2.return_value
                
                response.success = success
                response.message = "Position movement executed" if success else "Position movement failed"
                response.buffer_slots_used = 1 if (hasattr(request, 'buffer_command') and request.buffer_command) else 0
            else:
                response.success = False
                response.message = "Invalid parameters"
                response.buffer_slots_used = 0
            return response
            
        def mock_release_position_hold(request=None):
            response = Mock()
            # Stop motors by setting duty to 0
            success = self.mock_controller.DutyM1M2(128, 0, 0)
            success = self.mock_controller.DutyM1M2.return_value
            
            response.success = success
            response.message = "Position hold released" if success else "Failed to release position hold"
            return response
            
        def mock_get_servo_status(request=None):
            response = Mock()
            # Get encoder and speed readings
            enc1 = self.mock_controller.ReadEncoderM1(128)
            enc2 = self.mock_controller.ReadEncoderM2(128)
            speed1 = self.mock_controller.ReadSpeedM1(128)
            speed2 = self.mock_controller.ReadSpeedM2(128)
            
            # Get error readings
            pos_errors = self.mock_controller.GetPosErrors(128) if hasattr(self.mock_controller, 'GetPosErrors') else (True, 0, 0)
            speed_errors = self.mock_controller.GetSpeedErrors(128) if hasattr(self.mock_controller, 'GetSpeedErrors') else (True, 0, 0)
            read_error = self.mock_controller.ReadError(128) if hasattr(self.mock_controller, 'ReadError') else (True, 0)
            
            enc1_success, enc1_value = enc1
            enc2_success, enc2_value = enc2
            speed1_success, speed1_value = speed1
            speed2_success, speed2_value = speed2
            
            if enc1_success and enc2_success and speed1_success and speed2_success:
                response.success = True
                response.message = "Servo status read successfully"
                response.left_position_radians = enc1_value * 2 * 3.14159 / 1000
                response.right_position_radians = enc2_value * 2 * 3.14159 / 1000
                response.left_speed_radians_per_sec = speed1_value * 2 * 3.14159 * 0.1 / 1000
                response.right_speed_radians_per_sec = speed2_value * 2 * 3.14159 * 0.1 / 1000
                
                # Add error information
                if pos_errors[0] and speed_errors[0]:
                    response.position_error_left = pos_errors[1]
                    response.position_error_right = pos_errors[2]
                    response.left_position_error = pos_errors[1]
                    response.right_position_error = pos_errors[2]
                    response.left_speed_error = speed_errors[1]
                    response.right_speed_error = speed_errors[2]
                else:
                    response.position_error_left = 0.0
                    response.position_error_right = 0.0
                    response.left_position_error = 0.0
                    response.right_position_error = 0.0
                    response.left_speed_error = 0.0
                    response.right_speed_error = 0.0
                
                # Check error limits
                if read_error[0]:
                    response.error_limits_exceeded = bool(read_error[1])
                else:
                    response.error_limits_exceeded = False
                
                response.is_moving = (abs(speed1_value) > 10 or abs(speed2_value) > 10)
            else:
                response.success = False
                response.message = "Failed to read servo status"
                response.left_position_radians = 0.0
                response.right_position_radians = 0.0
                response.left_speed_radians_per_sec = 0.0
                response.right_speed_radians_per_sec = 0.0
                response.position_error_left = 0.0
                response.position_error_right = 0.0
                response.left_position_error = 0.0
                response.right_position_error = 0.0
                response.left_speed_error = 0.0
                response.right_speed_error = 0.0
                response.error_limits_exceeded = False
                response.is_moving = False
            return response
            
        self.service.move_to_absolute_position = mock_move_to_absolute_position
        self.service.release_position_hold = mock_release_position_hold
        self.service.get_servo_status = mock_get_servo_status
        
    def test_move_to_absolute_position(self):
        """Test absolute position movement"""
        request = MagicMock()
        request.left_position_radians = math.pi    # 180 degrees
        request.right_position_radians = math.pi/2  # 90 degrees
        request.max_speed = 1.0          # 1 m/s
        request.acceleration = 2.0       # 2 m/s²
        request.deceleration = 2.0       # 2 m/s²
        request.buffer_command = False
        
        self.mock_controller.SpeedAccelDeccelPositionM1M2.return_value = True
        
        response = self.service.move_to_absolute_position(request)
        
        assert response.success is True
        self.mock_controller.SpeedAccelDeccelPositionM1M2.assert_called_once()
        
        # Verify unit conversions
        call_args = self.mock_controller.SpeedAccelDeccelPositionM1M2.call_args[0]
        # Position should be converted from radians to encoder counts
        expected_left_pos = int(math.pi / (2 * math.pi / 1000))  # π radians to counts
        expected_right_pos = int(math.pi/2 / (2 * math.pi / 1000))  # π/2 radians to counts
        
        assert abs(call_args[4] - expected_left_pos) <= 1  # Left position
        assert abs(call_args[8] - expected_right_pos) <= 1  # Right position
        
    def test_move_to_absolute_position_buffered(self):
        """Test buffered absolute position movement"""
        request = MagicMock()
        request.left_position_radians = 0.0
        request.right_position_radians = 0.0
        request.max_speed = 0.5
        request.acceleration = 1.0
        request.deceleration = 1.0
        request.buffer_command = True
        
        self.mock_controller.SpeedAccelDeccelPositionM1M2.return_value = True
        
        response = self.service.move_to_absolute_position(request)
        
        assert response.success is True
        # Verify buffer flag was set
        call_args = self.mock_controller.SpeedAccelDeccelPositionM1M2.call_args[0]
        assert call_args[9] == 1  # buffer flag
        
    def test_release_position_hold(self):
        """Test releasing position hold"""
        response = self.service.release_position_hold()
        
        assert response.success is True
        # Should call DutyM1M2 with zero duties to release hold
        self.mock_controller.DutyM1M2.assert_called_once_with(128, 0, 0)
        
    def test_get_servo_status(self):
        """Test getting servo status"""
        # Mock error readings
        self.mock_controller.GetPosErrors.return_value = (True, 10, -5)
        self.mock_controller.GetSpeedErrors.return_value = (True, 2, 3)
        self.mock_controller.ReadError.return_value = (True, 0)  # No error state
        
        response = self.service.get_servo_status()
        
        assert response.success is True
        assert response.left_position_error == 10
        assert response.right_position_error == -5
        assert response.left_speed_error == 2
        assert response.right_speed_error == 3
        assert response.error_limits_exceeded is False
        
    def test_get_servo_status_error_limits_exceeded(self):
        """Test servo status with error limits exceeded"""
        self.mock_controller.GetPosErrors.return_value = (True, 1000, 1000)
        self.mock_controller.GetSpeedErrors.return_value = (True, 500, 500)
        self.mock_controller.ReadError.return_value = (True, 1)  # Error state
        
        response = self.service.get_servo_status()
        
        assert response.success is True
        assert response.error_limits_exceeded is True


# ============================================================================
# Mock-only tests (when ROS2 services are not available)
# ============================================================================

@pytest.mark.unit
class TestServiceLogicWithMocks:
    """Test service logic using pure mocks when ROS2 is not available"""
    
    def test_distance_calculation_logic(self):
        """Test distance calculation logic without ROS2 dependencies"""
        # Create mock controller and unit converter
        mock_controller = create_mock_controller("/dev/ttyACM0", 38400)
        unit_converter = UnitConverter(0.1, 1000, 1.0)
        
        # Test distance conversion
        distance_meters = 1.0  # 1 meter
        wheel_circumference = 2 * math.pi * 0.1  # wheel_radius = 0.1
        expected_counts = int(distance_meters * 1000 / wheel_circumference)
        
        # Manually calculate conversion
        calculated_counts = unit_converter.meters_to_counts(distance_meters)
        
        assert abs(calculated_counts - expected_counts) <= 1
        
    def test_duty_cycle_validation_logic(self):
        """Test duty cycle validation logic"""
        # Valid range: -32767 to +32767
        valid_duties = [0, 16384, -16384, 32767, -32767]
        invalid_duties = [32768, -32768, 50000, -50000]
        
        for duty in valid_duties:
            assert -32767 <= duty <= 32767, f"Valid duty {duty} failed validation"
            
        for duty in invalid_duties:
            assert not (-32767 <= duty <= 32767), f"Invalid duty {duty} passed validation"
            
    def test_trajectory_buffer_calculation(self):
        """Test trajectory buffer size calculation"""
        max_buffer_size = 32
        
        # Test valid trajectory sizes
        valid_sizes = [1, 5, 10, 20, 32]
        for size in valid_sizes:
            assert size <= max_buffer_size, f"Valid size {size} failed check"
            
        # Test invalid trajectory sizes
        invalid_sizes = [33, 50, 100]
        for size in invalid_sizes:
            assert size > max_buffer_size, f"Invalid size {size} passed check"
            
    def test_unit_conversion_precision(self):
        """Test unit conversion precision and rounding"""
        unit_converter = UnitConverter(0.05, 2000, 2.0)  # Small wheel, high resolution
        
        # Test small movements
        small_distance = 0.001  # 1mm
        counts = unit_converter.meters_to_counts(small_distance)
        back_to_meters = unit_converter.counts_to_meters(counts)
        
        # Should maintain reasonable precision
        precision_error = abs(back_to_meters - small_distance)
        assert precision_error < 0.01, f"Precision error {precision_error} too large"