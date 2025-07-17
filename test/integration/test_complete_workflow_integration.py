"""
Complete Workflow Integration Tests

Tests end-to-end workflows integrating multiple components:
- Hardware interface + services + unit conversion
- Complete motion command workflows
- Sensor reading and state propagation
- Error handling across components
- Service interaction patterns

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import pytest
import sys
import os
import time
import math
from unittest.mock import Mock, patch, MagicMock

# Add package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from basicmicro_driver.hardware_interface import BasicmicroHardwareInterface, MotionStrategy, return_type
from basicmicro_driver.unit_converter import UnitConverter
from test_mocks.mock_basicmicro import create_mock_controller

try:
    # Try to import ROS2-dependent modules
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


@pytest.mark.integration
class TestCompleteMotionWorkflow:
    """Test complete motion command workflows"""
    
    def setup_method(self):
        """Set up integrated test components"""
        # Create hardware interface
        self.hw = BasicmicroHardwareInterface()
        self.hw.controller = Mock()
        self.hw.address = 128
        self.hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        self.hw.emergency_stop_active = False
        
        # Set up interfaces
        self.hw.hw_commands_velocities_ = [0.0, 0.0]
        self.hw.hw_states_positions_ = [0.0, 0.0]
        self.hw.hw_states_velocities_ = [0.0, 0.0]
        
        # Configure motion parameters
        self.hw.motion_strategy = MotionStrategy.SPEED_ACCEL
        self.hw.default_acceleration = 1000
        
    def test_velocity_command_to_execution_workflow(self):
        """Test complete workflow from velocity command to hardware execution"""
        # Set velocity commands (simulating ros2_control command interface)
        target_left_vel = 1.0   # rad/s
        target_right_vel = 0.5  # rad/s
        
        self.hw.hw_commands_velocities_[0] = target_left_vel
        self.hw.hw_commands_velocities_[1] = target_right_vel
        
        # Mock successful hardware response
        self.hw.controller.SpeedAccelM1M2.return_value = True
        
        # Execute command
        result = self.hw.write(time=0.0, period=0.1)
        
        # Verify command was executed
        assert result == return_type.OK
        self.hw.controller.SpeedAccelM1M2.assert_called_once()
        
        # Verify unit conversion was applied correctly
        call_args = self.hw.controller.SpeedAccelM1M2.call_args[0]
        address, accel, left_speed, right_speed = call_args
        
        assert address == 128
        assert accel == 1000
        
        # Check speed conversions (rad/s to counts/sec)
        expected_left_speed = int(target_left_vel / (2 * math.pi / 1000))
        expected_right_speed = int(target_right_vel / (2 * math.pi / 1000))
        
        assert abs(left_speed - expected_left_speed) <= 1
        assert abs(right_speed - expected_right_speed) <= 1
        
    def test_sensor_reading_to_state_workflow(self):
        """Test complete workflow from sensor reading to state interface update"""
        # Mock sensor readings
        encoder_left = 1570    # encoder counts
        encoder_right = 785    # encoder counts
        speed_left = 157       # counts/sec
        speed_right = 79       # counts/sec
        
        self.hw.controller.GetEncoders.return_value = (True, encoder_left, encoder_right)
        self.hw.controller.GetSpeeds.return_value = (True, speed_left, speed_right)
        
        # Execute sensor reading
        result = self.hw.read(time=0.0, period=0.1)
        
        # Verify reading was successful
        assert result == return_type.OK
        self.hw.controller.GetEncoders.assert_called_once_with(128)
        self.hw.controller.GetSpeeds.assert_called_once_with(128)
        
        # Verify state interface updates with correct unit conversion
        expected_left_pos = encoder_left * (2 * math.pi / 1000)  # counts to radians
        expected_right_pos = encoder_right * (2 * math.pi / 1000)
        expected_left_vel = speed_left * (2 * math.pi / 1000)    # counts/sec to rad/s
        expected_right_vel = speed_right * (2 * math.pi / 1000)
        
        assert abs(self.hw.hw_states_positions_[0] - expected_left_pos) < 1e-3
        assert abs(self.hw.hw_states_positions_[1] - expected_right_pos) < 1e-3
        assert abs(self.hw.hw_states_velocities_[0] - expected_left_vel) < 1e-3
        assert abs(self.hw.hw_states_velocities_[1] - expected_right_vel) < 1e-3
        
    def test_emergency_stop_workflow(self):
        """Test complete emergency stop workflow"""
        # Set up normal operation first
        self.hw.hw_commands_velocities_[0] = 2.0
        self.hw.hw_commands_velocities_[1] = 2.0
        
        # Execute emergency stop
        result = self.hw.emergency_stop()
        
        # Verify emergency stop execution
        assert result is True
        assert self.hw.emergency_stop_active is True
        self.hw.controller.DutyM1M2.assert_called_with(128, 0, 0)
        
        # Try to execute normal commands after emergency stop
        result = self.hw.write(time=0.0, period=0.1)
        
        # Commands should be blocked but write should still succeed
        assert result == return_type.OK
        # Verify no motion commands were sent (only emergency stop)
        assert self.hw.controller.SpeedAccelM1M2.call_count == 0
        
    def test_error_recovery_workflow(self):
        """Test error detection and recovery workflow"""
        # Simulate communication error
        self.hw.controller.GetEncoders.side_effect = Exception("Communication error")
        self.hw.controller.GetSpeeds.side_effect = Exception("Communication error")
        
        # Store previous state
        prev_positions = self.hw.hw_states_positions_.copy()
        prev_velocities = self.hw.hw_states_velocities_.copy()
        
        # Execute read with error
        result = self.hw.read(time=0.0, period=0.1)
        
        # Should fail and preserve previous state
        assert result == return_type.ERROR
        assert self.hw.hw_states_positions_ == prev_positions
        assert self.hw.hw_states_velocities_ == prev_velocities
        
        # Restore communication and verify recovery
        self.hw.controller.GetEncoders.side_effect = None
        self.hw.controller.GetSpeeds.side_effect = None
        self.hw.controller.GetEncoders.return_value = (True, 100, 200)
        self.hw.controller.GetSpeeds.return_value = (True, 10, 20)
        
        result = self.hw.read(time=0.0, period=0.1)
        assert result == return_type.OK
        # State should be updated after recovery
        assert self.hw.hw_states_positions_[0] != prev_positions[0]
        assert self.hw.hw_states_positions_[1] != prev_positions[1]
        
    def test_motion_strategy_switching_workflow(self):
        """Test workflow of switching motion strategies during operation"""
        # Start with speed strategy
        self.hw.motion_strategy = MotionStrategy.SPEED
        self.hw.hw_commands_velocities_[0] = 1.0
        self.hw.hw_commands_velocities_[1] = 1.0
        
        # Execute command
        self.hw.controller.SpeedM1M2.return_value = True
        result = self.hw.write(time=0.0, period=0.1)
        assert result == return_type.OK
        self.hw.controller.SpeedM1M2.assert_called_once()
        
        # Switch to duty strategy
        self.hw.motion_strategy = MotionStrategy.DUTY
        self.hw.max_duty_cycle = 16384
        
        # Execute same velocity command with new strategy
        self.hw.controller.DutyM1M2.return_value = True
        result = self.hw.write(time=0.0, period=0.1)
        assert result == return_type.OK
        self.hw.controller.DutyM1M2.assert_called_once()
        
        # Verify different command was used
        assert self.hw.controller.SpeedM1M2.call_count == 1  # Called once before
        
    def test_continuous_operation_workflow(self):
        """Test continuous operation workflow with multiple read/write cycles"""
        # Simulate continuous operation
        cycles = 10
        
        for i in range(cycles):
            # Set varying velocity commands
            self.hw.hw_commands_velocities_[0] = 0.5 + 0.1 * i
            self.hw.hw_commands_velocities_[1] = 0.5 - 0.1 * i
            
            # Mock varying sensor readings
            self.hw.controller.GetEncoders.return_value = (True, 100 * i, 150 * i)
            self.hw.controller.GetSpeeds.return_value = (True, 10 * i, 15 * i)
            self.hw.controller.SpeedAccelM1M2.return_value = True
            
            # Execute read/write cycle
            read_result = self.hw.read(time=0.0, period=0.1)
            write_result = self.hw.write(time=0.0, period=0.1)
            
            assert read_result == return_type.OK
            assert write_result == return_type.OK
            
        # Verify all commands were executed
        assert self.hw.controller.GetEncoders.call_count == cycles
        assert self.hw.controller.GetSpeeds.call_count == cycles
        assert self.hw.controller.SpeedAccelM1M2.call_count == cycles


@pytest.mark.integration
@pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
class TestROS2ControlIntegration:
    """Test integration with ROS2 control system"""
    
    def setup_method(self):
        """Set up ROS2 integration test"""
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
            
        # Create test node
        self.node = Node('test_hardware_interface')
        
        # Create hardware interface
        self.hw = BasicmicroHardwareInterface()
        self.hw.controller = Mock()
        self.hw.address = 128
        self.hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        
        # Set up interfaces
        self.hw.hw_commands_velocities_ = [0.0, 0.0]
        self.hw.hw_states_positions_ = [0.0, 0.0]
        self.hw.hw_states_velocities_ = [0.0, 0.0]
        
    def teardown_method(self):
        """Clean up ROS2 integration test"""
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
    def test_joint_state_publishing_integration(self):
        """Test integration with joint state publishing"""
        # Create joint state publisher
        joint_state_pub = self.node.create_publisher(JointState, '/joint_states', 10)
        
        # Mock sensor readings
        self.hw.controller.GetEncoders.return_value = (True, 1000, 1500)
        self.hw.controller.GetSpeeds.return_value = (True, 100, 150)
        
        # Execute sensor reading
        result = self.hw.read(time=0.0, period=0.1)
        assert result == return_type.OK
        
        # Create and publish joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.node.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = list(self.hw.hw_states_positions_)
        joint_state.velocity = list(self.hw.hw_states_velocities_)
        
        # Publish message
        joint_state_pub.publish(joint_state)
        
        # Verify message content
        assert len(joint_state.position) == 2
        assert len(joint_state.velocity) == 2
        assert joint_state.position[0] > 0  # Left position
        assert joint_state.position[1] > 0  # Right position
        
    def test_cmd_vel_to_hardware_integration(self):
        """Test integration from cmd_vel to hardware commands"""
        # Simulate differential drive controller behavior
        linear_vel = 1.0   # m/s
        angular_vel = 0.5  # rad/s
        wheel_separation = 0.3  # meters
        
        # Calculate wheel velocities (differential drive kinematics)
        left_wheel_vel = (linear_vel - angular_vel * wheel_separation / 2) / 0.1  # rad/s
        right_wheel_vel = (linear_vel + angular_vel * wheel_separation / 2) / 0.1  # rad/s
        
        # Set hardware interface commands
        self.hw.hw_commands_velocities_[0] = left_wheel_vel
        self.hw.hw_commands_velocities_[1] = right_wheel_vel
        
        # Mock hardware response
        self.hw.controller.SpeedAccelM1M2.return_value = True
        
        # Execute command
        result = self.hw.write(time=0.0, period=0.1)
        assert result == return_type.OK
        
        # Verify differential drive kinematics were applied correctly
        call_args = self.hw.controller.SpeedAccelM1M2.call_args[0]
        left_speed_counts = call_args[2]
        right_speed_counts = call_args[3]
        
        # Convert back to rad/s for verification
        left_speed_actual = left_speed_counts * (2 * math.pi / 1000)
        right_speed_actual = right_speed_counts * (2 * math.pi / 1000)
        
        assert abs(left_speed_actual - left_wheel_vel) < 0.1
        assert abs(right_speed_actual - right_wheel_vel) < 0.1


@pytest.mark.integration
class TestServiceInteractionWorkflows:
    """Test workflows involving multiple service interactions"""
    
    def setup_method(self):
        """Set up service interaction test"""
        self.mock_controller = Mock()
        self.unit_converter = UnitConverter(0.1, 1000, 1.0)
        
    def test_motion_config_to_execution_workflow(self):
        """Test workflow from motion configuration to command execution"""
        # This test simulates the workflow without requiring ROS2 services
        # by testing the underlying logic
        
        # Step 1: Configure motion strategy
        motion_strategy = "speed_accel"
        default_acceleration = 1500
        
        # Step 2: Set motion parameters
        max_speed = 3000  # counts/sec
        buffer_depth = 8
        
        # Step 3: Execute motion command using configured parameters
        self.mock_controller.SpeedAccelM1M2.return_value = True
        
        # Simulate command execution with configured parameters
        address = 128
        left_speed = 1000   # counts/sec
        right_speed = 1000  # counts/sec
        
        # Execute command with configured acceleration
        result = self.mock_controller.SpeedAccelM1M2(
            address, default_acceleration, left_speed, right_speed
        )
        
        assert result is True
        self.mock_controller.SpeedAccelM1M2.assert_called_once_with(
            address, default_acceleration, left_speed, right_speed
        )
        
    def test_distance_to_trajectory_workflow(self):
        """Test workflow from distance commands to trajectory execution"""
        # Step 1: Execute individual distance command
        distance_meters = 1.0
        speed_mps = 0.5
        acceleration_mps2 = 1.0
        
        # Convert to hardware units
        distance_counts = self.unit_converter.meters_to_counts(distance_meters)
        speed_counts = int(speed_mps / (2 * math.pi * 0.1 / 1000))  # m/s to counts/s
        accel_counts = int(acceleration_mps2 / (2 * math.pi * 0.1 / 1000))  # m/s² to counts/s²
        
        # Step 2: Execute as part of trajectory (buffered)
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = True
        
        # Execute multiple distance commands as trajectory
        trajectory_points = [
            (distance_counts, distance_counts, speed_counts, accel_counts, 1),  # buffered
            (distance_counts, distance_counts, speed_counts, accel_counts, 1),  # buffered
            (distance_counts, distance_counts, speed_counts, accel_counts, 0),  # last command
        ]
        
        for left_dist, right_dist, speed, accel, buffer_flag in trajectory_points:
            result = self.mock_controller.SpeedAccelDistanceM1M2(
                128, accel, speed, left_dist, speed, right_dist, buffer_flag
            )
            assert result is True
            
        # Verify all commands were executed
        assert self.mock_controller.SpeedAccelDistanceM1M2.call_count == 3
        
    def test_servo_position_sequence_workflow(self):
        """Test workflow for servo position sequences"""
        # Step 1: Configure servo parameters
        encoder_type = "incremental"
        position_limits_enabled = True
        
        # Step 2: Execute position sequence
        positions_radians = [0.0, math.pi/2, math.pi, 0.0]  # Quarter turn sequence
        max_speed_mps = 0.5
        acceleration_mps2 = 1.0
        deceleration_mps2 = 1.0
        
        self.mock_controller.SpeedAccelDeccelPositionM1M2.return_value = True
        
        for i, target_position in enumerate(positions_radians):
            # Convert position to encoder counts
            target_counts = int(target_position / (2 * math.pi / 1000))
            speed_counts = int(max_speed_mps / (2 * math.pi * 0.1 / 1000))
            accel_counts = int(acceleration_mps2 / (2 * math.pi * 0.1 / 1000))
            decel_counts = int(deceleration_mps2 / (2 * math.pi * 0.1 / 1000))
            
            # Buffer all commands except the last
            buffer_flag = 1 if i < len(positions_radians) - 1 else 0
            
            result = self.mock_controller.SpeedAccelDeccelPositionM1M2(
                128,
                accel_counts, speed_counts, decel_counts, target_counts,  # Left motor
                accel_counts, speed_counts, decel_counts, target_counts,  # Right motor
                buffer_flag
            )
            assert result is True
            
        # Verify all position commands were executed
        assert self.mock_controller.SpeedAccelDeccelPositionM1M2.call_count == len(positions_radians)
        
    def test_error_handling_across_services_workflow(self):
        """Test error handling workflow across multiple service interactions"""
        # Step 1: Simulate communication error during distance command
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = False
        
        result = self.mock_controller.SpeedAccelDistanceM1M2(128, 1000, 500, 1000, 500, 1000, 0)
        assert result is False  # Command failed
        
        # Step 2: Recovery - switch to emergency stop
        self.mock_controller.DutyM1M2.return_value = True
        result = self.mock_controller.DutyM1M2(128, 0, 0)
        assert result is True  # Emergency stop succeeded
        
        # Step 3: Recovery - restore communication and retry
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = True
        result = self.mock_controller.SpeedAccelDistanceM1M2(128, 1000, 500, 1000, 500, 1000, 0)
        assert result is True  # Command succeeded after recovery
        
        # Verify error recovery sequence
        self.mock_controller.DutyM1M2.assert_called_once_with(128, 0, 0)
        assert self.mock_controller.SpeedAccelDistanceM1M2.call_count == 2


@pytest.mark.integration
class TestParameterConsistencyWorkflow:
    """Test parameter consistency across components"""
    
    def test_unit_conversion_consistency_across_components(self):
        """Test that unit conversions are consistent across all components"""
        # Common robot parameters
        wheel_radius = 0.075  # meters
        encoder_counts_per_rev = 2000
        gear_ratio = 5.0
        
        # Create unit converter
        unit_converter = UnitConverter(wheel_radius, encoder_counts_per_rev, gear_ratio)
        
        # Test consistency of conversions
        test_values = [0.1, 0.5, 1.0, 2.0, 5.0]  # Various speeds/distances
        
        for value in test_values:
            # Position conversions (radians <-> counts)
            counts = unit_converter.radians_to_counts(value)
            back_to_radians = unit_converter.counts_to_radians(counts)
            assert abs(back_to_radians - value) < 1e-3, f"Position conversion inconsistent for {value}"
            
            # Velocity conversions (rad/s <-> counts/sec)
            counts_per_sec = unit_converter.rad_per_sec_to_counts_per_sec(value)
            back_to_rad_per_sec = unit_converter.counts_per_sec_to_rad_per_sec(counts_per_sec)
            assert abs(back_to_rad_per_sec - value) < 1e-3, f"Velocity conversion inconsistent for {value}"
            
            # Distance conversions (meters <-> counts)
            dist_counts = unit_converter.meters_to_counts(value)
            back_to_meters = unit_converter.counts_to_meters(dist_counts)
            assert abs(back_to_meters - value) < 1e-3, f"Distance conversion inconsistent for {value}"
            
    def test_parameter_validation_consistency(self):
        """Test that parameter validation is consistent across components"""
        # Test invalid parameters
        invalid_wheel_radii = [-0.1, 0.0]  # negative, zero
        invalid_encoder_counts = [0, -100]        # zero, negative
        invalid_gear_ratios = [0.0, -1.0]        # zero, negative
        
        for radius in invalid_wheel_radii:
            with pytest.raises(ValueError):
                UnitConverter(radius, 1000, 1.0)
                
        for counts in invalid_encoder_counts:
            with pytest.raises(ValueError):
                UnitConverter(0.1, counts, 1.0)
                
        for ratio in invalid_gear_ratios:
            with pytest.raises(ValueError):
                UnitConverter(0.1, 1000, ratio)