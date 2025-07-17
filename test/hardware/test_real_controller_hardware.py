"""
Hardware-in-the-Loop Tests for Real Controller

Tests that run with actual Basicmicro hardware controllers. These tests are
disabled by default and require specific hardware setup and configuration.

IMPORTANT: These tests require actual hardware and should only be run in
controlled environments with proper safety measures in place.

Hardware Requirements:
- Basicmicro controller (RoboClaw or MCP)
- USB/Serial connection
- Motors or load simulation
- Emergency stop capability

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import pytest
import sys
import os
import time
import math
import serial
from unittest.mock import patch

# Add package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from basicmicro_driver.hardware_interface import BasicmicroHardwareInterface, MotionStrategy
from basicmicro_driver.unit_converter import UnitConverter

try:
    from hardware_interface_msgs.msg import return_type
except ImportError:
    # Fallback for testing without full ROS2 installation
    class return_type:
        OK = 0
        ERROR = 1

try:
    # Try to import the actual Basicmicro library
    from basicmicro import Basicmicro
    BASICMICRO_AVAILABLE = True
except ImportError:
    BASICMICRO_AVAILABLE = False


# Hardware test configuration
HARDWARE_CONFIG = {
    'port': '/dev/ttyACM0',  # Default port - can be overridden
    'baud': 38400,
    'address': 0x80,
    'wheel_radius': 0.1,
    'encoder_counts_per_rev': 1000,
    'gear_ratio': 1.0,
    'max_test_speed': 1000,  # Safe speed limit for testing (counts/sec) - increased for cogging
    'max_test_duty': 16384,  # Safe duty cycle limit (50% of max) - increased for cogging
    'test_timeout': 10.0,   # Maximum test duration
}


# Hardware test configuration is now in conftest.py
# Tests are disabled by default - use --hardware flag to enable


@pytest.fixture(scope="session")
def hardware_controller(request):
    """Create real hardware controller connection"""
    if not BASICMICRO_AVAILABLE:
        pytest.skip("Basicmicro library not available")
        
    # Get hardware configuration from command line
    port = request.config.getoption("--hardware-port")
    baud = request.config.getoption("--hardware-baud")
    address = request.config.getoption("--hardware-address")
    
    # Update configuration
    config = HARDWARE_CONFIG.copy()
    config['port'] = port
    config['baud'] = baud
    config['address'] = address
    
    try:
        # Test if port is available
        with serial.Serial(port, baud, timeout=1) as ser:
            pass
    except (serial.SerialException, OSError) as e:
        pytest.skip(f"Hardware port {port} not available: {e}")
        
    # Create controller connection
    controller = Basicmicro(port, baud)
    
    try:
        # Test connection
        if not controller.Open():
            pytest.skip(f"Could not open hardware controller on {port}")
            
        # Verify communication
        version_result = controller.ReadVersion(address)
        if not version_result[0]:
            pytest.skip(f"Could not communicate with controller at address {address}")
            
        print(f"Connected to hardware: {version_result[1]}")
        
        # Safety: Emergency stop at start
        controller.DutyM1M2(address, 0, 0)
        time.sleep(0.1)
        
        yield controller, config
        
    finally:
        # Safety: Emergency stop at end
        try:
            controller.DutyM1M2(address, 0, 0)
            time.sleep(0.1)
            controller.close()
        except:
            pass


@pytest.mark.hardware
@pytest.mark.skipif(not BASICMICRO_AVAILABLE, reason="Basicmicro library not available")
class TestHardwareBasicCommunication:
    """Test basic communication with real hardware"""
    
    def test_controller_connection(self, hardware_controller):
        """Test controller connection and basic communication"""
        controller, config = hardware_controller
        address = config['address']
        
        # Test version reading
        result = controller.ReadVersion(address)
        assert result[0], "Failed to read controller version"  # Accept any truthy value
        assert len(result[1]) > 0, "Empty version string"
        print(f"Controller version: {result[1]}")
        
        # Test basic status reading
        status_result = controller.GetStatus(address)
        assert status_result[0], "Failed to read controller status"  # Accept any truthy value
        
    def test_encoder_reading(self, hardware_controller):
        """Test encoder reading from real hardware"""
        controller, config = hardware_controller
        address = config['address']
        
        # Read encoders
        result = controller.GetEncoders(address)
        assert result[0], "Failed to read encoders"  # Accept any truthy value
        
        left_encoder, right_encoder = result[1], result[2]
        print(f"Encoder readings - Left: {left_encoder}, Right: {right_encoder}")
        
        # Encoders should be readable (values can be any integer)
        assert isinstance(left_encoder, int), "Left encoder not integer"
        assert isinstance(right_encoder, int), "Right encoder not integer"
        
    def test_speed_reading(self, hardware_controller):
        """Test speed reading from real hardware"""
        controller, config = hardware_controller
        address = config['address']
        
        # Read speeds
        result = controller.GetSpeeds(address)
        assert result[0], "Failed to read speeds"  # Accept any truthy value
        
        left_speed, right_speed = result[1], result[2]
        print(f"Speed readings - Left: {left_speed}, Right: {right_speed}")
        
        # Speeds should be readable (should be near zero if motors are stopped)
        assert isinstance(left_speed, int), "Left speed not integer"
        assert isinstance(right_speed, int), "Right speed not integer"
        
    def test_voltage_monitoring(self, hardware_controller):
        """Test voltage monitoring"""
        controller, config = hardware_controller
        address = config['address']
        
        # Read voltages
        result = controller.GetVolts(address)
        assert result[0], "Failed to read voltages"  # Accept any truthy value
        
        main_voltage, logic_voltage = result[1], result[2]
        print(f"Voltages - Main: {main_voltage/10:.1f}V, Logic: {logic_voltage/10:.1f}V")
        
        # Voltages should be reasonable (divide by 10 for actual voltage)
        assert main_voltage > 50, "Main voltage too low (below 5V)"  # At least 5V
        assert main_voltage < 300, "Main voltage too high (above 30V)"  # Max reasonable
        
        # Logic voltage may be 0 on some controllers (USB-powered), so just check if present
        if logic_voltage > 0:
            assert logic_voltage > 30, "Logic voltage too low (below 3V)"
            assert logic_voltage < 60, "Logic voltage too high (above 6V)"
        else:
            print("Logic voltage reading 0 - may be USB-powered controller")


@pytest.mark.hardware
@pytest.mark.skipif(not BASICMICRO_AVAILABLE, reason="Basicmicro library not available")
class TestHardwareMotorControl:
    """Test motor control with real hardware"""
    
    def test_duty_cycle_commands(self, hardware_controller):
        """Test duty cycle motor commands"""
        controller, config = hardware_controller
        address = config['address']
        max_duty = config['max_test_duty']
        
        # Safety: Start with zero duty
        result = controller.DutyM1M2(address, 0, 0)
        assert result is True, "Failed to set zero duty"
        time.sleep(0.5)
        
        # Test small positive duty
        small_duty = max_duty // 4  # 25% of max test duty
        result = controller.DutyM1M2(address, small_duty, small_duty)
        assert result is True, f"Failed to set duty to {small_duty}"
        time.sleep(1.0)
        
        # Check that motors are running (speeds should be non-zero)
        speed_result = controller.GetSpeeds(address)
        if speed_result[0]:
            left_speed, right_speed = speed_result[1], speed_result[2]
            print(f"Speeds at duty {small_duty}: Left={left_speed}, Right={right_speed}")
            
        # Test negative duty
        result = controller.DutyM1M2(address, -small_duty, -small_duty)
        assert result is True, f"Failed to set negative duty to {-small_duty}"
        time.sleep(1.0)
        
        # Safety: Return to zero
        result = controller.DutyM1M2(address, 0, 0)
        assert result is True, "Failed to return to zero duty"
        time.sleep(0.5)
        
    def test_speed_commands(self, hardware_controller):
        """Test speed motor commands"""
        controller, config = hardware_controller
        address = config['address']
        max_speed = config['max_test_speed']
        
        # Safety: Start with zero speed
        result = controller.SpeedM1M2(address, 0, 0)
        assert result is True, "Failed to set zero speed"
        time.sleep(0.5)
        
        # Test positive speed
        test_speed = max_speed // 2
        result = controller.SpeedM1M2(address, test_speed, test_speed)
        assert result is True, f"Failed to set speed to {test_speed}"
        time.sleep(2.0)  # Allow time for speed control to engage
        
        # Check actual speeds
        speed_result = controller.GetSpeeds(address)
        if speed_result[0]:
            left_speed, right_speed = speed_result[1], speed_result[2]
            print(f"Target: {test_speed}, Actual: Left={left_speed}, Right={right_speed}")
            
            # Speeds should be reasonably close to target (within 30% for hardware variations)
            assert abs(left_speed - test_speed) < test_speed * 0.3, \
                f"Left speed {left_speed} too far from target {test_speed}"
            assert abs(right_speed - test_speed) < test_speed * 0.3, \
                f"Right speed {right_speed} too far from target {test_speed}"
                
        # Test speed change
        new_speed = test_speed // 2
        result = controller.SpeedM1M2(address, new_speed, new_speed)
        assert result is True, f"Failed to change speed to {new_speed}"
        time.sleep(2.0)
        
        # Safety: Return to zero
        result = controller.SpeedM1M2(address, 0, 0)
        assert result is True, "Failed to return to zero speed"
        time.sleep(1.0)
        
    def test_emergency_stop(self, hardware_controller):
        """Test emergency stop functionality"""
        controller, config = hardware_controller
        address = config['address']
        max_speed = config['max_test_speed']
        
        # Start motors at moderate speed
        test_speed = max_speed // 2
        result = controller.SpeedM1M2(address, test_speed, test_speed)
        assert result is True, "Failed to start motors"
        time.sleep(1.0)
        
        # Verify motors are running
        speed_result = controller.GetSpeeds(address)
        assert speed_result[0], "Failed to read speeds"
        left_speed, right_speed = speed_result[1], speed_result[2]
        assert abs(left_speed) > test_speed // 4, "Left motor not running"
        assert abs(right_speed) > test_speed // 4, "Right motor not running"
        
        # Execute emergency stop
        result = controller.DutyM1M2(address, 0, 0)
        assert result is True, "Emergency stop command failed"
        
        # Verify motors stop quickly
        time.sleep(0.5)
        speed_result = controller.GetSpeeds(address)
        if speed_result[0]:
            left_speed, right_speed = speed_result[1], speed_result[2]
            print(f"Speeds after emergency stop: Left={left_speed}, Right={right_speed}")
            
            # Motors should be stopped or nearly stopped
            assert abs(left_speed) < test_speed // 4, f"Left motor still running: {left_speed}"
            assert abs(right_speed) < test_speed // 4, f"Right motor still running: {right_speed}"


@pytest.mark.hardware
@pytest.mark.skipif(not BASICMICRO_AVAILABLE, reason="Basicmicro library not available")
class TestHardwareInterfaceIntegration:
    """Test hardware interface integration with real hardware"""
    
    def test_hardware_interface_initialization(self, hardware_controller):
        """Test hardware interface initialization with real hardware"""
        controller, config = hardware_controller
        
        # Create hardware interface
        hw = BasicmicroHardwareInterface()
        
        # Manually configure (normally done through URDF)
        hw.port = config['port']
        hw.baud = config['baud']
        hw.address = config['address']
        hw.wheel_radius = config['wheel_radius']
        hw.encoder_counts_per_rev = config['encoder_counts_per_rev']
        hw.gear_ratio = config['gear_ratio']
        hw.motion_strategy = MotionStrategy.SPEED
        
        # Initialize unit converter
        hw.unit_converter = UnitConverter(
            config['wheel_radius'],
            config['encoder_counts_per_rev'],
            config['gear_ratio']
        )
        
        # Use real controller
        hw.controller = controller
        hw.emergency_stop_active = False
        
        # Set up interfaces
        hw.hw_commands_velocities_ = [0.0, 0.0]
        hw.hw_states_positions_ = [0.0, 0.0]
        hw.hw_states_velocities_ = [0.0, 0.0]
        
        # Test sensor reading
        result = hw.read(time=0.0, period=0.1)
        assert result == return_type.OK, "Hardware interface read failed"
        
        print(f"Position state: {hw.hw_states_positions_}")
        print(f"Velocity state: {hw.hw_states_velocities_}")
        
        # Positions and velocities should be valid numbers
        assert all(isinstance(pos, (int, float)) for pos in hw.hw_states_positions_)
        assert all(isinstance(vel, (int, float)) for vel in hw.hw_states_velocities_)
        
    def test_velocity_command_execution(self, hardware_controller):
        """Test velocity command execution through hardware interface"""
        controller, config = hardware_controller
        
        # Set up hardware interface
        hw = BasicmicroHardwareInterface()
        hw.port = config['port']
        hw.baud = config['baud']
        hw.address = config['address']
        hw.wheel_radius = config['wheel_radius']
        hw.encoder_counts_per_rev = config['encoder_counts_per_rev']
        hw.gear_ratio = config['gear_ratio']
        hw.motion_strategy = MotionStrategy.SPEED
        hw.controller = controller
        hw.emergency_stop_active = False
        
        hw.unit_converter = UnitConverter(
            config['wheel_radius'],
            config['encoder_counts_per_rev'],
            config['gear_ratio']
        )
        
        hw.hw_commands_velocities_ = [0.0, 0.0]
        hw.hw_states_positions_ = [0.0, 0.0]
        hw.hw_states_velocities_ = [0.0, 0.0]
        
        # Safety: Start with zero velocity
        result = hw.write(time=0.0, period=0.1)
        assert result == return_type.OK, "Failed to set zero velocity"
        time.sleep(0.5)
        
        # Set small velocity command (rad/s)
        target_velocity = 0.5  # rad/s
        hw.hw_commands_velocities_[0] = target_velocity
        hw.hw_commands_velocities_[1] = target_velocity
        
        # Execute command
        result = hw.write(time=0.0, period=0.1)
        assert result == return_type.OK, "Failed to execute velocity command"
        time.sleep(2.0)  # Allow time for response
        
        # Read actual velocities
        result = hw.read(time=0.0, period=0.1)
        assert result == return_type.OK, "Failed to read velocities"
        
        actual_left_vel = hw.hw_states_velocities_[0]
        actual_right_vel = hw.hw_states_velocities_[1]
        
        print(f"Target velocity: {target_velocity} rad/s")
        print(f"Actual velocities: Left={actual_left_vel:.3f}, Right={actual_right_vel:.3f} rad/s")
        
        # Velocities should be reasonably close to target (within 100% due to hardware variations, cogging, and load differences)
        # Note: The important thing is that velocity commands are processed and motors respond
        tolerance = 1.0
        assert abs(actual_left_vel - target_velocity) < target_velocity * tolerance, \
            f"Left velocity {actual_left_vel} too far from target {target_velocity}"
        assert abs(actual_right_vel - target_velocity) < target_velocity * tolerance, \
            f"Right velocity {actual_right_vel} too far from target {target_velocity}"
            
        # Safety: Return to zero
        hw.hw_commands_velocities_[0] = 0.0
        hw.hw_commands_velocities_[1] = 0.0
        result = hw.write(time=0.0, period=0.1)
        assert result == return_type.OK, "Failed to return to zero velocity"
        time.sleep(1.0)
        
    def test_encoder_position_tracking(self, hardware_controller):
        """Test encoder position tracking during motion"""
        controller, config = hardware_controller
        
        # Set up hardware interface
        hw = BasicmicroHardwareInterface()
        hw.controller = controller
        hw.address = config['address']
        hw.unit_converter = UnitConverter(
            config['wheel_radius'],
            config['encoder_counts_per_rev'],
            config['gear_ratio']
        )
        hw.motion_strategy = MotionStrategy.SPEED
        hw.emergency_stop_active = False
        
        hw.hw_commands_velocities_ = [0.0, 0.0]
        hw.hw_states_positions_ = [0.0, 0.0]
        hw.hw_states_velocities_ = [0.0, 0.0]
        
        # Read initial position
        result = hw.read(time=0.0, period=0.1)
        assert result == return_type.OK, "Failed to read initial position"
        initial_left_pos = hw.hw_states_positions_[0]
        initial_right_pos = hw.hw_states_positions_[1]
        
        print(f"Initial positions: Left={initial_left_pos:.3f}, Right={initial_right_pos:.3f} rad")
        
        # Move motors for short duration
        hw.hw_commands_velocities_[0] = 0.5  # rad/s
        hw.hw_commands_velocities_[1] = 0.5  # rad/s
        
        result = hw.write(time=0.0, period=0.1)
        assert result == return_type.OK, "Failed to start motion"
        
        # Monitor position for several readings
        motion_duration = 3.0  # seconds
        start_time = time.time()
        position_history = []
        
        while time.time() - start_time < motion_duration:
            result = hw.read(time=0.0, period=0.1)
            assert result == return_type.OK, "Failed to read position during motion"
            
            position_history.append({
                'time': time.time() - start_time,
                'left_pos': hw.hw_states_positions_[0],
                'right_pos': hw.hw_states_positions_[1],
                'left_vel': hw.hw_states_velocities_[0],
                'right_vel': hw.hw_states_velocities_[1]
            })
            
            time.sleep(0.1)  # 10Hz monitoring
            
        # Stop motors
        hw.hw_commands_velocities_[0] = 0.0
        hw.hw_commands_velocities_[1] = 0.0
        result = hw.write(time=0.0, period=0.1)
        assert result == return_type.OK, "Failed to stop motion"
        time.sleep(0.5)
        
        # Verify position changed during motion
        final_left_pos = position_history[-1]['left_pos']
        final_right_pos = position_history[-1]['right_pos']
        
        left_position_change = abs(final_left_pos - initial_left_pos)
        right_position_change = abs(final_right_pos - initial_right_pos)
        
        print(f"Position changes: Left={left_position_change:.3f}, Right={right_position_change:.3f} rad")
        
        # Position should have changed during motion (at least 0.1 radians)
        assert left_position_change > 0.1, f"Left position didn't change enough: {left_position_change}"
        assert right_position_change > 0.1, f"Right position didn't change enough: {right_position_change}"
        
        # Print motion summary
        print(f"Motion duration: {motion_duration}s")
        print(f"Samples collected: {len(position_history)}")
        if position_history:
            avg_left_vel = sum(p['left_vel'] for p in position_history) / len(position_history)
            avg_right_vel = sum(p['right_vel'] for p in position_history) / len(position_history)
            print(f"Average velocities: Left={avg_left_vel:.3f}, Right={avg_right_vel:.3f} rad/s")


@pytest.mark.hardware
@pytest.mark.skipif(not BASICMICRO_AVAILABLE, reason="Basicmicro library not available") 
class TestHardwareSafetyFeatures:
    """Test safety features with real hardware"""
    
    def test_communication_timeout_handling(self, hardware_controller):
        """Test handling of communication timeouts"""
        controller, config = hardware_controller
        address = config['address']
        
        # Test normal communication first
        result = controller.GetEncoders(address)
        assert result[0], "Normal communication failed"  # Accept any truthy value
        
        # Simulate timeout by using invalid address
        invalid_address = 0x88  # Likely unused address
        
        # This should timeout or fail
        try:
            result = controller.GetEncoders(invalid_address)
            print(f"Communication to invalid address result: {result[0]}")
        except Exception as e:
            print(f"Expected timeout/error for invalid address: {e}")
            # This is expected behavior
        
        # Verify normal communication still works
        result = controller.GetEncoders(address)
        assert result[0], "Communication recovery failed"  # Accept any truthy value
        
    def test_voltage_monitoring_limits(self, hardware_controller):
        """Test voltage monitoring and limits"""
        controller, config = hardware_controller
        address = config['address']
        
        # Read voltages
        result = controller.GetVolts(address)
        assert result[0], "Failed to read voltages"  # Accept any truthy value
        
        main_voltage, logic_voltage = result[1], result[2]
        main_v = main_voltage / 10.0
        logic_v = logic_voltage / 10.0
        
        print(f"System voltages - Main: {main_v:.1f}V, Logic: {logic_v:.1f}V")
        
        # Check for reasonable voltage levels
        assert main_v > 6.0, f"Main voltage too low: {main_v}V"
        assert main_v < 30.0, f"Main voltage too high: {main_v}V"
        
        # Logic voltage may be 0 if not connected (USB-powered setup)
        if logic_v > 0:
            assert logic_v > 3.0, f"Logic voltage too low: {logic_v}V"
            assert logic_v < 6.0, f"Logic voltage too high: {logic_v}V"
        else:
            print("Logic voltage 0V - USB-powered setup detected")
        
        # Voltage difference should be reasonable
        voltage_diff = abs(main_v - logic_v)
        assert voltage_diff < 20.0, f"Voltage difference too large: {voltage_diff}V"
        
    def test_current_monitoring(self, hardware_controller):
        """Test current monitoring during operation"""
        controller, config = hardware_controller
        address = config['address']
        max_duty = config['max_test_duty']
        
        # Read initial currents (motors stopped)
        result = controller.ReadCurrents(address)
        if result[0]:  # Some controllers may not support current reading
            initial_left_current, initial_right_current = result[1], result[2]
            print(f"Initial currents - Left: {initial_left_current}mA, Right: {initial_right_current}mA")
            
            # Start motors at low duty
            controller.DutyM1M2(address, max_duty // 4, max_duty // 4)
            time.sleep(1.0)
            
            # Read currents under load
            result = controller.ReadCurrents(address)
            if result[0]:
                load_left_current, load_right_current = result[1], result[2]
                print(f"Load currents - Left: {load_left_current}mA, Right: {load_right_current}mA")
                
                # Currents should increase under load (if motors are connected)
                # Note: This may not always be true for unloaded motors
                print(f"Current change - Left: {load_left_current - initial_left_current}mA, "
                      f"Right: {load_right_current - initial_right_current}mA")
                      
            # Stop motors
            controller.DutyM1M2(address, 0, 0)
            time.sleep(0.5)
        else:
            pytest.skip("Controller does not support current monitoring")
            
    def test_temperature_monitoring(self, hardware_controller):
        """Test temperature monitoring"""
        controller, config = hardware_controller
        address = config['address']
        
        # Read temperatures
        result = controller.GetTemps(address)
        if result[0]:  # Some controllers may not support temperature reading
            temp1, temp2 = result[1], result[2]
            temp1_c = temp1 / 10.0  # Convert to Celsius
            temp2_c = temp2 / 10.0
            
            print(f"Temperatures - Sensor 1: {temp1_c:.1f}°C, Sensor 2: {temp2_c:.1f}°C")
            
            # Check for reasonable temperature ranges
            assert temp1_c > -20.0, f"Temperature 1 too low: {temp1_c}°C"
            assert temp1_c < 85.0, f"Temperature 1 too high: {temp1_c}°C"
            assert temp2_c > -20.0, f"Temperature 2 too low: {temp2_c}°C"
            assert temp2_c < 85.0, f"Temperature 2 too high: {temp2_c}°C"
            
            # Temperature difference should be reasonable
            temp_diff = abs(temp1_c - temp2_c)
            assert temp_diff < 50.0, f"Temperature difference too large: {temp_diff}°C"
        else:
            pytest.skip("Controller does not support temperature monitoring")