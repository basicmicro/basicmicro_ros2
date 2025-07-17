"""
Mock Basicmicro Controller for Hardware-Free Testing

This module provides a comprehensive mock implementation of the Basicmicro
controller interface for testing the ROS2 driver without requiring actual
hardware. The mock simulates realistic behavior including:

- Command execution with realistic timing delays
- Deterministic sensor readings for testing logic
- Error simulation for testing error handling
- Buffer management simulation
- Controller type simulation (RoboClaw vs MCP)

Author: Your Name
License: Apache-2.0
"""

import time
import threading
from typing import Tuple, Optional, Dict, Any
from enum import Enum
import math


class ControllerType(Enum):
    """Simulated controller types"""
    ROBOCLAW = "RoboClaw"
    MCP = "MCP"


class MockBasicmicro:
    """
    Mock Basicmicro controller for testing without hardware.
    
    Simulates all major Basicmicro library commands and responses with
    realistic behavior patterns for comprehensive testing.
    """
    
    def __init__(self, port: str, baud: int = 38400):
        """Initialize mock controller"""
        self.port = port
        self.baud = baud
        self.connected = False
        self.address = 0x80
        
        # Simulated hardware state
        self._encoder_counts = [0, 0]  # Left, Right encoder counts
        self._encoder_speeds = [0, 0]  # Left, Right speeds (counts/sec)
        self._target_speeds = [0, 0]   # Target speeds for speed control
        self._duty_cycles = [0, 0]     # Current duty cycles (-32767 to 32767)
        self._voltages = [12.0, 5.0]   # Main battery, Logic battery (volts)
        self._currents = [0.0, 0.0]    # Motor currents (amps)
        self._temperatures = [25.0, 25.0]  # Temperature readings (°C)
        
        # Motion control state
        self._motion_strategy = "speed_accel"
        self._default_acceleration = 1000
        self._max_speed = 5000
        self._buffer_depth = 4
        
        # Buffer simulation
        self._command_buffers = [[], []]  # Separate buffers for each motor
        self._executing_command = [False, False]
        self._buffer_lock = threading.Lock()
        
        # Position control and servo state
        self._position_mode = False
        self._target_positions = [0, 0]
        self._position_errors = [0, 0]
        self._speed_errors = [0, 0]
        self._position_limits_enabled = False
        self._position_limits = [-1000000, 1000000, -1000000, 1000000]  # left_min, left_max, right_min, right_max
        
        # Controller type simulation
        self._controller_type = ControllerType.ROBOCLAW
        self._version_string = "USB RoboClaw 2x15A v4.1.34"
        self._pin_functions = {
            'home_pin': 'auto_mode',  # or 'user_mode'
            'limit_switches': 'both_directions'  # or 'forward_only', 'backward_only'
        }
        
        # Error simulation
        self._communication_failure_rate = 0.0  # 0.0 = no failures, 1.0 = always fail
        self._error_state = False
        self._error_limits_exceeded = False
        
        # Performance simulation
        self._command_delay = 0.001  # 1ms base delay
        self._communication_delay = 0.0005 if baud > 38400 else 0.001  # Additional delay for lower baud rates
        
        # Start simulation thread
        self._simulation_thread = None
        self._simulation_running = False
        
    def Open(self) -> bool:
        """Open connection to mock controller"""
        time.sleep(0.01)  # Simulate connection time
        self.connected = True
        self._start_simulation()
        return True
        
    def Close(self) -> bool:
        """Close connection to mock controller"""
        self.connected = False
        self._stop_simulation()
        return True
        
    def __enter__(self):
        """Context manager entry"""
        self.Open()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()
        
    def _start_simulation(self):
        """Start background simulation thread"""
        if not self._simulation_running:
            self._simulation_running = True
            self._simulation_thread = threading.Thread(target=self._simulation_loop, daemon=True)
            self._simulation_thread.start()
            
    def _stop_simulation(self):
        """Stop background simulation thread"""
        self._simulation_running = False
        if self._simulation_thread:
            self._simulation_thread.join(timeout=1.0)
            
    def _simulation_loop(self):
        """Background simulation of motor dynamics"""
        last_time = time.time()
        
        while self._simulation_running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Simulate motor dynamics
            for i in range(2):
                # Update encoder counts based on current speed
                speed_counts_per_sec = self._encoder_speeds[i]
                self._encoder_counts[i] += int(speed_counts_per_sec * dt)
                
                # Simulate acceleration towards target speeds
                target_speed = self._target_speeds[i]
                current_speed = self._encoder_speeds[i]
                speed_diff = target_speed - current_speed
                
                # Apply simulated acceleration
                max_accel = self._default_acceleration * dt
                if abs(speed_diff) <= max_accel:
                    self._encoder_speeds[i] = target_speed
                else:
                    accel_direction = 1 if speed_diff > 0 else -1
                    self._encoder_speeds[i] += accel_direction * max_accel
                    
                # Update position errors for servo mode
                if self._position_mode:
                    self._position_errors[i] = self._target_positions[i] - self._encoder_counts[i]
                    
                # Update speed errors
                self._speed_errors[i] = target_speed - self._encoder_speeds[i]
                
            # Process buffered commands
            self._process_command_buffers()
            
            # Simulate temperature and current based on motor activity
            self._update_sensor_readings()
            
            time.sleep(0.01)  # 100Hz simulation rate
            
    def _process_command_buffers(self):
        """Process buffered commands"""
        with self._buffer_lock:
            for motor_idx in range(2):
                if self._command_buffers[motor_idx] and not self._executing_command[motor_idx]:
                    # Start executing next command
                    command = self._command_buffers[motor_idx].pop(0)
                    self._executing_command[motor_idx] = True
                    # Simulate command execution (simplified)
                    threading.Timer(0.1, lambda: self._complete_command(motor_idx)).start()
                    
    def _complete_command(self, motor_idx: int):
        """Mark command as completed"""
        self._executing_command[motor_idx] = False
        
    def _update_sensor_readings(self):
        """Update simulated sensor readings"""
        # Simulate current based on motor load
        for i in range(2):
            motor_load = abs(self._encoder_speeds[i]) / 5000.0  # Normalized load
            self._currents[i] = motor_load * 3.0  # Max 3A current
            
        # Simulate temperature rise with motor usage
        ambient_temp = 25.0
        for i in range(2):
            heat_generation = self._currents[i] * 2.0  # Simple heating model
            self._temperatures[i] = ambient_temp + heat_generation
            
    def _simulate_communication_delay(self):
        """Simulate realistic communication timing"""
        delay = self._command_delay + self._communication_delay
        time.sleep(delay)
        
    def _check_communication_failure(self) -> bool:
        """Simulate communication failures for testing"""
        import random
        return random.random() < self._communication_failure_rate
        
    # Core Motion Commands
    def DutyM1M2(self, address: int, left_duty: int, right_duty: int) -> bool:
        """Set duty cycle for both motors"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return False
            
        # Clear buffers and stop position mode
        with self._buffer_lock:
            self._command_buffers = [[], []]
            self._executing_command = [False, False]
            
        self._position_mode = False
        self._duty_cycles = [left_duty, right_duty]
        
        # Convert duty to approximate speed for simulation
        for i in range(2):
            duty_percent = self._duty_cycles[i] / 32767.0
            self._target_speeds[i] = int(duty_percent * self._max_speed)
            
        return True
        
    def DutyAccelM1M2(self, address: int, accel1: int, duty1: int, accel2: int, duty2: int) -> bool:
        """Set duty cycle with acceleration for both motors"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return False
            
        # Simulate smooth acceleration to target duty
        # Implementation simplified for mock
        return self.DutyM1M2(address, duty1, duty2)
        
    def SpeedM1M2(self, address: int, left_speed: int, right_speed: int) -> bool:
        """Set speed for both motors"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return False
            
        self._position_mode = False
        self._target_speeds = [left_speed, right_speed]
        return True
        
    def SpeedAccelM1M2(self, address: int, accel: int, left_speed: int, right_speed: int) -> bool:
        """Set speed with acceleration for both motors"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return False
            
        self._default_acceleration = accel
        return self.SpeedM1M2(address, left_speed, right_speed)
        
    def SpeedAccelDistanceM1M2(self, address: int, accel: int, speed1: int, distance1: int, 
                              speed2: int, distance2: int, buffer_cmd: int = 0) -> bool:
        """Execute distance-based movement"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return False
            
        command = {
            'type': 'distance',
            'accel': accel,
            'speeds': [speed1, speed2],
            'distances': [distance1, distance2],
            'start_positions': self._encoder_counts.copy()
        }
        
        if buffer_cmd:
            with self._buffer_lock:
                self._command_buffers[0].append(command)
                self._command_buffers[1].append(command)
        else:
            # Execute immediately
            self._target_speeds = [speed1, speed2]
            
        return True
        
    def SpeedAccelDeccelPositionM1M2(self, address: int, accel1: int, speed1: int, deccel1: int, position1: int,
                                    accel2: int, speed2: int, deccel2: int, position2: int, buffer_cmd: int = 0) -> bool:
        """Execute absolute position movement (servo mode)"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return False
            
        self._position_mode = True
        command = {
            'type': 'position',
            'accels': [accel1, accel2],
            'max_speeds': [speed1, speed2],
            'deccels': [deccel1, deccel2],
            'positions': [position1, position2]
        }
        
        if buffer_cmd:
            with self._buffer_lock:
                self._command_buffers[0].append(command)
                self._command_buffers[1].append(command)
        else:
            # Execute immediately
            self._target_positions = [position1, position2]
            
        return True
        
    # Sensor Reading Commands
    def GetEncoders(self, address: int) -> Tuple[bool, int, int]:
        """Read encoder counts"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, 0, 0)
            
        return (True, self._encoder_counts[0], self._encoder_counts[1])
        
    def GetSpeeds(self, address: int) -> Tuple[bool, int, int]:
        """Read motor speeds"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, 0, 0)
            
        return (True, self._encoder_speeds[0], self._encoder_speeds[1])
        
    def GetISpeeds(self, address: int) -> Tuple[bool, int, int]:
        """Read instantaneous speeds"""
        # Same as GetSpeeds for mock
        return self.GetSpeeds(address)
        
    def GetVolts(self, address: int) -> Tuple[bool, float, float]:
        """Read battery voltages"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, 0.0, 0.0)
            
        return (True, self._voltages[0], self._voltages[1])
        
    def ReadCurrents(self, address: int) -> Tuple[bool, float, float]:
        """Read motor currents"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, 0.0, 0.0)
            
        return (True, self._currents[0], self._currents[1])
        
    def GetTemps(self, address: int) -> Tuple[bool, float, float]:
        """Read temperature sensors"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, 0.0, 0.0)
            
        return (True, self._temperatures[0], self._temperatures[1])
        
    # Servo-specific Commands
    def GetPosErrors(self, address: int) -> Tuple[bool, int, int]:
        """Read position errors for servo mode"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, 0, 0)
            
        return (True, self._position_errors[0], self._position_errors[1])
        
    def GetSpeedErrors(self, address: int) -> Tuple[bool, int, int]:
        """Read speed errors"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, 0, 0)
            
        return (True, self._speed_errors[0], self._speed_errors[1])
        
    # Buffer Management
    def ReadBuffers(self, address: int) -> Tuple[bool, int]:
        """Read buffer status"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, 0)
            
        with self._buffer_lock:
            buffer_count = len(self._command_buffers[0])  # Assume both buffers same size
            
            if buffer_count == 0 and not any(self._executing_command):
                return (True, 0xFF)  # Empty, no execution
            elif buffer_count == 0 and any(self._executing_command):
                return (True, 0)  # Executing last command
            else:
                return (True, buffer_count)  # Commands in buffer
                
    # Controller Information
    def GetVersion(self, address: int) -> Tuple[bool, str]:
        """Get controller version string"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, "")
            
        return (True, self._version_string)
        
    def ReadVersion(self, address: int) -> Tuple[bool, str]:
        """Get controller version string (alias for GetVersion to match Basicmicro API)"""
        return self.GetVersion(address)
        
    def GetPinFunctions(self, address: int) -> Tuple[bool, Dict[str, Any]]:
        """Get pin configuration (mock implementation)"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, {})
            
        return (True, self._pin_functions)
        
    # Error and Status
    def ReadError(self, address: int) -> Tuple[bool, bool]:
        """Read error state"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, False)
            
        return (True, self._error_limits_exceeded)
        
    def GetStatus(self, address: int) -> Tuple[bool, Dict[str, Any]]:
        """Get comprehensive status"""
        self._simulate_communication_delay()
        if self._check_communication_failure():
            return (False, {})
            
        status = {
            'encoders': self._encoder_counts,
            'speeds': self._encoder_speeds,
            'voltages': self._voltages,
            'currents': self._currents,
            'temperatures': self._temperatures,
            'position_errors': self._position_errors,
            'speed_errors': self._speed_errors,
            'buffer_status': len(self._command_buffers[0]),
            'error_state': self._error_limits_exceeded
        }
        return (True, status)
        
    # Mock-specific methods for testing
    def set_communication_failure_rate(self, rate: float):
        """Set communication failure rate for testing error handling"""
        self._communication_failure_rate = max(0.0, min(1.0, rate))
        
    def set_controller_type(self, controller_type: ControllerType):
        """Set simulated controller type"""
        self._controller_type = controller_type
        if controller_type == ControllerType.ROBOCLAW:
            self._version_string = "USB RoboClaw 2x15A v4.1.34"
        else:
            self._version_string = "MCP Advanced Motor Controller v2.3.1"
            
    def trigger_error_state(self, enable: bool = True):
        """Trigger error state for testing error handling"""
        self._error_limits_exceeded = enable
        
    def reset_encoder_counts(self):
        """Reset encoder counts to zero"""
        self._encoder_counts = [0, 0]
        
    def set_encoder_counts(self, left: int, right: int):
        """Set specific encoder counts for testing"""
        self._encoder_counts = [left, right]
        
    def get_internal_state(self) -> Dict[str, Any]:
        """Get complete internal state for testing verification"""
        return {
            'encoder_counts': self._encoder_counts,
            'encoder_speeds': self._encoder_speeds,
            'target_speeds': self._target_speeds,
            'duty_cycles': self._duty_cycles,
            'voltages': self._voltages,
            'currents': self._currents,
            'temperatures': self._temperatures,
            'position_mode': self._position_mode,
            'target_positions': self._target_positions,
            'position_errors': self._position_errors,
            'speed_errors': self._speed_errors,
            'buffer_counts': [len(buf) for buf in self._command_buffers],
            'executing_commands': self._executing_command,
            'controller_type': self._controller_type,
            'error_state': self._error_limits_exceeded
        }


# Convenience function for creating mock instance
def create_mock_controller(port: str = "/dev/ttyMOCK", baud: int = 38400, 
                          controller_type: ControllerType = ControllerType.ROBOCLAW) -> MockBasicmicro:
    """Create a configured mock controller for testing"""
    mock = MockBasicmicro(port, baud)
    mock.set_controller_type(controller_type)
    return mock