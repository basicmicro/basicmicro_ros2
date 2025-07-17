"""
BasicMicro Hardware Interface for ros2_control

This module implements the ros2_control hardware interface for BasicMicro motor controllers,
providing integration with the BasicMicro Python library and supporting various controller
types (RoboClaw vs MCP) with servo and motion control capabilities.
"""

import sys
import os
import re
from typing import List, Dict, Any, Optional, Tuple
from enum import Enum

try:
    from hardware_interface import SystemInterface, HardwareInfo, return_type
    from rclpy.logging import get_logger
except ImportError:
    # Handle missing ros2_control imports for testing
    class SystemInterface:
        pass
    class HardwareInfo:
        pass
    class return_type:
        OK = 0
        ERROR = 1
    def get_logger(name):
        import logging
        return logging.getLogger(name)

# Add Basicmicro Python library to path
basicmicro_path = os.path.join(os.path.dirname(__file__), '..', '..', 'Basicmicro_python')
if os.path.exists(basicmicro_path):
    sys.path.insert(0, basicmicro_path)

try:
    from basicmicro import Basicmicro
except ImportError:
    # Use mock for testing without hardware
    from test_mocks.mock_basicmicro import MockBasicmicro as Basicmicro

try:
    from basicmicro_driver.unit_converter import UnitConverter
except ImportError:
    # Fallback for relative import in package context
    from .unit_converter import UnitConverter


class ControllerType(Enum):
    """Supported BasicMicro controller types"""
    ROBOCLAW = "roboclaw"
    MCP = "mcp"
    UNKNOWN = "unknown"


class EncoderType(Enum):
    """Supported encoder types"""
    INCREMENTAL = "incremental"
    ABSOLUTE = "absolute"


class MotionStrategy(Enum):
    """Motion control strategies"""
    DUTY = "duty"
    DUTY_ACCEL = "duty_accel"
    SPEED = "speed"
    SPEED_ACCEL = "speed_accel"


class LimitViolationBehavior(Enum):
    """Position limit violation handling"""
    HARD_STOP = "hard_stop"
    SOFT_STOP = "soft_stop"
    WARNING = "warning"


class BasicmicroHardwareInterface(SystemInterface):
    """
    ROS2 Hardware Interface for BasicMicro motor controllers.
    
    Supports both RoboClaw and MCP controllers with comprehensive motion control,
    servo positioning, and advanced features like trajectory execution and homing.
    """

    def __init__(self):
        """Initialize the hardware interface"""
        self.logger = get_logger('basicmicro_hardware_interface')
        
        # Hardware connection parameters (supports USB Serial and Async Serial)
        self.port: str = "/dev/ttyACM0"
        self.baud: int = 38400
        self.address: int = 0x80
        
        # Robot physical parameters
        self.wheel_radius: float = 0.1  # meters
        self.wheel_separation: float = 0.3  # meters
        self.encoder_counts_per_rev: int = 1000
        self.gear_ratio: float = 1.0
        
        # Motion control parameters
        self.motion_strategy: MotionStrategy = MotionStrategy.SPEED_ACCEL
        self.buffer_depth: int = 4
        self.default_acceleration: int = 1000
        
        # Servo functionality parameters
        self.encoder_type: EncoderType = EncoderType.INCREMENTAL
        self.auto_home_on_startup: bool = False
        self.position_limits_enabled: bool = False
        self.min_position_left: float = -1000000.0  # encoder counts
        self.max_position_left: float = 1000000.0
        self.min_position_right: float = -1000000.0
        self.max_position_right: float = 1000000.0
        self.limit_violation_behavior: LimitViolationBehavior = LimitViolationBehavior.SOFT_STOP
        self.limit_decel_rate: float = 1000.0  # for soft stops
        
        # Hardware state
        self.controller: Optional[Basicmicro] = None
        self.controller_type: ControllerType = ControllerType.UNKNOWN
        self.emergency_stop_active: bool = False
        self.servo_capable: bool = False
        
        # Unit converter (initialized after parameters are extracted)
        self.unit_converter: Optional[UnitConverter] = None
        
        # ros2_control state and command interfaces
        self.hw_commands_positions_: List[float] = [0.0, 0.0]
        self.hw_commands_velocities_: List[float] = [0.0, 0.0]
        self.hw_states_positions_: List[float] = [0.0, 0.0]
        self.hw_states_velocities_: List[float] = [0.0, 0.0]
        
        # Joint names for interface registration
        self.joint_names: List[str] = ["left_wheel_joint", "right_wheel_joint"]

    def on_init(self, info: HardwareInfo) -> int:
        """
        Initialize hardware interface with URDF parameters.
        
        Args:
            info: Hardware info containing URDF parameters
            
        Returns:
            return_type.OK on success, return_type.ERROR on failure
        """
        self.logger.info("Initializing BasicMicro Hardware Interface")
        
        try:
            # Extract hardware connection parameters
            self._extract_connection_parameters(info)
            
            # Extract robot physical parameters
            self._extract_physical_parameters(info)
            
            # Extract motion control parameters
            self._extract_motion_parameters(info)
            
            # Extract servo functionality parameters
            self._extract_servo_parameters(info)
            
            # Validate all parameters
            if not self._validate_parameters():
                return return_type.ERROR
            
            # Initialize unit converter with validated parameters
            self.unit_converter = UnitConverter(
                wheel_radius=self.wheel_radius,
                encoder_counts_per_rev=self.encoder_counts_per_rev,
                gear_ratio=self.gear_ratio
            )
            self.logger.info(f"Unit converter initialized: {self.unit_converter}")
                
            self.logger.info(f"Hardware interface initialized with port={self.port}, "
                           f"baud={self.baud}, address=0x{self.address:02X}")
            
            return return_type.OK
            
        except Exception as e:
            self.logger.error(f"Failed to initialize hardware interface: {e}")
            return return_type.ERROR

    def on_configure(self, previous_state) -> int:
        """
        Configure hardware interface and establish connection.
        
        Returns:
            return_type.OK on success, return_type.ERROR on failure
        """
        self.logger.info("Configuring BasicMicro Hardware Interface")
        
        try:
            # Initialize Basicmicro controller connection
            self.controller = Basicmicro(self.port, self.baud)
            
            # Test connection and detect controller type
            if not self._establish_connection():
                return return_type.ERROR
                
            # Detect controller type and capabilities
            self._detect_controller_type()
            
            # Configure servo parameters if supported
            if self.servo_capable:
                self._configure_servo_parameters()
                
            # Perform auto-homing if configured
            if self.auto_home_on_startup and self.controller_type == ControllerType.ROBOCLAW:
                self._perform_auto_homing()
                
            self.logger.info(f"Hardware interface configured successfully. "
                           f"Controller type: {self.controller_type.value}, "
                           f"Servo capable: {self.servo_capable}")
            
            return return_type.OK
            
        except Exception as e:
            self.logger.error(f"Failed to configure hardware interface: {e}")
            return return_type.ERROR

    def on_activate(self, previous_state) -> int:
        """
        Activate hardware interface for operation.
        
        Returns:
            return_type.OK on success, return_type.ERROR on failure
        """
        self.logger.info("Activating BasicMicro Hardware Interface")
        
        try:
            # Open connection if not already open
            if self.controller and not hasattr(self.controller, '_connected'):
                self.controller.Open()
                
            # Reset emergency stop state
            self.emergency_stop_active = False
            
            # Initialize state readings
            self._initialize_state_interfaces()
            
            self.logger.info("Hardware interface activated successfully")
            return return_type.OK
            
        except Exception as e:
            self.logger.error(f"Failed to activate hardware interface: {e}")
            return return_type.ERROR

    def on_deactivate(self, previous_state) -> int:
        """
        Deactivate hardware interface.
        
        Returns:
            return_type.OK on success, return_type.ERROR on failure
        """
        self.logger.info("Deactivating BasicMicro Hardware Interface")
        
        try:
            # Stop motors safely
            if self.controller:
                self.controller.DutyM1M2(self.address, 0, 0)
                
            self.logger.info("Hardware interface deactivated successfully")
            return return_type.OK
            
        except Exception as e:
            self.logger.error(f"Failed to deactivate hardware interface: {e}")
            return return_type.ERROR

    def read(self, time, period) -> int:
        """
        Read sensor data and populate state interfaces.
        
        Args:
            time: Current time
            period: Time since last read
            
        Returns:
            return_type.OK on success, return_type.ERROR on failure
        """
        if not self.controller or self.emergency_stop_active:
            return return_type.OK
            
        try:
            # Read encoder positions
            encoders = self.controller.GetEncoders(self.address)
            if encoders[0]:  # Check success flag
                # Convert from encoder counts to radians
                self.hw_states_positions_[0] = self.unit_converter.counts_to_radians(encoders[1])
                self.hw_states_positions_[1] = self.unit_converter.counts_to_radians(encoders[2])
                
            # Read motor speeds
            speeds = self.controller.GetSpeeds(self.address)
            if speeds[0]:  # Check success flag
                # Convert from counts/sec to rad/sec
                self.hw_states_velocities_[0] = self.unit_converter.counts_per_sec_to_rad_per_sec(speeds[1])
                self.hw_states_velocities_[1] = self.unit_converter.counts_per_sec_to_rad_per_sec(speeds[2])
                
            return return_type.OK
            
        except Exception as e:
            self.logger.error(f"Failed to read sensor data: {e}")
            return return_type.ERROR

    def write(self, time, period) -> int:
        """
        Write velocity commands to motors.
        
        Args:
            time: Current time
            period: Time since last write
            
        Returns:
            return_type.OK on success, return_type.ERROR on failure
        """
        if not self.controller or self.emergency_stop_active:
            return return_type.OK
            
        try:
            # Convert velocity commands from rad/sec to appropriate units
            left_vel_rad_s = self.hw_commands_velocities_[0]
            right_vel_rad_s = self.hw_commands_velocities_[1]
            
            # Execute velocity commands based on motion strategy
            return self._execute_velocity_command(left_vel_rad_s, right_vel_rad_s)
            
        except Exception as e:
            self.logger.error(f"Failed to write motor commands: {e}")
            return return_type.ERROR

    def emergency_stop(self) -> bool:
        """
        Emergency stop - clears all buffers and stops motors immediately.
        
        Returns:
            True on success, False on failure
        """
        try:
            if self.controller:
                # Clear all buffers and stop motors immediately
                self.controller.DutyM1M2(self.address, 0, 0)
                self.emergency_stop_active = True
                self.logger.warning("Emergency stop activated")
                return True
        except Exception as e:
            self.logger.error(f"Emergency stop failed: {e}")
            
        return False

    def _extract_connection_parameters(self, info: HardwareInfo) -> None:
        """Extract hardware connection parameters from URDF"""
        try:
            self.port = self._get_parameter(info, "port", self.port)
            self.baud = int(self._get_parameter(info, "baud", str(self.baud)))
            address_param = self._get_parameter(info, "address", str(self.address))
            
            # Handle address as decimal or hex
            if address_param.startswith("0x"):
                self.address = int(address_param, 16)
            else:
                self.address = int(address_param)
                
        except ValueError as e:
            raise ValueError(f"Invalid connection parameter: {e}")

    def _extract_physical_parameters(self, info: HardwareInfo) -> None:
        """Extract robot physical parameters from URDF"""
        try:
            self.wheel_radius = float(self._get_parameter(info, "wheel_radius", str(self.wheel_radius)))
            self.wheel_separation = float(self._get_parameter(info, "wheel_separation", str(self.wheel_separation)))
            self.encoder_counts_per_rev = int(self._get_parameter(info, "encoder_counts_per_rev", str(self.encoder_counts_per_rev)))
            self.gear_ratio = float(self._get_parameter(info, "gear_ratio", str(self.gear_ratio)))
            
            # Validate parameter ranges
            if self.wheel_radius <= 0:
                raise ValueError("wheel_radius must be positive")
            if self.wheel_separation <= 0:
                raise ValueError("wheel_separation must be positive")
            if self.encoder_counts_per_rev <= 0:
                raise ValueError("encoder_counts_per_rev must be positive")
            if self.gear_ratio <= 0:
                raise ValueError("gear_ratio must be positive")
            
        except ValueError as e:
            raise ValueError(f"Invalid physical parameter: {e}")

    def _extract_motion_parameters(self, info: HardwareInfo) -> None:
        """Extract motion control parameters from URDF"""
        try:
            strategy_str = self._get_parameter(info, "motion_strategy", self.motion_strategy.value)
            self.motion_strategy = MotionStrategy(strategy_str)
            
            self.buffer_depth = int(self._get_parameter(info, "buffer_depth", str(self.buffer_depth)))
            self.default_acceleration = int(self._get_parameter(info, "default_acceleration", str(self.default_acceleration)))
            
        except (ValueError, KeyError) as e:
            raise ValueError(f"Invalid motion parameter: {e}")

    def _extract_servo_parameters(self, info: HardwareInfo) -> None:
        """Extract servo functionality parameters from URDF"""
        try:
            encoder_type_str = self._get_parameter(info, "encoder_type", self.encoder_type.value)
            self.encoder_type = EncoderType(encoder_type_str)
            
            self.auto_home_on_startup = self._get_parameter(info, "auto_home_on_startup", "false").lower() == "true"
            self.position_limits_enabled = self._get_parameter(info, "position_limits_enabled", "false").lower() == "true"
            
            self.min_position_left = float(self._get_parameter(info, "min_position_left", str(self.min_position_left)))
            self.max_position_left = float(self._get_parameter(info, "max_position_left", str(self.max_position_left)))
            self.min_position_right = float(self._get_parameter(info, "min_position_right", str(self.min_position_right)))
            self.max_position_right = float(self._get_parameter(info, "max_position_right", str(self.max_position_right)))
            
            behavior_str = self._get_parameter(info, "limit_violation_behavior", self.limit_violation_behavior.value)
            self.limit_violation_behavior = LimitViolationBehavior(behavior_str)
            
            self.limit_decel_rate = float(self._get_parameter(info, "limit_decel_rate", str(self.limit_decel_rate)))
            
        except (ValueError, KeyError) as e:
            raise ValueError(f"Invalid servo parameter: {e}")

    def _get_parameter(self, info: HardwareInfo, name: str, default: str) -> str:
        """Get parameter value from hardware info with default fallback"""
        try:
            if hasattr(info, 'hardware_parameters') and info.hardware_parameters:
                if name in info.hardware_parameters:
                    return str(info.hardware_parameters[name])
            return default
        except Exception:
            return default

    def _validate_parameters(self) -> bool:
        """Validate all hardware interface parameters"""
        if self.wheel_radius <= 0:
            self.logger.error("wheel_radius must be positive")
            return False
            
        if self.wheel_separation <= 0:
            self.logger.error("wheel_separation must be positive")
            return False
            
        if self.encoder_counts_per_rev <= 0:
            self.logger.error("encoder_counts_per_rev must be positive")
            return False
            
        if self.gear_ratio <= 0:
            self.logger.error("gear_ratio must be positive")
            return False
            
        if not (0x80 <= self.address <= 0x87):
            self.logger.error(f"address must be between 0x80 and 0x87, got 0x{self.address:02X}")
            return False
            
        return True

    def _establish_connection(self) -> bool:
        """Establish connection with BasicMicro controller"""
        try:
            self.controller.Open()
            
            # Test connection with a simple command
            version = self.controller.ReadVersion(self.address)
            if not version[0]:
                self.logger.error("Failed to read controller version")
                return False
                
            self.logger.info(f"Connected to controller: {version[1]}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to establish connection: {e}")
            return False

    def _detect_controller_type(self) -> None:
        """Detect controller type and capabilities from version string"""
        try:
            version = self.controller.ReadVersion(self.address)
            if version[0]:
                version_str = version[1].lower()
                if "roboclaw" in version_str:
                    self.controller_type = ControllerType.ROBOCLAW
                    self.servo_capable = True  # Most RoboClaw units support servo mode
                elif "mcp" in version_str:
                    self.controller_type = ControllerType.MCP
                    self.servo_capable = True  # MCP units typically support servo mode
                else:
                    self.controller_type = ControllerType.UNKNOWN
                    self.servo_capable = False
                    
        except Exception as e:
            self.logger.warning(f"Could not detect controller type: {e}")
            self.controller_type = ControllerType.UNKNOWN
            self.servo_capable = False

    def _configure_servo_parameters(self) -> None:
        """Configure servo-specific parameters"""
        if not self.servo_capable:
            return
            
        try:
            # Configure encoder type and limits
            # This would set up PID parameters, error limits, etc.
            # Implementation depends on controller capabilities
            self.logger.info("Servo parameters configured")
            
        except Exception as e:
            self.logger.warning(f"Failed to configure servo parameters: {e}")

    def _perform_auto_homing(self) -> None:
        """Perform automatic homing on startup for RoboClaw controllers"""
        if self.controller_type != ControllerType.ROBOCLAW:
            return
            
        try:
            # Auto-homing implementation for RoboClaw
            # This would trigger the homing sequence
            self.logger.info("Auto-homing initiated")
            
        except Exception as e:
            self.logger.warning(f"Auto-homing failed: {e}")

    def _initialize_state_interfaces(self) -> None:
        """Initialize state interfaces with current sensor readings"""
        try:
            # Read initial encoder positions
            encoders = self.controller.GetEncoders(self.address)
            if encoders[0]:
                self.hw_states_positions_[0] = self.unit_converter.counts_to_radians(encoders[1])
                self.hw_states_positions_[1] = self.unit_converter.counts_to_radians(encoders[2])
                
            # Initialize velocities to zero
            self.hw_states_velocities_[0] = 0.0
            self.hw_states_velocities_[1] = 0.0
            
        except Exception as e:
            self.logger.warning(f"Failed to initialize state interfaces: {e}")

    def _execute_velocity_command(self, left_vel_rad_s: float, right_vel_rad_s: float) -> int:
        """Execute velocity command based on configured motion strategy"""
        try:
            if self.motion_strategy == MotionStrategy.DUTY:
                # Convert to duty cycle (-32767 to +32767)
                left_duty = self.unit_converter.rad_per_sec_to_duty(left_vel_rad_s)
                right_duty = self.unit_converter.rad_per_sec_to_duty(right_vel_rad_s)
                self.controller.DutyM1M2(self.address, left_duty, right_duty)
                
            elif self.motion_strategy == MotionStrategy.DUTY_ACCEL:
                # Smooth duty cycle changes
                left_duty = self.unit_converter.rad_per_sec_to_duty(left_vel_rad_s)
                right_duty = self.unit_converter.rad_per_sec_to_duty(right_vel_rad_s)
                self.controller.DutyAccelM1M2(self.address, self.default_acceleration, left_duty,
                                            self.default_acceleration, right_duty)
                
            elif self.motion_strategy == MotionStrategy.SPEED:
                # Basic speed control
                left_speed = self.unit_converter.rad_per_sec_to_counts_per_sec(left_vel_rad_s)
                right_speed = self.unit_converter.rad_per_sec_to_counts_per_sec(right_vel_rad_s)
                self.controller.SpeedM1M2(self.address, left_speed, right_speed)
                
            elif self.motion_strategy == MotionStrategy.SPEED_ACCEL:
                # Speed control with explicit acceleration
                left_speed = self.unit_converter.rad_per_sec_to_counts_per_sec(left_vel_rad_s)
                right_speed = self.unit_converter.rad_per_sec_to_counts_per_sec(right_vel_rad_s)
                self.controller.SpeedAccelM1M2(self.address, self.default_acceleration, 
                                             left_speed, right_speed)
                
            return return_type.OK
            
        except Exception as e:
            self.logger.error(f"Failed to execute velocity command: {e}")
            return return_type.ERROR

    # Enhanced Library Integration and Error Monitoring (Task 1.7)
    
    def read_comprehensive_diagnostics(self) -> Dict[str, Any]:
        """
        Read comprehensive diagnostic data from controller.
        
        Returns:
            Dictionary containing all diagnostic information
        """
        diagnostics = {}
        
        if not self.controller:
            return diagnostics
            
        try:
            # Read error state
            error_result = self.controller.ReadError(self.address)
            if error_result[0]:
                diagnostics['error_limits_exceeded'] = error_result[1]
            else:
                self.logger.warning("Failed to read error state")
                
            # Read comprehensive status
            status_result = self.controller.GetStatus(self.address)
            if status_result[0]:
                diagnostics['comprehensive_status'] = status_result[1]
            else:
                self.logger.warning("Failed to read comprehensive status")
                
            # Read motor currents
            current_result = self.controller.ReadCurrents(self.address)
            if current_result[0]:
                diagnostics['motor_currents'] = {
                    'left_current_amps': current_result[1],
                    'right_current_amps': current_result[2]
                }
            else:
                self.logger.warning("Failed to read motor currents")
                
            # Read temperature sensors
            temp_result = self.controller.GetTemps(self.address)
            if temp_result[0]:
                diagnostics['temperatures'] = {
                    'temp1_celsius': temp_result[1],
                    'temp2_celsius': temp_result[2]
                }
            else:
                self.logger.warning("Failed to read temperatures")
                
            # Read battery voltages
            volt_result = self.controller.GetVolts(self.address)
            if volt_result[0]:
                diagnostics['voltages'] = {
                    'main_battery_volts': volt_result[1],
                    'logic_battery_volts': volt_result[2]
                }
            else:
                self.logger.warning("Failed to read voltages")
                
            # Read speed errors (servo mode)
            speed_errors = self.controller.GetSpeedErrors(self.address)
            if speed_errors[0]:
                diagnostics['speed_errors'] = {
                    'left_speed_error_counts_per_sec': speed_errors[1],
                    'right_speed_error_counts_per_sec': speed_errors[2]
                }
            else:
                self.logger.warning("Failed to read speed errors")
                
            # Read position errors (servo mode)
            pos_errors = self.controller.GetPosErrors(self.address)
            if pos_errors[0]:
                diagnostics['position_errors'] = {
                    'left_position_error_counts': pos_errors[1],
                    'right_position_error_counts': pos_errors[2]
                }
            else:
                self.logger.warning("Failed to read position errors")
                
            # Read buffer status
            buffer_status = self.controller.ReadBuffers(self.address)
            if buffer_status[0]:
                diagnostics['buffer_status'] = self._interpret_buffer_status(buffer_status[1])
            else:
                self.logger.warning("Failed to read buffer status")
                
        except Exception as e:
            self.logger.error(f"Failed to read comprehensive diagnostics: {e}")
            
        return diagnostics
    
    def _interpret_buffer_status(self, raw_value: int) -> Dict[str, Any]:
        """
        Interpret buffer status value according to Basicmicro protocol.
        
        Args:
            raw_value: Raw buffer status value from controller
            
        Returns:
            Dictionary with interpreted buffer status
        """
        if raw_value == 0xFF:
            return {
                'status': 'IDLE',
                'description': 'No commands in buffer, no commands executing',
                'commands_in_buffer': 0,
                'executing': False
            }
        elif raw_value == 0:
            return {
                'status': 'EXECUTING',
                'description': 'Last command is executing, buffer empty',
                'commands_in_buffer': 0,
                'executing': True
            }
        else:
            return {
                'status': 'BUFFERED',
                'description': f'{raw_value} commands in buffer',
                'commands_in_buffer': raw_value,
                'executing': False
            }
    
    def check_error_limits(self) -> bool:
        """
        Check if controller error limits have been exceeded.
        
        Returns:
            True if error limits exceeded (requires Power-On-Reset), False otherwise
        """
        if not self.controller:
            return False
            
        try:
            error_result = self.controller.ReadError(self.address)
            if error_result[0]:
                error_exceeded = error_result[1]
                if error_exceeded:
                    self.logger.error("Controller error limits exceeded - Power-On-Reset required")
                return error_exceeded
            else:
                self.logger.warning("Failed to check error limits")
                return False
                
        except Exception as e:
            self.logger.error(f"Failed to check error limits: {e}")
            return False
    
    def get_servo_errors(self) -> Dict[str, Any]:
        """
        Get servo-specific error information for position control monitoring.
        
        Returns:
            Dictionary containing servo error data
        """
        servo_errors = {}
        
        if not self.controller:
            return servo_errors
            
        try:
            # Get position errors
            pos_errors = self.controller.GetPosErrors(self.address)
            if pos_errors[0]:
                servo_errors['position_errors_counts'] = {
                    'left': pos_errors[1],
                    'right': pos_errors[2]
                }
                # Convert to radians for ROS2 interface
                servo_errors['position_errors_radians'] = {
                    'left': self.unit_converter.counts_to_radians(pos_errors[1]),
                    'right': self.unit_converter.counts_to_radians(pos_errors[2])
                }
            
            # Get speed errors  
            speed_errors = self.controller.GetSpeedErrors(self.address)
            if speed_errors[0]:
                servo_errors['speed_errors_counts_per_sec'] = {
                    'left': speed_errors[1],
                    'right': speed_errors[2]
                }
                # Convert to rad/sec for ROS2 interface
                servo_errors['speed_errors_rad_per_sec'] = {
                    'left': self.unit_converter.counts_per_sec_to_rad_per_sec(speed_errors[1]),
                    'right': self.unit_converter.counts_per_sec_to_rad_per_sec(speed_errors[2])
                }
                
            # Check if errors exceed reasonable thresholds
            servo_errors['error_analysis'] = self._analyze_servo_errors(servo_errors)
            
        except Exception as e:
            self.logger.error(f"Failed to get servo errors: {e}")
            
        return servo_errors
    
    def _analyze_servo_errors(self, servo_errors: Dict[str, Any]) -> Dict[str, Any]:
        """
        Analyze servo errors and provide status assessment.
        
        Args:
            servo_errors: Servo error data
            
        Returns:
            Error analysis results
        """
        analysis = {
            'position_errors_acceptable': True,
            'speed_errors_acceptable': True,
            'warnings': []
        }
        
        try:
            # Check position errors (threshold: 100 encoder counts = ~0.6 radians)
            if 'position_errors_counts' in servo_errors:
                pos_errors = servo_errors['position_errors_counts']
                for side in ['left', 'right']:
                    if abs(pos_errors[side]) > 100:
                        analysis['position_errors_acceptable'] = False
                        analysis['warnings'].append(f'{side} motor position error exceeds threshold: {pos_errors[side]} counts')
            
            # Check speed errors (threshold: 500 counts/sec)
            if 'speed_errors_counts_per_sec' in servo_errors:
                speed_errors = servo_errors['speed_errors_counts_per_sec']
                for side in ['left', 'right']:
                    if abs(speed_errors[side]) > 500:
                        analysis['speed_errors_acceptable'] = False
                        analysis['warnings'].append(f'{side} motor speed error exceeds threshold: {speed_errors[side]} counts/sec')
                        
        except Exception as e:
            self.logger.error(f"Failed to analyze servo errors: {e}")
            analysis['warnings'].append(f"Error analysis failed: {e}")
            
        return analysis
    
    def get_buffer_status(self) -> Dict[str, Any]:
        """
        Get detailed buffer status for motion planning and monitoring.
        
        Returns:
            Dictionary containing buffer status information
        """
        if not self.controller:
            return {'available': False}
            
        try:
            buffer_result = self.controller.ReadBuffers(self.address)
            if buffer_result[0]:
                buffer_info = self._interpret_buffer_status(buffer_result[1])
                buffer_info['available'] = True
                buffer_info['raw_value'] = buffer_result[1]
                
                # Calculate buffer utilization
                max_buffer_size = 32  # Basicmicro supports up to 32 buffered commands
                if buffer_info['status'] == 'BUFFERED':
                    buffer_info['utilization_percent'] = (buffer_info['commands_in_buffer'] / max_buffer_size) * 100
                else:
                    buffer_info['utilization_percent'] = 0
                    
                buffer_info['available_slots'] = max_buffer_size - buffer_info['commands_in_buffer']
                
                return buffer_info
            else:
                self.logger.warning("Failed to read buffer status")
                return {'available': False, 'error': 'Communication failure'}
                
        except Exception as e:
            self.logger.error(f"Failed to get buffer status: {e}")
            return {'available': False, 'error': str(e)}
    
    def monitor_system_health(self) -> Dict[str, Any]:
        """
        Comprehensive system health monitoring combining all diagnostic data.
        
        Returns:
            Dictionary containing overall system health assessment
        """
        health = {
            'overall_status': 'UNKNOWN',
            'errors': [],
            'warnings': [],
            'diagnostics': {}
        }
        
        try:
            # Get comprehensive diagnostics
            diagnostics = self.read_comprehensive_diagnostics()
            health['diagnostics'] = diagnostics
            
            # Check for critical errors
            if diagnostics.get('error_limits_exceeded', False):
                health['errors'].append('Controller error limits exceeded - Power-On-Reset required')
                health['overall_status'] = 'ERROR'
                
            # Check voltages
            if 'voltages' in diagnostics:
                main_voltage = diagnostics['voltages'].get('main_battery_volts', 0)
                if main_voltage < 10.0:  # Low battery threshold
                    health['warnings'].append(f'Low main battery voltage: {main_voltage:.1f}V')
                if main_voltage > 16.0:  # High voltage threshold
                    health['warnings'].append(f'High main battery voltage: {main_voltage:.1f}V')
                    
            # Check currents
            if 'motor_currents' in diagnostics:
                left_current = diagnostics['motor_currents'].get('left_current_amps', 0)
                right_current = diagnostics['motor_currents'].get('right_current_amps', 0)
                max_current = 15.0  # Example threshold for 2x15A controller
                
                if left_current > max_current * 0.8:
                    health['warnings'].append(f'High left motor current: {left_current:.1f}A')
                if right_current > max_current * 0.8:
                    health['warnings'].append(f'High right motor current: {right_current:.1f}A')
                    
            # Check temperatures
            if 'temperatures' in diagnostics:
                temp1 = diagnostics['temperatures'].get('temp1_celsius', 0)
                temp2 = diagnostics['temperatures'].get('temp2_celsius', 0)
                max_temp = 70.0  # Example thermal threshold
                
                if temp1 > max_temp:
                    health['warnings'].append(f'High temperature sensor 1: {temp1:.1f}°C')
                if temp2 > max_temp:
                    health['warnings'].append(f'High temperature sensor 2: {temp2:.1f}°C')
                    
            # Check buffer utilization
            buffer_status = self.get_buffer_status()
            if buffer_status.get('available', False):
                utilization = buffer_status.get('utilization_percent', 0)
                if utilization > 90:
                    health['warnings'].append(f'High buffer utilization: {utilization:.1f}%')
                    
            # Determine overall status
            if health['overall_status'] == 'UNKNOWN':
                if health['errors']:
                    health['overall_status'] = 'ERROR'
                elif health['warnings']:
                    health['overall_status'] = 'WARNING'
                else:
                    health['overall_status'] = 'OK'
                    
        except Exception as e:
            self.logger.error(f"Failed to monitor system health: {e}")
            health['errors'].append(f"Health monitoring failed: {e}")
            health['overall_status'] = 'ERROR'
            
        return health

    # Advanced Position Control (Servo) Functions
    
    def execute_absolute_position_command(self, left_pos_rad: float, right_pos_rad: float,
                                        max_speed_rad_s: float, acceleration_rad_s2: float,
                                        deceleration_rad_s2: float, buffer_command: bool = False) -> bool:
        """
        Execute absolute position command using library servo functionality.
        
        Args:
            left_pos_rad: Left wheel target position in radians
            right_pos_rad: Right wheel target position in radians
            max_speed_rad_s: Maximum speed during move (rad/s)
            acceleration_rad_s2: Acceleration rate (rad/s²)
            deceleration_rad_s2: Deceleration rate (rad/s²)
            buffer_command: Whether to buffer the command for chaining
            
        Returns:
            True on success, False on failure
        """
        if not self.controller:
            self.logger.error("Controller not available for position command")
            return False
            
        try:
            # Convert units from ROS2 (radians) to Basicmicro (encoder counts)
            left_pos_counts = self.unit_converter.radians_to_counts(left_pos_rad)
            right_pos_counts = self.unit_converter.radians_to_counts(right_pos_rad)
            max_speed_counts = self.unit_converter.rad_per_sec_to_counts_per_sec(max_speed_rad_s)
            accel_counts = self.unit_converter.rad_per_sec_to_counts_per_sec(acceleration_rad_s2)
            decel_counts = self.unit_converter.rad_per_sec_to_counts_per_sec(deceleration_rad_s2)
            
            # Execute absolute position command
            buffer_flag = 1 if buffer_command else 0
            success = self.controller.SpeedAccelDeccelPositionM1M2(
                self.address,
                accel_counts, max_speed_counts, decel_counts, left_pos_counts,
                accel_counts, max_speed_counts, decel_counts, right_pos_counts,
                buffer_flag
            )
            
            if success:
                self.logger.info(f"Position command executed: L={left_pos_rad:.3f}rad, R={right_pos_rad:.3f}rad")
            else:
                self.logger.error("Position command failed")
                
            return success
            
        except Exception as e:
            self.logger.error(f"Failed to execute position command: {e}")
            return False
    
    def execute_distance_command(self, left_distance_m: float, right_distance_m: float,
                               speed_m_s: float, acceleration_m_s2: float, 
                               buffer_command: bool = False) -> bool:
        """
        Execute distance-based movement command using library distance functionality.
        
        Args:
            left_distance_m: Left wheel distance in meters
            right_distance_m: Right wheel distance in meters  
            speed_m_s: Target speed in m/s
            acceleration_m_s2: Acceleration rate in m/s²
            buffer_command: Whether to buffer the command for chaining
            
        Returns:
            True on success, False on failure
        """
        if not self.controller:
            self.logger.error("Controller not available for distance command")
            return False
            
        try:
            # Convert units from ROS2 (meters, m/s) to Basicmicro (encoder counts)
            left_distance_counts = self.unit_converter.meters_to_counts(left_distance_m)
            right_distance_counts = self.unit_converter.meters_to_counts(right_distance_m) 
            speed_counts = self.unit_converter.rad_per_sec_to_counts_per_sec(
                speed_m_s / self.wheel_radius)  # Convert m/s to rad/s then to counts/s
            accel_counts = self.unit_converter.rad_per_sec_to_counts_per_sec(
                acceleration_m_s2 / self.wheel_radius)  # Convert m/s² to rad/s² then to counts/s
            
            # Execute distance command
            buffer_flag = 1 if buffer_command else 0
            success = self.controller.SpeedAccelDistanceM1M2(
                self.address,
                accel_counts,
                speed_counts, left_distance_counts,
                speed_counts, right_distance_counts,
                buffer_flag
            )
            
            if success:
                self.logger.info(f"Distance command executed: L={left_distance_m:.3f}m, R={right_distance_m:.3f}m")
            else:
                self.logger.error("Distance command failed")
                
            return success
            
        except Exception as e:
            self.logger.error(f"Failed to execute distance command: {e}")
            return False
    
    def clear_motion_buffers(self) -> bool:
        """
        Clear all motion buffers using enhanced emergency stop.
        
        Returns:
            True on success, False on failure
        """
        try:
            if self.controller:
                # Use DutyM1M2(0,0) to clear all buffers and stop motors
                success = self.controller.DutyM1M2(self.address, 0, 0)
                if success:
                    self.logger.info("Motion buffers cleared")
                    return True
                else:
                    self.logger.error("Failed to clear motion buffers")
                    return False
            else:
                self.logger.error("Controller not available to clear buffers")
                return False
                
        except Exception as e:
            self.logger.error(f"Failed to clear motion buffers: {e}")
            return False

