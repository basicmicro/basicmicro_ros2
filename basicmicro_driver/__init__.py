"""
Basicmicro Driver for ROS2

A comprehensive ROS2 driver for Basicmicro (formerly RoboClaw) motor controllers
using the ros2_control framework. This driver provides:

- ros2_control hardware interface integration
- Multiple motion control strategies (duty, speed, distance, position)
- Advanced servo positioning and homing capabilities
- Comprehensive diagnostic and monitoring features
- High-performance optimizations with Cython extensions
- Hardware-free testing with mock interfaces

Main Components:
- hardware_interface: ros2_control integration
- motion_services: Advanced motion control services
- diagnostics: Monitoring and error reporting
- utils: Utility functions and unit conversions

Author: Your Name
License: Apache-2.0
Version: 1.0.0
"""

__version__ = "1.0.0"
__author__ = "Your Name"
__email__ = "your.email@example.com"
__license__ = "Apache-2.0"

# Import main public interfaces when they're created
# from .hardware_interface import BasicmicroHardwareInterface
# from .unit_converter import UnitConverter
# from .motion_services import MotionConfigService

__all__ = [
    "__version__",
    "__author__", 
    "__email__",
    "__license__",
    # "BasicmicroHardwareInterface",
    # "UnitConverter", 
    # "MotionConfigService",
]