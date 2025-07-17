"""
Test Mocks Package

Mock implementations for hardware-free testing of the Basicmicro driver.

This package provides:
- MockBasicmicro: Complete mock implementation of Basicmicro controller
- Additional mock utilities for testing various scenarios

Author: Your Name
License: Apache-2.0
"""

from .mock_basicmicro import MockBasicmicro, create_mock_controller, ControllerType

__all__ = [
    "MockBasicmicro",
    "create_mock_controller", 
    "ControllerType"
]