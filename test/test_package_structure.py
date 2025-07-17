"""
Package Structure Validation Tests

Tests to verify that the ROS2 package structure is correctly set up and all
required components are present and functional.

Author: Your Name
License: Apache-2.0
"""

import pytest
import os
import sys
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET


class TestPackageStructure:
    """Test package structure and basic functionality"""
    
    @classmethod
    def setup_class(cls):
        """Set up test class"""
        # Get package root directory
        test_dir = Path(__file__).parent
        cls.package_root = test_dir.parent
        cls.package_name = "basicmicro_driver"
        
    def test_package_root_exists(self):
        """Test that package root directory exists"""
        assert self.package_root.exists(), f"Package root {self.package_root} does not exist"
        assert self.package_root.is_dir(), f"Package root {self.package_root} is not a directory"
        
    def test_required_directories_exist(self):
        """Test that all required directories exist"""
        required_dirs = [
            "basicmicro_driver",  # Main package directory
            "launch",
            "config", 
            "test",
            "test_mocks",
            "benchmarks",
            "resource"
        ]
        
        for dir_name in required_dirs:
            dir_path = self.package_root / dir_name
            assert dir_path.exists(), f"Required directory {dir_name} does not exist"
            assert dir_path.is_dir(), f"Required path {dir_name} is not a directory"
            
    def test_package_xml_exists_and_valid(self):
        """Test that package.xml exists and is valid"""
        package_xml = self.package_root / "package.xml"
        assert package_xml.exists(), "package.xml does not exist"
        
        # Test XML is valid
        try:
            tree = ET.parse(package_xml)
            root = tree.getroot()
        except ET.ParseError as e:
            pytest.fail(f"package.xml is not valid XML: {e}")
            
        # Test required elements exist
        assert root.tag == "package", "Root element must be 'package'"
        
        name_elem = root.find("name")
        assert name_elem is not None, "package.xml must contain <name> element"
        assert name_elem.text == self.package_name, f"Package name should be {self.package_name}"
        
        version_elem = root.find("version")
        assert version_elem is not None, "package.xml must contain <version> element"
        
        description_elem = root.find("description")
        assert description_elem is not None, "package.xml must contain <description> element"
        
        maintainer_elem = root.find("maintainer")
        assert maintainer_elem is not None, "package.xml must contain <maintainer> element"
        
        license_elem = root.find("license")
        assert license_elem is not None, "package.xml must contain <license> element"
        
    def test_setup_py_exists_and_valid(self):
        """Test that setup.py exists and is valid Python"""
        setup_py = self.package_root / "setup.py"
        assert setup_py.exists(), "setup.py does not exist"
        
        # Test file is valid Python
        spec = importlib.util.spec_from_file_location("setup", setup_py)
        assert spec is not None, "setup.py is not a valid Python module"
        
        # Read content and verify it has required components
        content = setup_py.read_text()
        assert "from setuptools import setup" in content, "setup.py must import setuptools"
        # Check for package name either as literal or variable
        has_name = (f"name={self.package_name}" in content or 
                   f"name='{self.package_name}'" in content or 
                   f'name="{self.package_name}"' in content or
                   ("name=package_name" in content and f"package_name = '{self.package_name}'" in content))
        assert has_name, f"setup.py must specify package name as {self.package_name}"
        assert "entry_points" in content, "setup.py must define entry_points"
        
    def test_setup_cfg_exists(self):
        """Test that setup.cfg exists"""
        setup_cfg = self.package_root / "setup.cfg"
        assert setup_cfg.exists(), "setup.cfg does not exist"
        
        # Read and verify basic structure
        content = setup_cfg.read_text()
        assert "[develop]" in content, "setup.cfg must contain [develop] section"
        assert "[install]" in content, "setup.cfg must contain [install] section"
        
    def test_package_init_exists_and_importable(self):
        """Test that package __init__.py exists and is importable"""
        package_dir = self.package_root / self.package_name
        init_py = package_dir / "__init__.py"
        
        assert init_py.exists(), f"{self.package_name}/__init__.py does not exist"
        
        # Test file is valid Python
        spec = importlib.util.spec_from_file_location("basicmicro_driver", init_py)
        assert spec is not None, "__init__.py is not a valid Python module"
        
        # Verify it contains version info
        content = init_py.read_text()
        assert "__version__" in content, "__init__.py must define __version__"
        
    def test_resource_file_exists(self):
        """Test that the resource file exists"""
        resource_file = self.package_root / "resource" / self.package_name
        assert resource_file.exists(), f"Resource file resource/{self.package_name} does not exist"
        
    def test_mock_basicmicro_exists_and_importable(self):
        """Test that mock_basicmicro.py exists and is importable"""
        mock_file = self.package_root / "test_mocks" / "mock_basicmicro.py"
        assert mock_file.exists(), "test_mocks/mock_basicmicro.py does not exist"
        
        # Test file is valid Python
        spec = importlib.util.spec_from_file_location("mock_basicmicro", mock_file)
        assert spec is not None, "mock_basicmicro.py is not a valid Python module"
        
        # Verify it contains MockBasicmicro class
        content = mock_file.read_text()
        assert "class MockBasicmicro" in content, "mock_basicmicro.py must define MockBasicmicro class"
        
    def test_package_dependencies(self):
        """Test that required dependencies are specified in package.xml"""
        package_xml = self.package_root / "package.xml"
        tree = ET.parse(package_xml)
        root = tree.getroot()
        
        # Get all dependency elements
        dependencies = []
        for dep_type in ["depend", "build_depend", "exec_depend", "test_depend"]:
            deps = root.findall(dep_type)
            dependencies.extend([dep.text for dep in deps])
            
        # Check for required ROS2 dependencies
        required_deps = [
            "rclpy",
            "std_msgs",
            "geometry_msgs",
            "sensor_msgs", 
            "nav_msgs",
            "diagnostic_msgs",
            "controller_interface",
            "hardware_interface",
            "pluginlib"
        ]
        
        for dep in required_deps:
            assert dep in dependencies, f"Required dependency {dep} not found in package.xml"
            
    def test_python_requirements_consistency(self):
        """Test that Python requirements are consistent between setup.py and package.xml"""
        # This is a basic check - in a real implementation, you'd want to parse both files
        # and verify the dependencies match
        setup_py = self.package_root / "setup.py"
        package_xml = self.package_root / "package.xml"
        
        setup_content = setup_py.read_text()
        xml_content = package_xml.read_text()
        
        # Basic checks for common dependencies
        common_deps = ["rclpy", "numpy"]
        for dep in common_deps:
            assert dep in setup_content, f"Dependency {dep} not found in setup.py"
            assert dep in xml_content or "python3-" + dep in xml_content, \
                   f"Dependency {dep} not found in package.xml"


class TestMockBasicmicro:
    """Test mock Basicmicro functionality"""
    
    @classmethod
    def setup_class(cls):
        """Set up test class"""
        # Add test_mocks to path
        test_dir = Path(__file__).parent
        package_root = test_dir.parent
        mocks_dir = package_root / "test_mocks"
        sys.path.insert(0, str(mocks_dir))
        
        # Import MockBasicmicro
        from mock_basicmicro import MockBasicmicro, create_mock_controller
        cls.MockBasicmicro = MockBasicmicro
        cls.create_mock_controller = create_mock_controller
        
    def test_mock_controller_creation(self):
        """Test that mock controller can be created"""
        mock = self.MockBasicmicro("/dev/ttyMOCK", 38400)
        assert mock.port == "/dev/ttyMOCK"
        assert mock.baud == 38400
        assert not mock.connected
        
    def test_mock_controller_connection(self):
        """Test mock controller connection/disconnection"""
        mock = self.MockBasicmicro("/dev/ttyMOCK", 38400)
        
        # Test connection
        result = mock.Open()
        assert result is True
        assert mock.connected is True
        
        # Test disconnection
        result = mock.close()
        assert result is True
        assert mock.connected is False
        
    def test_mock_controller_context_manager(self):
        """Test mock controller as context manager"""
        mock = self.MockBasicmicro("/dev/ttyMOCK", 38400)
        
        with mock:
            assert mock.connected is True
            
        assert mock.connected is False
        
    def test_mock_basic_commands(self):
        """Test basic mock controller commands"""
        mock = self.MockBasicmicro("/dev/ttyMOCK", 38400)
        mock.Open()
        
        try:
            # Test duty command
            result = mock.DutyM1M2(0x80, 16384, -16384)
            assert result is True
            
            # Test speed command
            result = mock.SpeedM1M2(0x80, 1000, -1000)
            assert result is True
            
            # Test encoder reading
            success, left_enc, right_enc = mock.GetEncoders(0x80)
            assert success is True
            assert isinstance(left_enc, int)
            assert isinstance(right_enc, int)
            
            # Test speed reading
            success, left_speed, right_speed = mock.GetSpeeds(0x80)
            assert success is True
            assert isinstance(left_speed, int)
            assert isinstance(right_speed, int)
            
        finally:
            mock.close()
            
    def test_mock_error_simulation(self):
        """Test mock error simulation functionality"""
        mock = self.MockBasicmicro("/dev/ttyMOCK", 38400)
        mock.Open()
        
        try:
            # Set communication failure rate
            mock.set_communication_failure_rate(1.0)  # Always fail
            
            # Commands should now fail
            result = mock.DutyM1M2(0x80, 0, 0)
            assert result is False
            
            success, _, _ = mock.GetEncoders(0x80)
            assert success is False
            
            # Reset failure rate
            mock.set_communication_failure_rate(0.0)  # Never fail
            
            # Commands should now succeed
            result = mock.DutyM1M2(0x80, 0, 0)
            assert result is True
            
        finally:
            mock.close()
            
    def test_create_mock_controller_function(self):
        """Test convenience function for creating mock controllers"""
        from mock_basicmicro import create_mock_controller
        
        mock = create_mock_controller()
        assert mock.port == "/dev/ttyMOCK"
        assert mock.baud == 38400
        
        # Test with custom parameters
        mock2 = create_mock_controller("/dev/custom", 115200)
        assert mock2.port == "/dev/custom"
        assert mock2.baud == 115200


if __name__ == "__main__":
    pytest.main([__file__, "-v"])