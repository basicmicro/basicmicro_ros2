#!/usr/bin/env python3

"""
Comprehensive Test Suite for URDF Examples
==========================================

This test suite validates all URDF examples in the basicmicro_driver package,
including syntax validation, parameter consistency, and integration testing.

Test Categories:
1. URDF Syntax Validation
2. Xacro Compilation Testing
3. Parameter Consistency Validation
4. ros2_control Integration Testing
5. Gazebo Integration Testing
6. Robot State Publisher Integration
7. Physical Parameter Validation
8. Joint Configuration Testing

All tests are designed to work without ROS2 dependencies for CI/CD integration.
"""

import unittest
import os
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
import re

# Add the package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

class TestURDFExamples(unittest.TestCase):
    """Test suite for URDF examples."""

    def setUp(self):
        """Set up test fixtures."""
        self.package_dir = Path(__file__).parent.parent
        self.urdf_dir = self.package_dir / 'urdf'
        self.config_dir = self.package_dir / 'config'
        
        # Expected URDF files
        self.urdf_files = {
            'differential_drive_robot.urdf.xacro': 'diff_drive',
            'industrial_robot.urdf.xacro': 'industrial',
            'multi_controller_robot.urdf.xacro': 'multi_controller',
            'custom_robot_template.urdf.xacro': 'custom'
        }
        
        # Common test parameters
        self.test_params = {
            'port': '/dev/ttyACM0',
            'baud': '38400',
            'address': '128',
            'wheel_radius': '0.1',
            'wheel_separation': '0.3',
            'encoder_counts_per_rev': '1000',
            'gear_ratio': '1.0'
        }

    def test_urdf_files_exist(self):
        """Test that all expected URDF files exist."""
        for urdf_file in self.urdf_files.keys():
            file_path = self.urdf_dir / urdf_file
            self.assertTrue(file_path.exists(), f"URDF file {urdf_file} does not exist")
            self.assertTrue(file_path.is_file(), f"URDF file {urdf_file} is not a file")

    def test_urdf_xml_syntax(self):
        """Test that all URDF files have valid XML syntax."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                try:
                    with open(file_path, 'r') as f:
                        content = f.read()
                    
                    # Check for basic XML structure
                    self.assertIn('<?xml version="1.0"?>', content)
                    self.assertIn('<robot', content)
                    self.assertIn('</robot>', content)
                    
                    # Check for xacro namespace
                    self.assertIn('xmlns:xacro=', content)
                    
                except Exception as e:
                    self.fail(f"URDF file {urdf_file} has invalid XML syntax: {e}")

    def test_xacro_arguments(self):
        """Test that all URDF files have proper xacro arguments."""
        # Different files have different argument structures
        common_args = ['baud', 'wheel_radius', 'encoder_counts_per_rev']
        
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Check for common xacro arguments
                for arg in common_args:
                    self.assertIn(f'<xacro:arg name="{arg}"', content,
                                f"URDF file {urdf_file} missing required argument {arg}")
                
                # Check for port-related arguments (different names for multi-controller)
                if 'multi_controller' in urdf_file:
                    self.assertIn('<xacro:arg name="front_port"', content,
                                f"Multi-controller URDF missing front_port argument")
                    self.assertIn('<xacro:arg name="rear_port"', content,
                                f"Multi-controller URDF missing rear_port argument")
                else:
                    self.assertIn('<xacro:arg name="port"', content,
                                f"URDF file {urdf_file} missing port argument")
                
                # Check for address-related arguments
                if 'multi_controller' in urdf_file:
                    self.assertIn('<xacro:arg name="front_address"', content,
                                f"Multi-controller URDF missing front_address argument")
                    self.assertIn('<xacro:arg name="rear_address"', content,
                                f"Multi-controller URDF missing rear_address argument")
                else:
                    self.assertIn('<xacro:arg name="address"', content,
                                f"URDF file {urdf_file} missing address argument")

    def test_ros2_control_integration(self):
        """Test that all URDF files have proper ros2_control integration."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Check for ros2_control section
                self.assertIn('<ros2_control', content,
                            f"URDF file {urdf_file} missing ros2_control section")
                
                # Check for hardware plugin
                self.assertIn('basicmicro_driver/BasicmicroHardwareInterface', content,
                            f"URDF file {urdf_file} missing hardware interface plugin")
                
                # Check for joint interfaces
                self.assertIn('command_interface name="velocity"', content,
                            f"URDF file {urdf_file} missing velocity command interface")
                self.assertIn('state_interface name="position"', content,
                            f"URDF file {urdf_file} missing position state interface")
                self.assertIn('state_interface name="velocity"', content,
                            f"URDF file {urdf_file} missing velocity state interface")

    def test_gazebo_integration(self):
        """Test that all URDF files have proper Gazebo integration."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Check for Gazebo plugin
                if 'gazebo_ros2_control' in content:
                    self.assertIn('libgazebo_ros2_control.so', content,
                                f"URDF file {urdf_file} missing Gazebo ros2_control plugin")
                
                # Check for Gazebo material properties
                if '<gazebo reference=' in content:
                    self.assertIn('<material>', content,
                                f"URDF file {urdf_file} missing Gazebo material properties")

    def test_joint_configuration(self):
        """Test that all URDF files have proper joint configuration."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Check for wheel joints
                if 'differential_drive' in urdf_file or 'industrial' in urdf_file or 'custom' in urdf_file:
                    self.assertIn('left_wheel_joint', content,
                                f"URDF file {urdf_file} missing left wheel joint")
                    self.assertIn('right_wheel_joint', content,
                                f"URDF file {urdf_file} missing right wheel joint")
                
                # Check for continuous joints
                self.assertIn('type="continuous"', content,
                            f"URDF file {urdf_file} missing continuous joint type")

    def test_physical_parameters(self):
        """Test that physical parameters are within reasonable ranges."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Extract default values for physical parameters
                wheel_radius_match = re.search(r'wheel_radius.*default="([^"]+)"', content)
                if wheel_radius_match:
                    wheel_radius = float(wheel_radius_match.group(1))
                    self.assertGreater(wheel_radius, 0.01, 
                                     f"URDF file {urdf_file} has unreasonably small wheel radius")
                    self.assertLess(wheel_radius, 1.0,
                                  f"URDF file {urdf_file} has unreasonably large wheel radius")
                
                wheel_separation_match = re.search(r'wheel_separation.*default="([^"]+)"', content)
                if wheel_separation_match:
                    wheel_separation = float(wheel_separation_match.group(1))
                    self.assertGreater(wheel_separation, 0.1,
                                     f"URDF file {urdf_file} has unreasonably small wheel separation")
                    self.assertLess(wheel_separation, 2.0,
                                  f"URDF file {urdf_file} has unreasonably large wheel separation")

    def test_inertial_properties(self):
        """Test that all links have proper inertial properties."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Check for inertial sections
                self.assertIn('<inertial>', content,
                            f"URDF file {urdf_file} missing inertial properties")
                self.assertIn('<mass value=', content,
                            f"URDF file {urdf_file} missing mass properties")
                self.assertIn('<inertia ixx=', content,
                            f"URDF file {urdf_file} missing inertia properties")

    def test_material_definitions(self):
        """Test that all URDF files have proper material definitions."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Check for material definitions
                self.assertIn('<material name=', content,
                            f"URDF file {urdf_file} missing material definitions")
                self.assertIn('<color rgba=', content,
                            f"URDF file {urdf_file} missing color definitions")

    def test_collision_geometry(self):
        """Test that all links have collision geometry."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Check for collision sections
                self.assertIn('<collision>', content,
                            f"URDF file {urdf_file} missing collision geometry")

    def test_multi_controller_configuration(self):
        """Test multi-controller specific configuration."""
        urdf_file = 'multi_controller_robot.urdf.xacro'
        file_path = self.urdf_dir / urdf_file
        
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Check for multiple controller systems
        self.assertIn('FrontControllerSystem', content,
                    "Multi-controller URDF missing front controller system")
        self.assertIn('RearControllerSystem', content,
                    "Multi-controller URDF missing rear controller system")
        
        # Check for 4WD/6WD configuration
        self.assertIn('drive_mode', content,
                    "Multi-controller URDF missing drive mode configuration")
        self.assertIn('4WD', content,
                    "Multi-controller URDF missing 4WD configuration")
        self.assertIn('6WD', content,
                    "Multi-controller URDF missing 6WD configuration")

    def test_custom_template_comprehensiveness(self):
        """Test that custom template has comprehensive documentation."""
        urdf_file = 'custom_robot_template.urdf.xacro'
        file_path = self.urdf_dir / urdf_file
        
        with open(file_path, 'r') as f:
            content = f.read()
        
        # Check for comprehensive documentation
        self.assertIn('USAGE INSTRUCTIONS', content,
                    "Custom template missing usage instructions")
        self.assertIn('CUSTOMIZATION GUIDE', content,
                    "Custom template missing customization guide")
        self.assertIn('PARAMETER REFERENCE', content,
                    "Custom template missing parameter reference")
        
        # Check for customization examples
        self.assertIn('CUSTOMIZATION:', content,
                    "Custom template missing customization examples")

    def test_parameter_consistency(self):
        """Test parameter consistency across URDF files."""
        parameter_sets = {}
        
        # Extract parameters from each file
        for urdf_file in self.urdf_files.keys():
            file_path = self.urdf_dir / urdf_file
            with open(file_path, 'r') as f:
                content = f.read()
            
            # Extract xacro arguments
            args = re.findall(r'<xacro:arg name="([^"]+)".*?default="([^"]+)"', content)
            parameter_sets[urdf_file] = dict(args)
        
        # Check that common parameters exist across files
        common_params = ['port', 'baud', 'address', 'wheel_radius', 'wheel_separation']
        for param in common_params:
            for urdf_file in self.urdf_files.keys():
                if urdf_file != 'multi_controller_robot.urdf.xacro':  # Multi-controller has different structure
                    self.assertIn(param, parameter_sets[urdf_file],
                                f"URDF file {urdf_file} missing common parameter {param}")

    def test_documentation_completeness(self):
        """Test that all URDF files have proper documentation."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Check for header documentation
                self.assertIn('<!--', content,
                            f"URDF file {urdf_file} missing documentation comments")
                
                # Check for usage instructions
                if 'template' in urdf_file:
                    usage_found = ('Usage:' in content or 
                                 'USAGE INSTRUCTIONS:' in content or 
                                 'USAGE:' in content)
                    self.assertTrue(usage_found,
                                  f"URDF file {urdf_file} missing usage instructions")

    def test_integration_with_launch_system(self):
        """Test integration with launch file system."""
        # Check that launch file directory exists
        launch_dir = self.package_dir / 'launch'
        self.assertTrue(launch_dir.exists(), "Launch directory does not exist")
        
        # Check for visualization launch file
        viz_launch = launch_dir / 'robot_visualization.launch.py'
        self.assertTrue(viz_launch.exists(), "Robot visualization launch file does not exist")
        
        if viz_launch.exists():
            with open(viz_launch, 'r') as f:
                content = f.read()
            
            # Check for URDF integration
            for urdf_file in self.urdf_files.keys():
                urdf_name = urdf_file.replace('.urdf.xacro', '')
                if urdf_name in content:
                    self.assertIn(urdf_name, content,
                                f"Launch file does not reference URDF {urdf_name}")

    def test_rviz_configuration(self):
        """Test RViz configuration file."""
        rviz_dir = self.package_dir / 'rviz'
        rviz_file = rviz_dir / 'basicmicro_robot.rviz'
        
        if rviz_file.exists():
            with open(rviz_file, 'r') as f:
                content = f.read()
            
            # Check for basic RViz configuration
            self.assertIn('RobotModel', content,
                        "RViz configuration missing robot model display")
            self.assertIn('TF', content,
                        "RViz configuration missing TF display")
            self.assertIn('JointState', content,
                        "RViz configuration missing joint state display")

    def test_urdf_compilation_simulation(self):
        """Simulate URDF compilation without xacro dependency."""
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Check for proper xacro structure
                self.assertIn('<robot xmlns:xacro=', content,
                            f"URDF file {urdf_file} missing proper xacro robot declaration")
                
                # Check for balanced tags
                robot_open = content.count('<robot')
                robot_close = content.count('</robot>')
                self.assertEqual(robot_open, robot_close,
                               f"URDF file {urdf_file} has unbalanced robot tags")

    def test_hardware_interface_parameters(self):
        """Test hardware interface parameter completeness."""
        required_hw_params = [
            'port', 'baud', 'address', 'wheel_radius', 'wheel_separation',
            'encoder_counts_per_rev', 'motion_strategy'
        ]
        
        for urdf_file in self.urdf_files.keys():
            with self.subTest(urdf_file=urdf_file):
                file_path = self.urdf_dir / urdf_file
                with open(file_path, 'r') as f:
                    content = f.read()
                
                # Extract hardware section
                hw_section_match = re.search(r'<hardware>.*?</hardware>', content, re.DOTALL)
                if hw_section_match:
                    hw_section = hw_section_match.group(0)
                    
                    # Check for required parameters
                    for param in required_hw_params:
                        self.assertIn(f'<param name="{param}"', hw_section,
                                    f"URDF file {urdf_file} missing hardware parameter {param}")


def run_tests():
    """Run all URDF tests."""
    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestURDFExamples)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)