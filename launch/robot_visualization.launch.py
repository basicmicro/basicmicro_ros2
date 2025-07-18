#!/usr/bin/env python3

"""
Robot Visualization Launch File for Basicmicro Driver
====================================================

This launch file provides comprehensive robot visualization capabilities
for the Basicmicro driver, including robot_state_publisher integration
and RViz visualization.

Usage:
    ros2 launch basicmicro_driver robot_visualization.launch.py

Key Features:
- Configurable robot description loading
- robot_state_publisher integration
- RViz visualization with custom configuration
- Joint state publisher for manual control
- TF tree broadcasting
- Support for different robot configurations

Launch Arguments:
- robot_description_file: Path to robot URDF file
- robot_config: Predefined robot configuration
- use_joint_state_publisher: Enable manual joint control
- use_rviz: Start RViz visualization
- rviz_config: Path to RViz configuration file
- use_sim_time: Use simulation time
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for robot visualization."""
    
    # Package directories
    pkg_basicmicro_ros2 = get_package_share_directory('basicmicro_ros2')
    
    # Default paths
    default_urdf_path = os.path.join(pkg_basicmicro_ros2, 'urdf', 'differential_drive_robot.urdf.xacro')
    default_rviz_path = os.path.join(pkg_basicmicro_ros2, 'rviz', 'basicmicro_robot.rviz')
    
    # Launch arguments
    robot_description_file_arg = DeclareLaunchArgument(
        'robot_description_file',
        default_value=default_urdf_path,
        description='Path to robot description file (URDF or xacro)'
    )
    
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value='diff_drive',
        description='Robot configuration type',
        choices=['diff_drive', 'industrial', 'multi_controller', 'custom']
    )
    
    use_joint_state_publisher_arg = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='true',
        description='Start joint state publisher for manual control'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz visualization'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_path,
        description='Path to RViz configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Hardware connection parameters
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for hardware connection'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='38400',
        description='Baud rate for serial communication'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='128',
        description='Controller address'
    )
    
    # Robot physical parameters
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Wheel radius in meters'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.3',
        description='Distance between wheels in meters'
    )
    
    encoder_counts_per_rev_arg = DeclareLaunchArgument(
        'encoder_counts_per_rev',
        default_value='1000',
        description='Encoder counts per revolution'
    )
    
    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='1.0',
        description='Gear ratio'
    )
    
    def get_robot_description(context):
        """Generate robot description based on configuration."""
        robot_config = LaunchConfiguration('robot_config').perform(context)
        
        # Select appropriate URDF file based on configuration
        if robot_config == 'diff_drive':
            urdf_file = os.path.join(pkg_basicmicro_ros2, 'urdf', 'differential_drive_robot.urdf.xacro')
        elif robot_config == 'industrial':
            urdf_file = os.path.join(pkg_basicmicro_ros2, 'urdf', 'industrial_robot.urdf.xacro')
        elif robot_config == 'multi_controller':
            urdf_file = os.path.join(pkg_basicmicro_ros2, 'urdf', 'multi_controller_robot.urdf.xacro')
        elif robot_config == 'custom':
            urdf_file = os.path.join(pkg_basicmicro_ros2, 'urdf', 'custom_robot_template.urdf.xacro')
        else:
            # Use explicitly provided file
            urdf_file = LaunchConfiguration('robot_description_file').perform(context)
        
        # Generate robot description with parameters
        robot_description = Command([
            'xacro ', urdf_file,
            ' port:=', LaunchConfiguration('port'),
            ' baud:=', LaunchConfiguration('baud'),
            ' address:=', LaunchConfiguration('address'),
            ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
            ' wheel_separation:=', LaunchConfiguration('wheel_separation'),
            ' encoder_counts_per_rev:=', LaunchConfiguration('encoder_counts_per_rev'),
            ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
        ])
        
        return robot_description
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': OpaqueFunction(function=get_robot_description),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    # Joint state publisher (for manual control)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    # Joint state publisher GUI (for interactive control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    # RViz visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    # TF static transformations (if needed)
    tf_base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    # Map to odom transformation (for navigation)
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    return LaunchDescription([
        # Launch arguments
        robot_description_file_arg,
        robot_config_arg,
        use_joint_state_publisher_arg,
        use_rviz_arg,
        rviz_config_arg,
        use_sim_time_arg,
        port_arg,
        baud_arg,
        address_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        encoder_counts_per_rev_arg,
        gear_ratio_arg,
        
        # Nodes
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz,
        tf_base_link_to_base_footprint,
        tf_map_to_odom,
    ])