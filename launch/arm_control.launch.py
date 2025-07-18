#!/usr/bin/env python3

"""
Two-Axis Arm Control Launch File
================================

Launch file for the Basicmicro two-axis arm system with servo position control.
Includes robot description, hardware interface, controllers, and visualization.

Usage:
    ros2 launch basicmicro_driver arm_control.launch.py
    ros2 launch basicmicro_driver arm_control.launch.py port:=/dev/ttyACM1 homing:=true

Arguments:
    port: Serial port for hardware connection (default: /dev/ttyACM1)
    baud: Baud rate (default: 38400)
    address: Controller address (default: 128)
    link1_length: Length of first arm segment (default: 0.3)
    link2_length: Length of second arm segment (default: 0.25)
    homing: Perform homing on startup (default: false)
    use_sim_time: Use simulation time (default: false)
    rviz: Launch RViz visualization (default: true)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for two-axis arm control"""
    
    # Package directory
    pkg_share = FindPackageShare(package='basicmicro_ros2').find('basicmicro_ros2')
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'two_axis_arm.urdf.xacro')
    
    # Controller configuration
    controllers_file = os.path.join(pkg_share, 'config', 'arm_controllers.yaml')
    
    # RViz configuration
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'arm_visualization.rviz')
    
    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM1',
            description='Serial port for hardware connection'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='38400',
            description='Baud rate for serial communication'
        ),
        DeclareLaunchArgument(
            'address',
            default_value='128',
            description='Controller address (0x80)'
        ),
        DeclareLaunchArgument(
            'link1_length',
            default_value='0.3',
            description='Length of first arm segment in meters'
        ),
        DeclareLaunchArgument(
            'link2_length',
            default_value='0.25',
            description='Length of second arm segment in meters'
        ),
        DeclareLaunchArgument(
            'joint1_min',
            default_value='-3.14159',
            description='Joint 1 minimum position (radians)'
        ),
        DeclareLaunchArgument(
            'joint1_max',
            default_value='3.14159',
            description='Joint 1 maximum position (radians)'
        ),
        DeclareLaunchArgument(
            'joint2_min',
            default_value='-1.5708',
            description='Joint 2 minimum position (radians)'
        ),
        DeclareLaunchArgument(
            'joint2_max',
            default_value='1.5708',
            description='Joint 2 maximum position (radians)'
        ),
        DeclareLaunchArgument(
            'homing',
            default_value='false',
            description='Perform homing on startup'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz visualization'
        ),
        DeclareLaunchArgument(
            'max_velocity',
            default_value='2.0',
            description='Maximum joint velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'max_acceleration',
            default_value='5.0',
            description='Maximum joint acceleration (rad/s²)'
        ),
        DeclareLaunchArgument(
            'gear_ratio',
            default_value='50.0',
            description='Motor gear ratio'
        ),
        DeclareLaunchArgument(
            'encoder_counts_per_rev',
            default_value='2000',
            description='Encoder counts per revolution'
        )
    ]
    
    # Robot description
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' port:=', LaunchConfiguration('port'),
        ' baud:=', LaunchConfiguration('baud'),
        ' address:=', LaunchConfiguration('address'),
        ' link1_length:=', LaunchConfiguration('link1_length'),
        ' link2_length:=', LaunchConfiguration('link2_length'),
        ' joint1_min_position:=', LaunchConfiguration('joint1_min'),
        ' joint1_max_position:=', LaunchConfiguration('joint1_max'),
        ' joint2_min_position:=', LaunchConfiguration('joint2_min'),
        ' joint2_max_position:=', LaunchConfiguration('joint2_max'),
        ' max_velocity:=', LaunchConfiguration('max_velocity'),
        ' max_acceleration:=', LaunchConfiguration('max_acceleration'),
        ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
        ' encoder_counts_per_rev:=', LaunchConfiguration('encoder_counts_per_rev')
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Nodes
    nodes = [
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        
        # Arm Node (Hardware Interface)
        Node(
            package='basicmicro_ros2',
            executable='arm_node.py',
            name='arm_node',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
                {'baud': LaunchConfiguration('baud')},
                {'address': LaunchConfiguration('address')},
                {'link1_length': LaunchConfiguration('link1_length')},
                {'link2_length': LaunchConfiguration('link2_length')},
                {'joint1_limits': [LaunchConfiguration('joint1_min'), LaunchConfiguration('joint1_max')]},
                {'joint2_limits': [LaunchConfiguration('joint2_min'), LaunchConfiguration('joint2_max')]},
                {'max_velocity': LaunchConfiguration('max_velocity')},
                {'max_acceleration': LaunchConfiguration('max_acceleration')},
                {'gear_ratio': LaunchConfiguration('gear_ratio')},
                {'encoder_counts_per_rev': LaunchConfiguration('encoder_counts_per_rev')},
                {'homing_on_startup': LaunchConfiguration('homing')},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        
        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description,
                controllers_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),
        
        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        
        # Arm Position Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_position_controller'],
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        )
    ]
    
    return LaunchDescription(declared_arguments + nodes)


if __name__ == '__main__':
    generate_launch_description()