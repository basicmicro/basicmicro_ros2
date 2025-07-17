# Navigation Stack Integration Guide

This guide demonstrates how to integrate the Basicmicro ROS2 driver with the ROS2 navigation stack (nav2) to create autonomous navigation capabilities for differential drive rovers.

## Prerequisites

Before following this guide, ensure you have:

- **Working rover example**: Successfully completed the [rover example](rover_example.md)
- **Hardware validation**: Confirmed motor control and odometry from [hardware validation](hardware_validation.md)
- **ROS2 environment**: nav2 packages installed (`sudo apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup`)
- **Sensor hardware**: LIDAR, camera, or other perception sensors
- **Workspace configured**: Basicmicro driver built and sourced

## Overview

The navigation stack integration enables:
- **Autonomous navigation** with obstacle avoidance
- **SLAM (Simultaneous Localization and Mapping)** capabilities
- **Path planning** with multiple algorithms
- **Localization** in known and unknown environments
- **Recovery behaviors** for navigation failures

## Navigation Stack Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   nav2_planner  │    │  nav2_controller │    │ basicmicro_node │
│                 │───▶│                  │───▶│                 │
│   (global path) │    │   (local path)   │    │  (motor control)│
└─────────────────┘    └──────────────────┘    └─────────────────┘
        ▲                        ▲                        │
        │                        │                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   map_server    │    │   costmap_2d     │    │    hardware     │
│                 │    │                  │    │                 │
│    (map data)   │    │ (obstacle map)   │    │ (encoders/IMU)  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        ▲                        ▲                        │
        │                        │                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   slam_toolbox  │    │   sensor data    │    │    odometry     │
│                 │    │                  │    │                 │
│ (mapping/SLAM)  │    │ (LIDAR/camera)   │    │   /odom topic   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Step 1: Rover Configuration for Navigation

### 1.1 Update Rover Parameters

Create navigation-specific parameters in `config/rover_nav_params.yaml`:

```yaml
basicmicro_hardware:
  ros__parameters:
    # Standard rover parameters
    port: "/dev/ttyACM0"
    baud: 38400
    address: 128
    
    # Differential drive parameters
    wheel_separation: 0.33  # meters between wheels
    wheel_radius: 0.065     # wheel radius in meters
    
    # Navigation-specific tuning
    max_velocity: 1.0       # m/s maximum velocity
    max_acceleration: 2.0   # m/s² maximum acceleration
    
    # Odometry configuration
    publish_rate: 50.0      # Hz - odometry publish rate
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    
    # Safety limits for navigation
    emergency_stop_enabled: true
    max_cmd_vel_age: 1.0    # seconds - command timeout
```

### 1.2 Robot Description for Navigation

Update your URDF to include navigation-specific frames and sensor mounts:

```xml
<?xml version="1.0"?>
<robot name="basicmicro_rover" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base link - navigation reference frame -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.083" iyy="0.147" izz="0.230" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  
  <!-- Footprint link for navigation planning -->
  <link name="base_footprint"/>
  
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>
  
  <!-- LIDAR mount (adjust position for your sensor) -->
  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>
  
  <!-- Wheel links for visualization -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.065" length="0.03"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>
  
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.065" length="0.03"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.165 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.165 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
</robot>
```

## Step 2: Navigation Launch Configuration

### 2.1 Main Navigation Launch File

Create `launch/rover_navigation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('basicmicro_driver'), 
                                   'maps', 'map.yaml'),
        description='Full path to map yaml file to load')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('basicmicro_driver'),
                                   'config', 'nav2_params.yaml'),
        description='Full path to param file to load')
    
    # Launch the rover hardware interface
    rover_node = Node(
        package='basicmicro_driver',
        executable='basicmicro_node.py',
        name='basicmicro_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('basicmicro_driver'),
            'config', 'rover_nav_params.yaml'
        ])],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom')
        ]
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(os.path.join(
                get_package_share_directory('basicmicro_driver'),
                'urdf', 'rover.urdf')).read()
        }]
    )
    
    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        rover_node,
        robot_state_publisher_node,
        nav2_bringup
    ])
```

### 2.2 SLAM Launch File

Create `launch/rover_slam.launch.py` for mapping:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('basicmicro_driver'),
            'config', 'mapper_params_online_async.yaml'
        ]),
        description='Full path to the ROS2 parameters file for SLAM')
    
    # Launch rover hardware
    rover_node = Node(
        package='basicmicro_driver',
        executable='basicmicro_node.py',
        name='basicmicro_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('basicmicro_driver'),
            'config', 'rover_nav_params.yaml'
        ])]
    )
    
    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        rover_node,
        slam_node
    ])
```

## Step 3: Navigation Parameters

### 3.1 Create nav2_params.yaml

Create `config/nav2_params.yaml` with rover-specific tuning:

```yaml
amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_to_pose_bt_xml: navigate_to_pose_w_replanning_and_recovery.xml
    default_nav_through_poses_bt_xml: navigate_through_poses_w_replanning_and_recovery.xml

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    
    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    
    # Goal checker parameters
    general_goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    
    # DWB controller parameters for differential drive
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: true
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.8
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.8
      min_speed_theta: 0.0
      acc_lim_x: 2.0
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.0
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: true
      stateful: true
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: true

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: "map.yaml"

map_saver:
  ros__parameters:
    use_sim_time: false
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: false
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

robot_state_publisher:
  ros__parameters:
    use_sim_time: false

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.8, 0.0, 1.0]
    min_velocity: [-0.8, 0.0, -1.0]
    max_accel: [2.0, 0.0, 3.2]
    max_decel: [-2.0, 0.0, -3.2]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

### 3.2 SLAM Configuration

Create `config/mapper_params_online_async.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping
    
    # If you want to start from a serialized map
    # map_file_name: /path/to/serialized_map
    # map_start_at_dock: true

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 20.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    # General Parameters
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1  
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true 
    loop_match_minimum_chain_size: 10           
    loop_match_maximum_variance_coarse: 3.0  
    loop_match_minimum_response_coarse: 0.35    
    loop_match_minimum_response_fine: 0.45

    # Correlation Parameters - Correlation Parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1 

    # Correlation Parameters - Loop Closure Parameters
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan Matcher Parameters
    distance_variance_penalty: 0.5      
    angle_variance_penalty: 1.0    

    fine_search_angle_offset: 0.00349     
    coarse_search_angle_offset: 0.349   
    coarse_angle_resolution: 0.0349        
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

## Step 4: Basic Usage Walkthrough

### 4.1 SLAM and Mapping

1. **Launch SLAM to create a map**:
   ```bash
   # Terminal 1: Launch rover with SLAM
   cd /path/to/your/ros2_ws
   source install/setup.bash
   ros2 launch basicmicro_driver rover_slam.launch.py
   
   # Terminal 2: Launch RVIZ for visualization
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/rviz/mapper.rviz
   
   # Terminal 3: Teleoperate rover to build map
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
   ```

2. **Drive the rover around** your environment to build a complete map

3. **Save the map**:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/basicmicro_driver/maps/my_map
   ```

### 4.2 Autonomous Navigation

1. **Launch navigation with the saved map**:
   ```bash
   # Update map path in rover_navigation.launch.py or pass as argument
   ros2 launch basicmicro_driver rover_navigation.launch.py map:=/path/to/your/map.yaml
   ```

2. **Launch RVIZ for navigation**:
   ```bash
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```

3. **Set initial pose** in RVIZ using "2D Pose Estimate" tool

4. **Set navigation goal** using "2D Nav Goal" tool

## Step 5: Advanced Configuration

### 5.1 Sensor Integration

**LIDAR Integration** (RP LIDAR A1):
```bash
# Install LIDAR driver
sudo apt install ros-${ROS_DISTRO}-rplidar-ros

# Launch LIDAR
ros2 launch rplidar_ros rplidar_a1_launch.py
```

**Camera Integration** (Intel RealSense):
```bash
# Install RealSense driver
sudo apt install ros-${ROS_DISTRO}-realsense2-camera

# Launch camera with depth
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true
```

### 5.2 Multi-Robot Navigation

For multiple rovers, use namespaces:

```python
# In launch file
rovers = ['robot1', 'robot2', 'robot3']

launch_descriptions = []
for robot in rovers:
    robot_group = GroupAction([
        PushRosNamespace(robot),
        Node(
            package='basicmicro_driver',
            executable='basicmicro_node.py',
            parameters=[{
                'port': f'/dev/ttyACM{rovers.index(robot)}',
                'robot_name': robot
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/navigation_launch.py']),
            launch_arguments={
                'namespace': robot,
                'use_namespace': 'true'
            }.items()
        )
    ])
    launch_descriptions.append(robot_group)
```

### 5.3 Performance Tuning

**For High-Speed Navigation**:
```yaml
# Increase control frequency
controller_server:
  ros__parameters:
    controller_frequency: 50.0  # From 20.0

# Reduce planning time
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0  # From 5.0
```

**For Precision Navigation**:
```yaml
# Tighter goal tolerances
general_goal_checker:
  xy_goal_tolerance: 0.1  # From 0.25
  yaw_goal_tolerance: 0.1  # From 0.25

# Smaller robot radius for tight spaces
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.18  # From 0.22
```

## Step 6: Troubleshooting

### 6.1 Common Issues

**Rover not moving during navigation**:
- Check cmd_vel topic: `ros2 topic echo /cmd_vel`
- Verify odometry: `ros2 topic echo /odom`
- Check motor controller connection
- Ensure wheel parameters match physical rover

**Poor localization**:
- Verify map quality (complete, no gaps)
- Check LIDAR data: `ros2 topic echo /scan`
- Tune AMCL parameters (particle count, noise models)
- Ensure proper initial pose estimate

**Navigation failures**:
- Check costmap visualization in RVIZ
- Verify robot footprint matches physical dimensions
- Tune recovery behaviors
- Check for transform tree issues: `ros2 run tf2_tools view_frames`

### 6.2 Diagnostic Commands

```bash
# Check navigation stack health
ros2 topic list | grep nav
ros2 service list | grep nav

# Monitor performance
ros2 topic hz /cmd_vel
ros2 topic hz /odom
ros2 topic hz /scan

# Debug transforms
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames

# Check navigation server status
ros2 service call /navigate_to_pose nav2_msgs/srv/NavigateToPose "{pose: {header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

## Expected Results

When properly configured, your rover should:
- **Create accurate maps** during SLAM with proper loop closure
- **Localize precisely** in known environments
- **Navigate autonomously** with obstacle avoidance
- **Recover from failures** using behavior server
- **Reach goals within tolerance** (default: 25cm position, 0.25 rad orientation)

## Performance Characteristics

- **Navigation accuracy**: ±10cm in typical indoor environments
- **Planning frequency**: 20Hz global, 20Hz local planning
- **Maximum velocity**: 0.8 m/s (configurable based on rover capabilities)
- **Obstacle detection range**: 2.5m (LIDAR dependent)
- **Map resolution**: 5cm/pixel (adjustable for performance vs accuracy)

## Next Steps

With navigation working, consider:
- **Advanced behaviors**: Custom recovery behaviors, complex navigation tasks
- **Fleet management**: Multi-robot coordination and task allocation
- **Sensor fusion**: IMU integration for improved odometry
- **Dynamic obstacles**: People detection and tracking
- **Mission planning**: Waypoint following and task execution

This navigation integration transforms your Basicmicro rover into a fully autonomous mobile robot capable of complex navigation tasks in real-world environments.