cmake_minimum_required(VERSION 3.8)
project(basicmicro_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Generate message and service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/PositionPoint.msg"
  "srv/SetMotionStrategy.srv"
  "srv/SetMotionParameters.srv"
  "srv/MoveDistance.srv"
  "srv/SetPositionLimits.srv"
  "srv/GetPositionLimits.srv"
  "srv/ExecuteTrajectory.srv"
  "srv/ExecutePositionSequence.srv"
  "srv/GetAvailableHomingMethods.srv"
  "srv/GetServoStatus.srv"
  "srv/MoveToAbsolutePosition.srv"
  "srv/PerformHoming.srv"
  "srv/ReleasePositionHold.srv"
  "srv/SetDutyCycle.srv"
  "srv/SetDutyCycleAccel.srv"
  "srv/SetHomingConfiguration.srv"
)

# Install Python modules manually
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS
  basicmicro_driver/basicmicro_node.py
  basicmicro_driver/motion_config_service.py
  basicmicro_driver/distance_movement_service.py
  basicmicro_driver/trajectory_service.py
  basicmicro_driver/duty_control_service.py
  basicmicro_driver/servo_position_service.py
  basicmicro_driver/trajectory_monitor.py
  basicmicro_driver/performance_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  msg
  srv
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()