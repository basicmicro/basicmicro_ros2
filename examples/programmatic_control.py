#!/usr/bin/env python3
"""
Programmatic Motor Control Example

This script demonstrates how to control motors programmatically using ROS2
without the delays associated with --once commands.

Usage:
    python3 programmatic_control.py

Requirements:
    - Basicmicro ROS2 driver running
    - ROS2 environment sourced
    - Motor controller connected

Author: Generated for Basicmicro ROS2 Driver
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import signal
import sys

class MotorController(Node):
    """Real-time motor controller without command delays."""
    
    def __init__(self):
        super().__init__('programmatic_motor_controller')
        
        # Create publisher for motor commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to driver status
        self.status_sub = self.create_subscription(
            String, 'basicmicro/status', self.status_callback, 10)
        
        # State tracking
        self.driver_connected = True
        self.last_status = "unknown"
        
        # Wait for publisher to be ready
        self.get_logger().info("Initializing motor controller...")
        time.sleep(1.0)
        self.get_logger().info("Motor controller ready - no delays for commands")
        
    def status_callback(self, msg):
        """Monitor driver connection status."""
        self.last_status = msg.data
        if "CONNECTION_FAILED" in msg.data:
            self.driver_connected = False
            self.get_logger().error(f"Driver connection failed: {msg.data}")
        else:
            self.driver_connected = True
            
    def send_command(self, linear_x=0.0, angular_z=0.0):
        """Send motor command immediately - no delay."""
        if not self.driver_connected:
            self.get_logger().warn("Driver not connected, command ignored")
            return False
            
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        self.cmd_pub.publish(msg)
        return True
        
    def move_forward(self, speed=0.1, duration=2.0):
        """Move forward for specified duration."""
        self.get_logger().info(f"Moving forward at {speed} m/s for {duration} seconds")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if not self.send_command(linear_x=speed):
                break
            time.sleep(0.1)  # 10Hz control rate
            
        self.stop_motors()
        
    def move_backward(self, speed=0.1, duration=2.0):
        """Move backward for specified duration."""
        self.get_logger().info(f"Moving backward at {speed} m/s for {duration} seconds")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if not self.send_command(linear_x=-speed):
                break
            time.sleep(0.1)  # 10Hz control rate
            
        self.stop_motors()
        
    def turn_left(self, angular_speed=0.5, duration=1.0):
        """Turn left for specified duration."""
        self.get_logger().info(f"Turning left at {angular_speed} rad/s for {duration} seconds")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if not self.send_command(angular_z=angular_speed):
                break
            time.sleep(0.1)  # 10Hz control rate
            
        self.stop_motors()
        
    def turn_right(self, angular_speed=0.5, duration=1.0):
        """Turn right for specified duration."""
        self.get_logger().info(f"Turning right at {angular_speed} rad/s for {duration} seconds")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if not self.send_command(angular_z=-angular_speed):
                break
            time.sleep(0.1)  # 10Hz control rate
            
        self.stop_motors()
        
    def stop_motors(self):
        """Stop motors immediately."""
        self.get_logger().info("Stopping motors")
        self.send_command(0.0, 0.0)
        
    def emergency_stop(self):
        """Emergency stop - called on Ctrl+C."""
        self.get_logger().warn("EMERGENCY STOP - Stopping all motors")
        self.send_command(0.0, 0.0)
        
    def run_demo_sequence(self):
        """Run a demonstration sequence."""
        self.get_logger().info("Starting demonstration sequence...")
        
        try:
            # Demo sequence
            self.move_forward(0.1, 2.0)     # Forward 0.1 m/s for 2 seconds
            time.sleep(0.5)                 # Pause
            
            self.move_backward(0.1, 1.0)    # Backward 0.1 m/s for 1 second
            time.sleep(0.5)                 # Pause
            
            self.turn_left(0.3, 1.0)        # Turn left 0.3 rad/s for 1 second
            time.sleep(0.5)                 # Pause
            
            self.turn_right(0.3, 1.0)       # Turn right 0.3 rad/s for 1 second
            time.sleep(0.5)                 # Pause
            
            self.get_logger().info("Demonstration sequence complete")
            
        except KeyboardInterrupt:
            self.emergency_stop()
            
    def run_interactive_control(self):
        """Run interactive control mode."""
        self.get_logger().info("Interactive control mode")
        self.get_logger().info("Commands: w=forward, s=backward, a=left, d=right, space=stop, q=quit")
        
        try:
            while True:
                command = input("Enter command (w/s/a/d/space/q): ").strip().lower()
                
                if command == 'w':
                    self.move_forward(0.1, 1.0)
                elif command == 's':
                    self.move_backward(0.1, 1.0)
                elif command == 'a':
                    self.turn_left(0.3, 0.5)
                elif command == 'd':
                    self.turn_right(0.3, 0.5)
                elif command == ' ' or command == 'space':
                    self.stop_motors()
                elif command == 'q':
                    break
                else:
                    self.get_logger().warn(f"Unknown command: {command}")
                    
        except KeyboardInterrupt:
            self.emergency_stop()
            
        self.get_logger().info("Interactive control ended")

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully."""
    print("\\nShutting down motor controller...")
    sys.exit(0)

def main():
    """Main entry point."""
    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize ROS2
    rclpy.init()
    
    # Create motor controller
    controller = MotorController()
    
    try:
        # Choose control mode
        print("Basicmicro Programmatic Motor Control")
        print("=====================================")
        print("1. Run demonstration sequence")
        print("2. Interactive control mode")
        print("3. Exit")
        
        choice = input("Enter choice (1/2/3): ").strip()
        
        if choice == '1':
            controller.run_demo_sequence()
        elif choice == '2':
            controller.run_interactive_control()
        elif choice == '3':
            controller.get_logger().info("Exiting without running motors")
        else:
            controller.get_logger().error("Invalid choice")
            
    except Exception as e:
        controller.get_logger().error(f"Error: {e}")
    finally:
        # Clean shutdown
        controller.emergency_stop()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()