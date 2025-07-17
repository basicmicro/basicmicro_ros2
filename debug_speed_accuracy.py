#!/usr/bin/env python3
"""
Debug Speed Accuracy Investigation
Investigates why performance test shows poor speed accuracy when Motion Studio shows good accuracy
"""

import sys
import os
import time
import math

# Add package to Python path
sys.path.insert(0, '.')

from basicmicro_driver.hardware_interface import BasicmicroHardwareInterface, MotionStrategy
from basicmicro_driver.unit_converter import UnitConverter
from basicmicro import Basicmicro

def debug_speed_accuracy():
    """Debug speed accuracy measurement issues"""
    
    config = {
        'port': '/dev/ttyACM1',
        'baud': 38400,
        'address': 0x80,
        'wheel_radius': 0.1,
        'encoder_counts_per_rev': 1000,
        'gear_ratio': 1.0,
    }
    
    print("🔍 SPEED ACCURACY DEBUG INVESTIGATION")
    print("=" * 50)
    
    # Test direct hardware commands first
    try:
        controller = Basicmicro(config['port'], config['baud'])
        if not controller.Open():
            print("❌ Could not open controller")
            return
            
        print("✅ Connected to hardware")
        
        # Initialize unit converter
        unit_converter = UnitConverter(
            config['wheel_radius'],
            config['encoder_counts_per_rev'],
            config['gear_ratio']
        )
        
        # Test various speeds
        test_speeds_rad_s = [1.0, 2.0, 5.0, 10.0]
        
        for target_speed_rad_s in test_speeds_rad_s:
            print(f"\n🎯 Testing {target_speed_rad_s} rad/s")
            
            # Convert to counts/sec
            target_speed_counts = unit_converter.rad_per_sec_to_counts_per_sec(target_speed_rad_s)
            print(f"  Target: {target_speed_rad_s} rad/s = {target_speed_counts:.0f} counts/sec")
            
            # Stop motors first
            controller.DutyM1M2(config['address'], 0, 0)
            time.sleep(0.1)
            
            # Set speed command
            result = controller.SpeedM1M2(config['address'], int(target_speed_counts), int(target_speed_counts))
            print(f"  Command result: {result}")
            
            # Allow time for speed to stabilize
            time.sleep(0.5)  # Longer stabilization time
            
            # Read actual speeds multiple times
            speed_readings = []
            for i in range(10):
                speeds = controller.GetSpeeds(config['address'])
                if speeds[0]:  # Success flag
                    speed_readings.append((speeds[1], speeds[2]))
                time.sleep(0.05)
            
            if speed_readings:
                # Calculate average actual speeds
                avg_left_counts = sum(r[0] for r in speed_readings) / len(speed_readings)
                avg_right_counts = sum(r[1] for r in speed_readings) / len(speed_readings)
                
                # Convert back to rad/s
                avg_left_rad_s = unit_converter.counts_per_sec_to_rad_per_sec(avg_left_counts)
                avg_right_rad_s = unit_converter.counts_per_sec_to_rad_per_sec(avg_right_counts)
                
                print(f"  Actual speeds (counts/sec): L={avg_left_counts:.0f}, R={avg_right_counts:.0f}")
                print(f"  Actual speeds (rad/s): L={avg_left_rad_s:.3f}, R={avg_right_rad_s:.3f}")
                
                # Calculate errors
                left_error = abs(avg_left_rad_s - target_speed_rad_s) / target_speed_rad_s * 100
                right_error = abs(avg_right_rad_s - target_speed_rad_s) / target_speed_rad_s * 100
                
                print(f"  Speed errors: L={left_error:.1f}%, R={right_error:.1f}%")
                
                # Check if within tolerance
                within_5_percent = left_error < 5.0 and right_error < 5.0
                within_10_percent = left_error < 10.0 and right_error < 10.0
                
                print(f"  Within 5%: {within_5_percent}")
                print(f"  Within 10%: {within_10_percent}")
                
                # Check unit conversion round-trip
                round_trip_counts = unit_converter.rad_per_sec_to_counts_per_sec(avg_left_rad_s)
                round_trip_rad_s = unit_converter.counts_per_sec_to_rad_per_sec(round_trip_counts)
                print(f"  Unit conversion round-trip: {avg_left_rad_s:.6f} → {round_trip_counts:.0f} → {round_trip_rad_s:.6f}")
                
            else:
                print("  ❌ No speed readings obtained")
        
        # Stop motors
        controller.DutyM1M2(config['address'], 0, 0)
        
        print("\n🔧 TESTING PERFORMANCE TEST SPEED MEASUREMENT METHOD")
        print("-" * 50)
        
        # Test the same method as performance test
        hw = BasicmicroHardwareInterface()
        hw.unit_converter = unit_converter
        hw.controller = controller
        hw.address = config['address']
        hw.motion_strategy = MotionStrategy.SPEED_ACCEL
        hw.default_acceleration = 1000
        hw.hw_commands_velocities_ = [0.0, 0.0]
        hw.hw_states_positions_ = [0.0, 0.0]
        hw.hw_states_velocities_ = [0.0, 0.0]
        hw.emergency_stop_active = False
        
        target_velocity = 5.0  # 5 rad/s test
        print(f"Testing performance test method with {target_velocity} rad/s")
        
        # Record initial state
        initial_speeds = controller.GetSpeeds(config['address'])
        print(f"Initial speeds: {initial_speeds}")
        
        # Set velocity command using hardware interface
        hw.hw_commands_velocities_[0] = target_velocity
        hw.hw_commands_velocities_[1] = target_velocity
        
        command_start = time.perf_counter()
        result = hw.write(time=0.0, period=0.1)
        command_end = time.perf_counter()
        
        print(f"Hardware interface write result: {result}")
        print(f"Command latency: {(command_end - command_start)*1000:.2f}ms")
        
        # Performance test timing (too short?)
        time.sleep(0.02)  # Same as performance test
        
        # Read final speeds
        final_speeds = controller.GetSpeeds(config['address'])
        print(f"Final speeds (after 0.02s): {final_speeds}")
        
        if final_speeds[0]:
            target_velocity_counts = unit_converter.rad_per_sec_to_counts_per_sec(target_velocity)
            actual_velocity_counts = final_speeds[1]
            
            print(f"Target: {target_velocity} rad/s = {target_velocity_counts:.0f} counts/sec")
            print(f"Actual: {actual_velocity_counts} counts/sec")
            
            if target_velocity != 0:
                speed_error = abs(actual_velocity_counts - target_velocity_counts) / target_velocity_counts
                print(f"Speed error (performance test method): {speed_error:.1%}")
                print(f"Within 10%: {speed_error < 0.1}")
        
        # Now test with longer stabilization time
        time.sleep(0.5)  # Longer stabilization
        
        final_speeds_stabilized = controller.GetSpeeds(config['address'])
        print(f"Final speeds (after 0.5s): {final_speeds_stabilized}")
        
        if final_speeds_stabilized[0]:
            actual_velocity_counts_stabilized = final_speeds_stabilized[1]
            
            print(f"Actual (stabilized): {actual_velocity_counts_stabilized} counts/sec")
            
            if target_velocity != 0:
                speed_error_stabilized = abs(actual_velocity_counts_stabilized - target_velocity_counts) / target_velocity_counts
                print(f"Speed error (stabilized): {speed_error_stabilized:.1%}")
                print(f"Within 10% (stabilized): {speed_error_stabilized < 0.1}")
        
        # Stop motors
        controller.DutyM1M2(config['address'], 0, 0)
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_speed_accuracy()