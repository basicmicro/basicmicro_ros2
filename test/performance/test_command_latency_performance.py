"""
Command Latency Performance Tests

Tests the latency of various command executions to ensure they meet
real-time performance requirements for robotic applications.

Performance Targets:
- Simple commands: <5ms
- Complex commands: <10ms
- Service calls: <20ms
- Bulk operations: <50ms

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import pytest
import sys
import os
import time
import statistics
from unittest.mock import Mock, patch
import math

# Add package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from basicmicro_driver.hardware_interface import BasicmicroHardwareInterface
from basicmicro_driver.unit_converter import UnitConverter
from test_mocks.mock_basicmicro import create_mock_controller


@pytest.mark.performance
class TestCommandLatencyPerformance:
    """Test command execution latency performance"""
    
    def setup_method(self):
        """Set up performance test environment"""
        self.hw = BasicmicroHardwareInterface()
        self.hw.controller = create_mock_controller("/dev/ttyACM0", 38400)
        self.hw.address = 128
        self.hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        self.hw.emergency_stop_active = False
        
        # Set up interfaces
        self.hw.hw_commands_velocities_ = [0.0, 0.0]
        self.hw.hw_states_positions_ = [0.0, 0.0]
        self.hw.hw_states_velocities_ = [0.0, 0.0]
        
        # Configure for performance testing
        self.hw.motion_strategy = self.hw.MotionStrategy.SPEED_ACCEL
        self.hw.default_acceleration = 1000
        
        # Performance test parameters
        self.num_iterations = 100
        self.latency_target_ms = 5.0  # 5ms target for simple commands
        
    def _measure_latency(self, operation, iterations=None):
        """Measure operation latency"""
        if iterations is None:
            iterations = self.num_iterations
            
        latencies = []
        
        for _ in range(iterations):
            start_time = time.perf_counter()
            operation()
            end_time = time.perf_counter()
            latencies.append((end_time - start_time) * 1000)  # Convert to ms
            
        return {
            'mean': statistics.mean(latencies),
            'median': statistics.median(latencies),
            'min': min(latencies),
            'max': max(latencies),
            'std': statistics.stdev(latencies) if len(latencies) > 1 else 0,
            'p95': sorted(latencies)[int(0.95 * len(latencies))],
            'p99': sorted(latencies)[int(0.99 * len(latencies))],
            'samples': latencies
        }
        
    def test_velocity_command_latency(self, performance_timer):
        """Test velocity command execution latency"""
        # Set up command
        self.hw.hw_commands_velocities_[0] = 1.0
        self.hw.hw_commands_velocities_[1] = 1.0
        self.hw.controller.SpeedAccelM1M2.return_value = True
        
        # Measure latency
        stats = self._measure_latency(lambda: self.hw.write())
        
        # Verify performance
        assert stats['mean'] < self.latency_target_ms, \
            f"Mean latency {stats['mean']:.2f}ms exceeds target {self.latency_target_ms}ms"
        assert stats['p95'] < self.latency_target_ms * 2, \
            f"95th percentile {stats['p95']:.2f}ms exceeds threshold"
        assert stats['max'] < self.latency_target_ms * 3, \
            f"Max latency {stats['max']:.2f}ms exceeds threshold"
            
        print(f"Velocity command latency - Mean: {stats['mean']:.2f}ms, "
              f"P95: {stats['p95']:.2f}ms, Max: {stats['max']:.2f}ms")
              
    def test_sensor_reading_latency(self, performance_timer):
        """Test sensor reading latency"""
        # Set up mock sensor responses
        self.hw.controller.GetEncoders.return_value = (True, 1000, 1500)
        self.hw.controller.GetSpeeds.return_value = (True, 100, 150)
        
        # Measure latency
        stats = self._measure_latency(lambda: self.hw.read())
        
        # Verify performance
        assert stats['mean'] < self.latency_target_ms, \
            f"Mean sensor reading latency {stats['mean']:.2f}ms exceeds target"
        assert stats['p95'] < self.latency_target_ms * 2, \
            f"95th percentile sensor reading {stats['p95']:.2f}ms exceeds threshold"
            
        print(f"Sensor reading latency - Mean: {stats['mean']:.2f}ms, "
              f"P95: {stats['p95']:.2f}ms, Max: {stats['max']:.2f}ms")
              
    def test_emergency_stop_latency(self, performance_timer):
        """Test emergency stop execution latency"""
        # Emergency stop must be as fast as possible
        emergency_target_ms = 2.0  # Stricter target for safety
        
        # Reset emergency stop flag for each test
        def emergency_stop_test():
            self.hw.emergency_stop_active = False
            return self.hw.emergency_stop()
            
        self.hw.controller.DutyM1M2.return_value = True
        
        # Measure latency
        stats = self._measure_latency(emergency_stop_test)
        
        # Verify performance (stricter requirements for safety)
        assert stats['mean'] < emergency_target_ms, \
            f"Mean emergency stop latency {stats['mean']:.2f}ms exceeds target {emergency_target_ms}ms"
        assert stats['max'] < emergency_target_ms * 2, \
            f"Max emergency stop latency {stats['max']:.2f}ms exceeds threshold"
            
        print(f"Emergency stop latency - Mean: {stats['mean']:.2f}ms, "
              f"P95: {stats['p95']:.2f}ms, Max: {stats['max']:.2f}ms")
              
    def test_unit_conversion_latency(self, performance_timer):
        """Test unit conversion performance"""
        conversion_target_ms = 0.1  # Very fast for mathematical operations
        
        # Test various conversion operations
        test_values = [0.1, 0.5, 1.0, 2.0, 5.0]
        
        def conversion_test():
            for value in test_values:
                # Position conversions
                counts = self.hw.unit_converter.radians_to_counts(value)
                self.hw.unit_converter.counts_to_radians(counts)
                
                # Velocity conversions
                counts_per_sec = self.hw.unit_converter.rad_per_sec_to_counts_per_sec(value)
                self.hw.unit_converter.counts_per_sec_to_rad_per_sec(counts_per_sec)
                
                # Distance conversions
                dist_counts = self.hw.unit_converter.meters_to_counts(value)
                self.hw.unit_converter.counts_to_meters(dist_counts)
                
        # Measure latency
        stats = self._measure_latency(conversion_test, iterations=1000)  # More iterations for precision
        
        # Verify performance
        assert stats['mean'] < conversion_target_ms, \
            f"Mean unit conversion latency {stats['mean']:.3f}ms exceeds target {conversion_target_ms}ms"
            
        print(f"Unit conversion latency - Mean: {stats['mean']:.3f}ms, "
              f"P95: {stats['p95']:.3f}ms, Max: {stats['max']:.3f}ms")
              
    def test_motion_strategy_switching_latency(self, performance_timer):
        """Test motion strategy switching performance"""
        strategies = [
            self.hw.MotionStrategy.DUTY,
            self.hw.MotionStrategy.SPEED,
            self.hw.MotionStrategy.SPEED_ACCEL,
            self.hw.MotionStrategy.DUTY_ACCEL
        ]
        
        # Mock all possible command types
        self.hw.controller.DutyM1M2.return_value = True
        self.hw.controller.SpeedM1M2.return_value = True
        self.hw.controller.SpeedAccelM1M2.return_value = True
        self.hw.controller.DutyAccelM1M2.return_value = True
        
        self.hw.max_duty_cycle = 16384
        
        def strategy_switch_test():
            for strategy in strategies:
                self.hw.motion_strategy = strategy
                self.hw.hw_commands_velocities_[0] = 1.0
                self.hw.hw_commands_velocities_[1] = 1.0
                self.hw.write()
                
        # Measure latency
        stats = self._measure_latency(strategy_switch_test, iterations=50)
        
        # Verify performance
        complex_target_ms = 10.0  # Higher target for complex operations
        assert stats['mean'] < complex_target_ms, \
            f"Mean strategy switching latency {stats['mean']:.2f}ms exceeds target"
            
        print(f"Motion strategy switching latency - Mean: {stats['mean']:.2f}ms, "
              f"P95: {stats['p95']:.2f}ms, Max: {stats['max']:.2f}ms")
              
    def test_continuous_operation_latency(self, performance_timer):
        """Test latency under continuous operation"""
        # Simulate continuous read/write cycles
        cycle_target_ms = 10.0  # Target for complete read/write cycle
        
        # Set up varying operation parameters
        self.hw.controller.GetEncoders.return_value = (True, 1000, 1500)
        self.hw.controller.GetSpeeds.return_value = (True, 100, 150)
        self.hw.controller.SpeedAccelM1M2.return_value = True
        
        def continuous_cycle():
            # Simulate varying velocity commands
            self.hw.hw_commands_velocities_[0] = 0.5 + 0.5 * math.sin(time.time())
            self.hw.hw_commands_velocities_[1] = 0.5 + 0.5 * math.cos(time.time())
            
            # Execute read/write cycle
            self.hw.read()
            self.hw.write()
            
        # Measure latency
        stats = self._measure_latency(continuous_cycle, iterations=200)
        
        # Verify performance
        assert stats['mean'] < cycle_target_ms, \
            f"Mean continuous cycle latency {stats['mean']:.2f}ms exceeds target"
        assert stats['p99'] < cycle_target_ms * 2, \
            f"99th percentile {stats['p99']:.2f}ms exceeds threshold"
            
        print(f"Continuous operation latency - Mean: {stats['mean']:.2f}ms, "
              f"P95: {stats['p95']:.2f}ms, P99: {stats['p99']:.2f}ms")
              
    def test_memory_allocation_impact(self, performance_timer, memory_profiler):
        """Test performance impact of memory allocation"""
        if memory_profiler is None:
            pytest.skip("Memory profiler not available")
            
        # Baseline measurement
        baseline_stats = self._measure_latency(lambda: self.hw.write(), iterations=50)
        baseline_memory = memory_profiler.get_usage()
        
        # Test with memory allocation
        def memory_intensive_operation():
            # Allocate some memory during operation
            temp_data = [0.0] * 10000  # Allocate list
            temp_dict = {i: i*2 for i in range(1000)}  # Allocate dict
            
            self.hw.write()
            
            # Clean up to trigger potential GC
            del temp_data
            del temp_dict
            
        memory_stats = self._measure_latency(memory_intensive_operation, iterations=50)
        memory_profiler.update()
        final_memory = memory_profiler.get_usage()
        
        # Performance should not degrade significantly due to memory allocation
        performance_degradation = (memory_stats['mean'] - baseline_stats['mean']) / baseline_stats['mean']
        
        assert performance_degradation < 0.5, \
            f"Memory allocation caused {performance_degradation*100:.1f}% performance degradation"
            
        print(f"Memory allocation impact - Baseline: {baseline_stats['mean']:.2f}ms, "
              f"With allocation: {memory_stats['mean']:.2f}ms, "
              f"Degradation: {performance_degradation*100:.1f}%")
        print(f"Memory usage - Delta: {final_memory['delta_mb']:.1f}MB, "
              f"Peak: {final_memory['peak_mb']:.1f}MB")


@pytest.mark.performance  
class TestBulkOperationPerformance:
    """Test performance of bulk operations"""
    
    def setup_method(self):
        """Set up bulk operation performance tests"""
        self.mock_controller = create_mock_controller("/dev/ttyACM0", 38400)
        self.unit_converter = UnitConverter(0.1, 1000, 1.0)
        
    def test_bulk_distance_command_performance(self, performance_timer):
        """Test bulk distance command performance"""
        bulk_target_ms = 50.0  # Target for bulk operations
        
        # Prepare bulk distance commands
        num_commands = 32  # Max buffer size
        commands = []
        for i in range(num_commands):
            commands.append({
                'distance': 0.1 * (i + 1),
                'speed': 0.5,
                'acceleration': 1.0
            })
            
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = True
        
        def bulk_distance_execution():
            for i, cmd in enumerate(commands):
                # Convert units
                distance_counts = self.unit_converter.meters_to_counts(cmd['distance'])
                speed_counts = int(cmd['speed'] / (2 * math.pi * 0.1 / 1000))
                accel_counts = int(cmd['acceleration'] / (2 * math.pi * 0.1 / 1000))
                
                # Execute command
                buffer_flag = 1 if i < len(commands) - 1 else 0
                self.mock_controller.SpeedAccelDistanceM1M2(
                    128, accel_counts, speed_counts, distance_counts,
                    speed_counts, distance_counts, buffer_flag
                )
                
        # Measure performance
        with performance_timer:
            bulk_distance_execution()
            
        elapsed_ms = performance_timer.elapsed * 1000
        
        assert elapsed_ms < bulk_target_ms, \
            f"Bulk distance commands took {elapsed_ms:.2f}ms, exceeds {bulk_target_ms}ms target"
            
        commands_per_ms = num_commands / elapsed_ms
        print(f"Bulk distance performance - {elapsed_ms:.2f}ms for {num_commands} commands "
              f"({commands_per_ms:.1f} commands/ms)")
              
    def test_bulk_sensor_reading_performance(self, performance_timer):
        """Test bulk sensor reading performance"""
        # Simulate reading multiple sensors rapidly
        num_readings = 100
        
        # Mock sensor responses
        self.mock_controller.GetEncoders.return_value = (True, 1000, 1500)
        self.mock_controller.GetSpeeds.return_value = (True, 100, 150)
        self.mock_controller.GetVolts.return_value = (True, 12.5, 5.0)
        self.mock_controller.GetCurrents.return_value = (True, 2.5, 3.0)
        self.mock_controller.GetTemps.return_value = (True, 25, 27)
        
        def bulk_sensor_reading():
            for _ in range(num_readings):
                # Read all sensor types
                self.mock_controller.GetEncoders(128)
                self.mock_controller.GetSpeeds(128)
                self.mock_controller.GetVolts(128)
                self.mock_controller.GetCurrents(128)
                self.mock_controller.GetTemps(128)
                
        # Measure performance
        with performance_timer:
            bulk_sensor_reading()
            
        elapsed_ms = performance_timer.elapsed * 1000
        readings_per_ms = (num_readings * 5) / elapsed_ms  # 5 sensor types per reading
        
        # Target: >10 readings/ms (realistic for 38400 baud)
        target_readings_per_ms = 10.0
        assert readings_per_ms > target_readings_per_ms, \
            f"Sensor reading rate {readings_per_ms:.1f} readings/ms below target {target_readings_per_ms}"
            
        print(f"Bulk sensor reading performance - {elapsed_ms:.2f}ms for {num_readings*5} readings "
              f"({readings_per_ms:.1f} readings/ms)")
              
    def test_trajectory_processing_performance(self, performance_timer):
        """Test trajectory processing performance"""
        # Create large trajectory
        trajectory_size = 100
        trajectory_points = []
        
        for i in range(trajectory_size):
            point = {
                'type': 'distance' if i % 2 == 0 else 'position',
                'left_value': 0.1 * i,
                'right_value': 0.1 * i,
                'speed': 0.5,
                'acceleration': 1.0,
                'deceleration': 1.0 if i % 2 == 1 else None
            }
            trajectory_points.append(point)
            
        # Mock command execution
        self.mock_controller.SpeedAccelDistanceM1M2.return_value = True
        self.mock_controller.SpeedAccelDeccelPositionM1M2.return_value = True
        
        def trajectory_processing():
            for i, point in enumerate(trajectory_points):
                if point['type'] == 'distance':
                    # Process distance command
                    distance_counts = self.unit_converter.meters_to_counts(point['left_value'])
                    speed_counts = int(point['speed'] / (2 * math.pi * 0.1 / 1000))
                    accel_counts = int(point['acceleration'] / (2 * math.pi * 0.1 / 1000))
                    
                    self.mock_controller.SpeedAccelDistanceM1M2(
                        128, accel_counts, speed_counts, distance_counts,
                        speed_counts, distance_counts, 1 if i < len(trajectory_points)-1 else 0
                    )
                else:
                    # Process position command
                    position_counts = int(point['left_value'] / (2 * math.pi / 1000))
                    speed_counts = int(point['speed'] / (2 * math.pi * 0.1 / 1000))
                    accel_counts = int(point['acceleration'] / (2 * math.pi * 0.1 / 1000))
                    decel_counts = int(point['deceleration'] / (2 * math.pi * 0.1 / 1000))
                    
                    self.mock_controller.SpeedAccelDeccelPositionM1M2(
                        128, accel_counts, speed_counts, decel_counts, position_counts,
                        accel_counts, speed_counts, decel_counts, position_counts,
                        1 if i < len(trajectory_points)-1 else 0
                    )
                    
        # Measure performance
        with performance_timer:
            trajectory_processing()
            
        elapsed_ms = performance_timer.elapsed * 1000
        points_per_ms = trajectory_size / elapsed_ms
        
        # Target: >1 point/ms for trajectory processing
        target_points_per_ms = 1.0
        assert points_per_ms > target_points_per_ms, \
            f"Trajectory processing rate {points_per_ms:.2f} points/ms below target {target_points_per_ms}"
            
        print(f"Trajectory processing performance - {elapsed_ms:.2f}ms for {trajectory_size} points "
              f"({points_per_ms:.2f} points/ms)")


@pytest.mark.performance
class TestConcurrentOperationPerformance:
    """Test performance under concurrent operations"""
    
    def setup_method(self):
        """Set up concurrent operation tests"""
        self.hw = BasicmicroHardwareInterface()
        self.hw.controller = create_mock_controller("/dev/ttyACM0", 38400)
        self.hw.address = 128
        self.hw.unit_converter = UnitConverter(0.1, 1000, 1.0)
        
        # Set up interfaces
        self.hw.hw_commands_velocities_ = [0.0, 0.0]
        self.hw.hw_states_positions_ = [0.0, 0.0]
        self.hw.hw_states_velocities_ = [0.0, 0.0]
        
        # Mock successful operations
        self.hw.controller.GetEncoders.return_value = (True, 1000, 1500)
        self.hw.controller.GetSpeeds.return_value = (True, 100, 150)
        self.hw.controller.SpeedAccelM1M2.return_value = True
        
    def test_interleaved_read_write_performance(self, performance_timer):
        """Test performance of interleaved read/write operations"""
        num_cycles = 100
        interleaved_target_ms = 50.0
        
        def interleaved_operations():
            for i in range(num_cycles):
                # Vary commands
                self.hw.hw_commands_velocities_[0] = 0.5 + 0.5 * (i % 10) / 10
                self.hw.hw_commands_velocities_[1] = 0.5 - 0.5 * (i % 10) / 10
                
                # Interleave read and write
                if i % 2 == 0:
                    self.hw.read()
                    self.hw.write()
                else:
                    self.hw.write()
                    self.hw.read()
                    
        # Measure performance
        with performance_timer:
            interleaved_operations()
            
        elapsed_ms = performance_timer.elapsed * 1000
        cycles_per_ms = num_cycles / elapsed_ms
        
        assert elapsed_ms < interleaved_target_ms, \
            f"Interleaved operations took {elapsed_ms:.2f}ms, exceeds {interleaved_target_ms}ms target"
            
        print(f"Interleaved read/write performance - {elapsed_ms:.2f}ms for {num_cycles} cycles "
              f"({cycles_per_ms:.2f} cycles/ms)")
              
    def test_error_recovery_performance(self, performance_timer):
        """Test performance during error recovery scenarios"""
        recovery_target_ms = 20.0
        
        def error_recovery_scenario():
            # Normal operation
            self.hw.read()
            self.hw.write()
            
            # Simulate error
            self.hw.controller.GetEncoders.return_value = (False, 0, 0)
            self.hw.read()  # Should fail gracefully
            
            # Recovery
            self.hw.controller.GetEncoders.return_value = (True, 1000, 1500)
            self.hw.read()  # Should succeed
            self.hw.write()
            
        # Measure performance
        with performance_timer:
            for _ in range(10):  # Multiple error/recovery cycles
                error_recovery_scenario()
                
        elapsed_ms = performance_timer.elapsed * 1000
        
        assert elapsed_ms < recovery_target_ms, \
            f"Error recovery took {elapsed_ms:.2f}ms, exceeds {recovery_target_ms}ms target"
            
        print(f"Error recovery performance - {elapsed_ms:.2f}ms for 10 error/recovery cycles")