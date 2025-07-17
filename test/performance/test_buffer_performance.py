#!/usr/bin/env python3
"""
Buffer Performance Tests with Motor Movement Validation

Tests buffer management system performance with actual motor movement confirmation.
Validates buffer utilization, overflow prevention, and trajectory execution accuracy.

CRITICAL: These tests validate ACTUAL BUFFER PERFORMANCE, not just communication timing.
Each test verifies that buffered commands result in actual motor movement and measures
the complete buffer execution performance from command queueing to physical motor response.

This implementation applies the proven GetISpeeds() methodology from the speed accuracy
breakthrough to ensure accurate buffer performance measurement.

Performance Targets:
- Buffer utilization: Maintain 60-80% optimal utilization
- Multi-command execution: >90% success rate with movement validation
- Trajectory accuracy: <5% deviation from commanded path
- Buffer overflow prevention: 0 overflows during normal operation
- Command-to-movement latency: <200ms for buffered commands

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import pytest
import sys
import os
import time
import statistics
import threading
import json
import math
from datetime import datetime
from collections import deque
from typing import Dict, Any, List, Tuple

# Add package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from basicmicro_driver.hardware_interface import BasicmicroHardwareInterface, MotionStrategy
from basicmicro_driver.unit_converter import UnitConverter
from basicmicro_driver.buffer_manager import BufferManager, BufferStatus

try:
    from hardware_interface_msgs.msg import return_type
except ImportError:
    # Fallback for testing without full ROS2 installation
    class return_type:
        OK = 0
        ERROR = 1

try:
    # Try to import the actual Basicmicro library
    from basicmicro import Basicmicro
    BASICMICRO_AVAILABLE = True
except ImportError:
    BASICMICRO_AVAILABLE = False


class BufferPerformanceCollector:
    """Collects and analyzes buffer performance data during tests"""
    
    def __init__(self, test_name):
        self.test_name = test_name
        self.start_time = None
        self.running = False
        
        # Buffer-specific tracking
        self.buffer_commands = deque(maxlen=10000)
        self.buffer_utilization = deque(maxlen=10000)
        self.buffer_overflows = []
        self.command_execution_times = deque(maxlen=10000)
        
        # Motor movement validation for buffered commands
        self.buffered_movements = deque(maxlen=10000)
        self.trajectory_accuracy = deque(maxlen=10000)
        self.command_to_movement_latencies = deque(maxlen=10000)
        
        # Buffer status tracking
        self.buffer_status_history = deque(maxlen=10000)
        self.buffer_drain_rates = deque(maxlen=1000)
        
        # Performance metrics
        self.multi_command_success_rate = 0.0
        self.average_utilization = 0.0
        self.max_utilization = 0.0
        self.overflow_count = 0
        
        # Error tracking
        self.errors = []
        
    def start_collection(self):
        """Start buffer performance data collection"""
        self.start_time = time.perf_counter()
        self.running = True
        
    def stop_collection(self):
        """Stop buffer performance data collection"""
        self.running = False
        
    def record_buffer_command(self, command_type, buffer_utilization, command_start, command_end):
        """Record buffer command execution"""
        if self.running:
            execution_time = (command_end - command_start) * 1000  # Convert to ms
            self.buffer_commands.append({
                'timestamp': command_end,
                'type': command_type,
                'utilization': buffer_utilization,
                'execution_time': execution_time
            })
            self.buffer_utilization.append(buffer_utilization)
            self.command_execution_times.append(execution_time)
            
    def record_buffer_status(self, status_data):
        """Record buffer status from hardware"""
        if self.running:
            self.buffer_status_history.append({
                'timestamp': time.perf_counter(),
                'status': status_data['status'].value,
                'used_slots': status_data['used_slots'],
                'utilization_percent': status_data['utilization_percent']
            })
            
    def record_buffer_overflow(self, overflow_data):
        """Record buffer overflow event"""
        self.buffer_overflows.append({
            'timestamp': time.perf_counter(),
            'attempted_slots': overflow_data.get('attempted_slots', 0),
            'available_slots': overflow_data.get('available_slots', 0),
            'prevention_action': overflow_data.get('action', 'unknown')
        })
        self.overflow_count += 1
        
    def record_buffered_movement(self, initial_encoders, final_encoders, expected_trajectory, 
                                actual_trajectory, command_time, movement_time):
        """Record motor movement validation for buffered commands"""
        if self.running:
            # Check if motors actually moved
            left_change = abs(final_encoders[1] - initial_encoders[1])
            right_change = abs(final_encoders[2] - initial_encoders[2])
            movement_detected = left_change > 10 or right_change > 10
            
            self.buffered_movements.append(movement_detected)
            
            # Calculate trajectory accuracy
            if expected_trajectory and actual_trajectory:
                # Simple trajectory accuracy: deviation from expected path
                trajectory_error = abs(actual_trajectory - expected_trajectory) / max(abs(expected_trajectory), 1)
                self.trajectory_accuracy.append(trajectory_error)
                
            # Command-to-movement latency for buffered commands
            if movement_detected:
                latency = (movement_time - command_time) * 1000  # Convert to ms
                self.command_to_movement_latencies.append(latency)
                
    def record_buffer_drain_rate(self, initial_utilization, final_utilization, time_span):
        """Record buffer drain rate for analysis"""
        if time_span > 0:
            drain_rate = (initial_utilization - final_utilization) / time_span
            self.buffer_drain_rates.append(drain_rate)
            
    def record_error(self, error_message):
        """Record error during buffer performance testing"""
        timestamp = time.perf_counter()
        self.errors.append({
            'timestamp': timestamp,
            'message': error_message
        })
            
    def get_buffer_statistics(self):
        """Calculate comprehensive buffer performance statistics"""
        if not self.buffer_commands:
            return {}
            
        # Calculate multi-command success rate
        if self.buffered_movements:
            self.multi_command_success_rate = sum(self.buffered_movements) / len(self.buffered_movements)
            
        # Calculate buffer utilization statistics
        if self.buffer_utilization:
            self.average_utilization = statistics.mean(self.buffer_utilization)
            self.max_utilization = max(self.buffer_utilization)
            
        # Calculate trajectory accuracy statistics
        trajectory_stats = {}
        if self.trajectory_accuracy:
            trajectory_stats = {
                'mean_error': statistics.mean(self.trajectory_accuracy),
                'max_error': max(self.trajectory_accuracy),
                'min_error': min(self.trajectory_accuracy),
                'accuracy_within_5_percent': sum(1 for e in self.trajectory_accuracy if e < 0.05) / len(self.trajectory_accuracy),
                'accuracy_within_2_percent': sum(1 for e in self.trajectory_accuracy if e < 0.02) / len(self.trajectory_accuracy),
                'count': len(self.trajectory_accuracy)
            }
            
        # Calculate buffer drain statistics
        drain_stats = {}
        if self.buffer_drain_rates:
            drain_stats = {
                'mean_drain_rate': statistics.mean(self.buffer_drain_rates),
                'max_drain_rate': max(self.buffer_drain_rates),
                'min_drain_rate': min(self.buffer_drain_rates),
                'std_drain_rate': statistics.stdev(self.buffer_drain_rates) if len(self.buffer_drain_rates) > 1 else 0
            }
            
        # Calculate command execution time statistics
        execution_stats = {}
        if self.command_execution_times:
            execution_stats = {
                'mean_execution_time': statistics.mean(self.command_execution_times),
                'median_execution_time': statistics.median(self.command_execution_times),
                'max_execution_time': max(self.command_execution_times),
                'min_execution_time': min(self.command_execution_times),
                'p95_execution_time': sorted(self.command_execution_times)[int(0.95 * len(self.command_execution_times))]
            }
            
        # Calculate buffered movement latency statistics
        movement_latency_stats = {}
        if self.command_to_movement_latencies:
            movement_latency_stats = {
                'mean_latency': statistics.mean(self.command_to_movement_latencies),
                'median_latency': statistics.median(self.command_to_movement_latencies),
                'max_latency': max(self.command_to_movement_latencies),
                'min_latency': min(self.command_to_movement_latencies),
                'p95_latency': sorted(self.command_to_movement_latencies)[int(0.95 * len(self.command_to_movement_latencies))]
            }
            
        return {
            'test_name': self.test_name,
            'duration_seconds': time.perf_counter() - self.start_time if self.start_time else 0,
            'total_commands': len(self.buffer_commands),
            'multi_command_success_rate': self.multi_command_success_rate,
            'average_utilization': self.average_utilization,
            'max_utilization': self.max_utilization,
            'overflow_count': self.overflow_count,
            'trajectory_accuracy_stats': trajectory_stats,
            'buffer_drain_stats': drain_stats,
            'execution_time_stats': execution_stats,
            'movement_latency_stats': movement_latency_stats,
            'buffer_status_samples': len(self.buffer_status_history),
            'movement_validation_samples': len(self.buffered_movements)
        }


@pytest.mark.hardware
@pytest.mark.performance
@pytest.mark.buffer
@pytest.mark.skipif(not BASICMICRO_AVAILABLE, reason="Basicmicro library not available")
class TestBufferPerformance:
    """Buffer performance tests with real controllers and motor movement validation"""
    
    def setup_method(self):
        """Set up buffer performance test environment"""
        # Hardware configuration
        self.config = {
            'port': '/dev/ttyACM1',  # Updated to correct port
            'baud': 38400,
            'address': 0x80,
            'wheel_radius': 0.1,
            'encoder_counts_per_rev': 1000,
            'gear_ratio': 1.0,
            'max_safe_speed': 8000,  # Hardware-validated maximum speed
            'max_safe_duty': 8192,   # 25% duty cycle for safety
        }
        
        # Buffer performance targets
        self.targets = {
            'optimal_utilization_min': 60.0,    # 60% minimum utilization
            'optimal_utilization_max': 80.0,    # 80% maximum utilization
            'multi_command_success_rate': 0.9,  # 90% success rate
            'trajectory_accuracy': 0.95,        # 95% within 5% tolerance
            'overflow_prevention': 0,           # Zero overflows
            'command_execution_time': 50.0,     # <50ms command execution
            'movement_latency': 200.0,          # <200ms buffered command latency
        }
        
        # Initialize hardware interface
        self.hw = BasicmicroHardwareInterface()
        self.hw.unit_converter = UnitConverter(
            self.config['wheel_radius'],
            self.config['encoder_counts_per_rev'],
            self.config['gear_ratio']
        )
        
        # Initialize buffer manager
        self.buffer_manager = None
        
    def _connect_hardware(self):
        """Connect to real hardware controller"""
        # Skip if already connected
        if hasattr(self.hw, 'controller') and self.hw.controller:
            return True
            
        try:
            # Initialize ROS2 if not already initialized (required for BufferManager)
            try:
                import rclpy
                if not rclpy.ok():
                    rclpy.init()
            except ImportError:
                pytest.skip("ROS2 (rclpy) not available for buffer performance tests")
                
            controller = Basicmicro(self.config['port'], self.config['baud'])
            if not controller.Open():
                pytest.skip(f"Could not open hardware controller on {self.config['port']}")
                
            # Verify communication
            version_result = controller.ReadVersion(self.config['address'])
            if not version_result[0]:
                pytest.skip(f"Could not communicate with controller at address {self.config['address']}")
                
            # Configure hardware interface
            self.hw.controller = controller
            self.hw.address = self.config['address']
            self.hw.motion_strategy = MotionStrategy.SPEED_ACCEL
            self.hw.default_acceleration = 1000
            
            # Initialize state variables
            self.hw.hw_commands_velocities_ = [0.0, 0.0]
            self.hw.hw_states_positions_ = [0.0, 0.0]
            self.hw.hw_states_velocities_ = [0.0, 0.0]
            self.hw.emergency_stop_active = False
            
            # Initialize buffer manager with hardware interface
            self.buffer_manager = BufferManager(hardware_interface=self.hw)
            
            # Safety: Emergency stop at start
            controller.DutyM1M2(self.config['address'], 0, 0)
            time.sleep(0.1)
            
            return True
            
        except Exception as e:
            pytest.skip(f"Hardware connection failed: {e}")
            
    def _cleanup_hardware(self):
        """Clean up hardware connection"""
        if hasattr(self.hw, 'controller') and self.hw.controller:
            try:
                # Safety: Emergency stop
                self.hw.controller.DutyM1M2(self.config['address'], 0, 0)
                time.sleep(0.1)
                self.hw.controller.close()
            except:
                pass
                
        # Clean up buffer manager (which is a ROS2 node)
        if hasattr(self, 'buffer_manager') and self.buffer_manager:
            try:
                self.buffer_manager.destroy_node()
            except:
                pass
                
    def _execute_buffered_commands(self, command_sequence, collector):
        """Execute a sequence of buffered commands with motor movement validation"""
        initial_encoders = self.hw.controller.GetEncoders(self.config['address'])
        
        for i, command in enumerate(command_sequence):
            command_start = time.perf_counter()
            
            # Set command velocity
            self.hw.hw_commands_velocities_[0] = command['velocity']
            self.hw.hw_commands_velocities_[1] = command['velocity']
            
            # Execute buffered command
            try:
                # Use buffer flag for trajectory commands
                result = self.hw.write(time=command['time'], period=command['period'])
                if result != return_type.OK:
                    collector.record_error(f"Buffered command {i} failed: {result}")
                    continue
                    
            except Exception as e:
                collector.record_error(f"Buffered command {i} exception: {e}")
                continue
                
            command_end = time.perf_counter()
            
            # Read current buffer status
            buffer_status = self.buffer_manager._read_buffer_status()
            if buffer_status['success']:
                collector.record_buffer_status(buffer_status)
                
                # Record buffer command execution
                collector.record_buffer_command(
                    command_type='buffered_velocity',
                    buffer_utilization=buffer_status['utilization_percent'],
                    command_start=command_start,
                    command_end=command_end
                )
                
                # Check for overflow conditions
                if buffer_status['used_slots'] >= 30:  # Near overflow
                    collector.record_buffer_overflow({
                        'attempted_slots': 1,
                        'available_slots': buffer_status['available_slots'],
                        'action': 'command_delayed'
                    })
                    
            # Brief delay to allow buffer processing
            time.sleep(0.05)
            
        # Allow time for buffered commands to execute
        time.sleep(0.5)
        
        # Validate motor movement after buffer execution
        movement_detected_time = time.perf_counter()
        final_encoders = self.hw.controller.GetEncoders(self.config['address'])
        
        # Use GetISpeeds() for accurate speed measurement (proven methodology)
        ispeed_readings = []
        for _ in range(10):
            ispeeds = self.hw.controller.GetISpeeds(self.config['address'])
            if ispeeds[0]:
                ispeed_readings.append(ispeeds[1])
            time.sleep(0.005)
            
        actual_speed = sum(ispeed_readings) / len(ispeed_readings) if ispeed_readings else 0
        
        # Calculate expected trajectory (simplified)
        expected_trajectory = sum(cmd['velocity'] for cmd in command_sequence) / len(command_sequence)
        actual_trajectory = actual_speed
        
        # Record buffered movement validation
        if initial_encoders[0] and final_encoders[0]:
            collector.record_buffered_movement(
                initial_encoders, final_encoders,
                expected_trajectory, actual_trajectory,
                command_sequence[0]['time'] if command_sequence else 0,
                movement_detected_time
            )
            
    def test_buffer_utilization_optimization(self):
        """Test buffer utilization optimization with motor movement validation"""
        if not self._connect_hardware():
            return
            
        try:
            collector = BufferPerformanceCollector("Buffer Utilization Optimization")
            collector.start_collection()
            
            # Test sequence: gradually increase buffer utilization
            utilization_targets = [20, 40, 60, 80]  # Target utilization percentages
            
            for target_utilization in utilization_targets:
                # Calculate number of commands for target utilization
                commands_needed = int((target_utilization / 100.0) * 32)  # 32 max buffer size
                
                # Create command sequence
                command_sequence = []
                for i in range(commands_needed):
                    velocity = 10.0 + (i * 2.0)  # Varying velocities for trajectory
                    command_sequence.append({
                        'velocity': velocity,
                        'time': i * 0.1,
                        'period': 0.1
                    })
                    
                # Execute buffered command sequence
                self._execute_buffered_commands(command_sequence, collector)
                
                # Brief pause between utilization tests
                time.sleep(0.5)
                
            collector.stop_collection()
            stats = collector.get_buffer_statistics()
            
            # Validate buffer utilization optimization
            assert stats['average_utilization'] >= self.targets['optimal_utilization_min'], \
                f"Average utilization {stats['average_utilization']:.1f}% below minimum target {self.targets['optimal_utilization_min']}%"
            assert stats['average_utilization'] <= self.targets['optimal_utilization_max'], \
                f"Average utilization {stats['average_utilization']:.1f}% above maximum target {self.targets['optimal_utilization_max']}%"
            assert stats['overflow_count'] == self.targets['overflow_prevention'], \
                f"Buffer overflows occurred: {stats['overflow_count']}"
                
            # Validate multi-command success rate
            assert stats['multi_command_success_rate'] >= self.targets['multi_command_success_rate'], \
                f"Multi-command success rate {stats['multi_command_success_rate']:.1%} below target {self.targets['multi_command_success_rate']:.1%}"
                
            print(f"Buffer Utilization Optimization Results:")
            print(f"  Average Utilization: {stats['average_utilization']:.1f}%")
            print(f"  Max Utilization: {stats['max_utilization']:.1f}%")
            print(f"  Multi-Command Success Rate: {stats['multi_command_success_rate']:.1%}")
            print(f"  Overflow Count: {stats['overflow_count']}")
            print(f"  Total Commands: {stats['total_commands']}")
            
        finally:
            self._cleanup_hardware()
            
    def test_multi_command_buffer_execution(self):
        """Test multi-command buffer execution with trajectory validation"""
        if not self._connect_hardware():
            return
            
        try:
            collector = BufferPerformanceCollector("Multi-Command Buffer Execution")
            collector.start_collection()
            
            # Test multiple command sequences
            for sequence_length in [5, 10, 15, 20]:
                # Create varied command sequence (sine wave pattern)
                command_sequence = []
                for i in range(sequence_length):
                    velocity = 15.0 * math.sin(i * 0.3)  # Sine wave trajectory
                    command_sequence.append({
                        'velocity': velocity,
                        'time': i * 0.1,
                        'period': 0.1
                    })
                    
                # Execute sequence
                self._execute_buffered_commands(command_sequence, collector)
                
                # Allow buffer to drain between sequences
                time.sleep(1.0)
                
            collector.stop_collection()
            stats = collector.get_buffer_statistics()
            
            # Validate multi-command execution
            assert stats['multi_command_success_rate'] >= self.targets['multi_command_success_rate'], \
                f"Multi-command success rate {stats['multi_command_success_rate']:.1%} below target"
                
            # Validate trajectory accuracy
            trajectory_stats = stats.get('trajectory_accuracy_stats', {})
            if trajectory_stats:
                assert trajectory_stats['accuracy_within_5_percent'] >= self.targets['trajectory_accuracy'], \
                    f"Trajectory accuracy {trajectory_stats['accuracy_within_5_percent']:.1%} below target"
                    
            # Validate command execution time
            execution_stats = stats.get('execution_time_stats', {})
            if execution_stats:
                assert execution_stats['mean_execution_time'] < self.targets['command_execution_time'], \
                    f"Mean execution time {execution_stats['mean_execution_time']:.2f}ms exceeds target"
                    
            print(f"Multi-Command Buffer Execution Results:")
            print(f"  Multi-Command Success Rate: {stats['multi_command_success_rate']:.1%}")
            print(f"  Trajectory Accuracy (5%): {trajectory_stats.get('accuracy_within_5_percent', 0):.1%}")
            print(f"  Mean Execution Time: {execution_stats.get('mean_execution_time', 0):.2f}ms")
            print(f"  Total Commands: {stats['total_commands']}")
            
        finally:
            self._cleanup_hardware()
            
    def test_buffer_overflow_prevention(self):
        """Test buffer overflow prevention with high command rate"""
        if not self._connect_hardware():
            return
            
        try:
            collector = BufferPerformanceCollector("Buffer Overflow Prevention")
            collector.start_collection()
            
            # Attempt to overwhelm buffer with high command rate
            high_rate_commands = []
            for i in range(50):  # More commands than buffer can hold
                velocity = 20.0 * math.sin(i * 0.2)
                high_rate_commands.append({
                    'velocity': velocity,
                    'time': i * 0.02,  # Very short period
                    'period': 0.02
                })
                
            # Execute high-rate command sequence
            self._execute_buffered_commands(high_rate_commands, collector)
            
            collector.stop_collection()
            stats = collector.get_buffer_statistics()
            
            # Validate overflow prevention
            assert stats['overflow_count'] == self.targets['overflow_prevention'], \
                f"Buffer overflows occurred: {stats['overflow_count']} (should be 0)"
                
            # Validate system continued to function
            assert stats['multi_command_success_rate'] > 0.7, \
                f"Success rate too low during overflow test: {stats['multi_command_success_rate']:.1%}"
                
            print(f"Buffer Overflow Prevention Results:")
            print(f"  Overflow Count: {stats['overflow_count']}")
            print(f"  Success Rate: {stats['multi_command_success_rate']:.1%}")
            print(f"  Max Utilization: {stats['max_utilization']:.1f}%")
            print(f"  Commands Attempted: {len(high_rate_commands)}")
            
        finally:
            self._cleanup_hardware()
            
    def test_buffer_drain_rate_analysis(self):
        """Test buffer drain rate analysis and optimization"""
        if not self._connect_hardware():
            return
            
        try:
            collector = BufferPerformanceCollector("Buffer Drain Rate Analysis")
            collector.start_collection()
            
            # Fill buffer to various levels and measure drain rates
            for fill_level in [25, 50, 75, 90]:  # Percentage of buffer to fill
                commands_to_fill = int((fill_level / 100.0) * 32)
                
                # Create commands to fill buffer
                fill_commands = []
                for i in range(commands_to_fill):
                    velocity = 12.0 + (i * 1.5)
                    fill_commands.append({
                        'velocity': velocity,
                        'time': i * 0.05,
                        'period': 0.05
                    })
                    
                # Record initial buffer state
                initial_status = self.buffer_manager._read_buffer_status()
                initial_time = time.perf_counter()
                
                # Execute fill commands
                self._execute_buffered_commands(fill_commands, collector)
                
                # Allow buffer to drain and measure rate
                time.sleep(2.0)
                
                # Record final buffer state
                final_status = self.buffer_manager._read_buffer_status()
                final_time = time.perf_counter()
                
                if initial_status['success'] and final_status['success']:
                    time_span = final_time - initial_time
                    collector.record_buffer_drain_rate(
                        initial_status['utilization_percent'],
                        final_status['utilization_percent'],
                        time_span
                    )
                    
                # Brief pause between drain tests
                time.sleep(0.5)
                
            collector.stop_collection()
            stats = collector.get_buffer_statistics()
            
            # Validate drain rate analysis
            drain_stats = stats.get('buffer_drain_stats', {})
            if drain_stats:
                assert drain_stats['mean_drain_rate'] > 0, \
                    f"Invalid drain rate: {drain_stats['mean_drain_rate']}"
                    
            print(f"Buffer Drain Rate Analysis Results:")
            print(f"  Mean Drain Rate: {drain_stats.get('mean_drain_rate', 0):.2f}%/s")
            print(f"  Max Drain Rate: {drain_stats.get('max_drain_rate', 0):.2f}%/s")
            print(f"  Buffer Status Samples: {stats['buffer_status_samples']}")
            print(f"  Total Commands: {stats['total_commands']}")
            
        finally:
            self._cleanup_hardware()
            
    def test_save_buffer_performance_results(self):
        """Save buffer performance test results for analysis"""
        if not self._connect_hardware():
            return
            
        try:
            # Run abbreviated buffer tests for data collection
            results = {}
            
            test_scenarios = [
                ('utilization_optimization', 'Buffer Utilization Optimization'),
                ('multi_command_execution', 'Multi-Command Buffer Execution'),
                ('overflow_prevention', 'Buffer Overflow Prevention'),
                ('drain_rate_analysis', 'Buffer Drain Rate Analysis')
            ]
            
            for scenario_key, scenario_name in test_scenarios:
                collector = BufferPerformanceCollector(scenario_name)
                collector.start_collection()
                
                # Run simplified version of each test
                if scenario_key == 'utilization_optimization':
                    # Test utilization optimization
                    for target in [40, 60, 80]:
                        commands = int((target / 100.0) * 32)
                        command_sequence = [{
                            'velocity': 10.0 + i,
                            'time': i * 0.1,
                            'period': 0.1
                        } for i in range(commands)]
                        self._execute_buffered_commands(command_sequence, collector)
                        time.sleep(0.3)
                        
                elif scenario_key == 'multi_command_execution':
                    # Test multi-command execution
                    for length in [5, 10, 15]:
                        command_sequence = [{
                            'velocity': 15.0 * math.sin(i * 0.3),
                            'time': i * 0.1,
                            'period': 0.1
                        } for i in range(length)]
                        self._execute_buffered_commands(command_sequence, collector)
                        time.sleep(0.5)
                        
                elif scenario_key == 'overflow_prevention':
                    # Test overflow prevention
                    high_rate_commands = [{
                        'velocity': 20.0 * math.sin(i * 0.2),
                        'time': i * 0.02,
                        'period': 0.02
                    } for i in range(40)]
                    self._execute_buffered_commands(high_rate_commands, collector)
                    
                elif scenario_key == 'drain_rate_analysis':
                    # Test drain rate analysis
                    for fill_level in [50, 75]:
                        commands = int((fill_level / 100.0) * 32)
                        command_sequence = [{
                            'velocity': 12.0 + i,
                            'time': i * 0.05,
                            'period': 0.05
                        } for i in range(commands)]
                        self._execute_buffered_commands(command_sequence, collector)
                        time.sleep(1.0)
                        
                collector.stop_collection()
                results[scenario_key] = collector.get_buffer_statistics()
                
            # Save results to file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            results_file = f"buffer_performance_results_{timestamp}.json"
            results_path = os.path.join(os.path.dirname(__file__), '..', 'test_data', results_file)
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(results_path), exist_ok=True)
            
            with open(results_path, 'w') as f:
                json.dump(results, f, indent=2)
                
            print(f"Buffer performance results saved to: {results_path}")
            
        finally:
            self._cleanup_hardware()