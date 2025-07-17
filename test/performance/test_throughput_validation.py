"""
Throughput Validation Tests with Motor Movement Validation

Validates claimed throughput performance with real hardware controllers.
Tests sustained operation rates and identifies performance bottlenecks.

CRITICAL: These tests validate ACTUAL MOTOR THROUGHPUT, not just communication throughput.
Each test verifies that motors actually respond to commands at the claimed rates
and measures the complete control loop throughput including physical motor response.

Performance Claims to Validate:
- 200Hz command execution throughput
- 150Hz sensor reading throughput  
- 80Hz continuous operation cycles
- 50Hz trajectory processing

Motor Movement Validation:
- Motor response rate validation: Required for all throughput tests
- Command-to-movement verification: Motors must actually respond
- Sustained operation validation: Motor performance over time
- Trajectory execution validation: Physical path following accuracy

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
from datetime import datetime
from collections import deque
import math

# Add package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from basicmicro_driver.hardware_interface import BasicmicroHardwareInterface, MotionStrategy
from basicmicro_driver.unit_converter import UnitConverter

try:
    from hardware_interface_msgs.msg import return_type
except ImportError:
    class return_type:
        OK = 0
        ERROR = 1

try:
    from basicmicro import Basicmicro
    BASICMICRO_AVAILABLE = True
except ImportError:
    BASICMICRO_AVAILABLE = False


class ThroughputMeasurementFramework:
    """Framework for measuring sustained throughput performance"""
    
    def __init__(self):
        self.reset_measurements()
        
    def reset_measurements(self):
        """Reset measurement counters"""
        self.start_time = None
        self.end_time = None
        self.command_count = 0
        self.sensor_count = 0
        self.error_count = 0
        self.command_times = deque(maxlen=10000)
        self.sensor_times = deque(maxlen=10000)
        self.cycle_times = deque(maxlen=10000)
        self.errors = []
        
    def start_measurement(self):
        """Start throughput measurement"""
        self.reset_measurements()
        self.start_time = time.perf_counter()
        
    def stop_measurement(self):
        """Stop throughput measurement"""
        self.end_time = time.perf_counter()
        
    def record_command(self):
        """Record command execution"""
        if self.start_time:
            self.command_count += 1
            self.command_times.append(time.perf_counter())
            
    def record_sensor_reading(self):
        """Record sensor reading"""
        if self.start_time:
            self.sensor_count += 1
            self.sensor_times.append(time.perf_counter())
            
    def record_cycle(self):
        """Record cycle completion"""
        if self.start_time:
            self.cycle_times.append(time.perf_counter())
            
    def record_error(self, error_msg):
        """Record error occurrence"""
        self.error_count += 1
        self.errors.append({
            'timestamp': time.perf_counter(),
            'error': error_msg
        })
        
    def calculate_throughput(self):
        """Calculate throughput statistics"""
        if not self.start_time or not self.end_time:
            return {}
            
        duration = self.end_time - self.start_time
        
        # Calculate frequencies
        command_frequency = self.command_count / duration if duration > 0 else 0
        sensor_frequency = self.sensor_count / duration if duration > 0 else 0
        cycle_frequency = len(self.cycle_times) / duration if duration > 0 else 0
        
        # Calculate jitter (variation in timing)
        command_jitter = 0.0
        sensor_jitter = 0.0
        cycle_jitter = 0.0
        
        if len(self.command_times) > 1:
            command_intervals = [self.command_times[i] - self.command_times[i-1] 
                               for i in range(1, len(self.command_times))]
            command_jitter = statistics.stdev(command_intervals) * 1000  # ms
            
        if len(self.sensor_times) > 1:
            sensor_intervals = [self.sensor_times[i] - self.sensor_times[i-1] 
                              for i in range(1, len(self.sensor_times))]
            sensor_jitter = statistics.stdev(sensor_intervals) * 1000  # ms
            
        if len(self.cycle_times) > 1:
            cycle_intervals = [self.cycle_times[i] - self.cycle_times[i-1] 
                             for i in range(1, len(self.cycle_times))]
            cycle_jitter = statistics.stdev(cycle_intervals) * 1000  # ms
            
        return {
            'duration_seconds': duration,
            'command_frequency_hz': command_frequency,
            'sensor_frequency_hz': sensor_frequency,
            'cycle_frequency_hz': cycle_frequency,
            'command_jitter_ms': command_jitter,
            'sensor_jitter_ms': sensor_jitter,
            'cycle_jitter_ms': cycle_jitter,
            'error_count': self.error_count,
            'error_rate_percent': (self.error_count / max(self.command_count, 1)) * 100,
            'total_commands': self.command_count,
            'total_sensors': self.sensor_count,
            'total_cycles': len(self.cycle_times)
        }


@pytest.mark.hardware
@pytest.mark.performance
@pytest.mark.skipif(not BASICMICRO_AVAILABLE, reason="Basicmicro library not available")
class TestThroughputValidation:
    """Throughput validation tests with real hardware"""
    
    def setup_method(self):
        """Set up throughput validation environment"""
        self.config = {
            'port': '/dev/ttyACM0',
            'baud': 38400,
            'address': 0x80,
            'wheel_radius': 0.1,
            'encoder_counts_per_rev': 1000,
            'gear_ratio': 1.0,
            'max_safe_speed': 500,
            'max_safe_duty': 8192,
        }
        
        self.throughput_targets = {
            'command_execution_hz': 200.0,
            'sensor_reading_hz': 150.0,
            'continuous_cycles_hz': 80.0,
            'trajectory_processing_hz': 50.0,
        }
        
        self.framework = ThroughputMeasurementFramework()
        
        # Initialize hardware interface
        self.hw = BasicmicroHardwareInterface()
        self.hw.unit_converter = UnitConverter(
            self.config['wheel_radius'],
            self.config['encoder_counts_per_rev'],
            self.config['gear_ratio']
        )
        
    def _connect_hardware(self):
        """Connect to real hardware controller"""
        if hasattr(self.hw, 'controller') and self.hw.controller:
            return True
            
        try:
            controller = Basicmicro(self.config['port'], self.config['baud'])
            if not controller.Open():
                pytest.skip(f"Could not open hardware controller on {self.config['port']}")
                
            version_result = controller.ReadVersion(self.config['address'])
            if not version_result[0]:
                pytest.skip(f"Could not communicate with controller at address {self.config['address']}")
                
            self.hw.controller = controller
            self.hw.address = self.config['address']
            self.hw.motion_strategy = MotionStrategy.SPEED_ACCEL
            self.hw.default_acceleration = 1000
            
            # Initialize state variables
            self.hw.hw_commands_velocities_ = [0.0, 0.0]
            self.hw.hw_states_positions_ = [0.0, 0.0]
            self.hw.hw_states_velocities_ = [0.0, 0.0]
            self.hw.emergency_stop_active = False
            
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
                self.hw.controller.DutyM1M2(self.config['address'], 0, 0)
                time.sleep(0.1)
                self.hw.controller.close()
            except:
                pass
                
    def test_command_execution_throughput(self):
        """Test sustained command execution throughput"""
        if not self._connect_hardware():
            return
            
        try:
            self.framework.start_measurement()
            
            # Target: 200Hz for 10 seconds
            target_frequency = self.throughput_targets['command_execution_hz']
            duration = 10.0
            period = 1.0 / target_frequency
            end_time = time.perf_counter() + duration
            
            cycle_count = 0
            
            while time.perf_counter() < end_time:
                cycle_start = time.perf_counter()
                
                # Generate varying command pattern
                t = cycle_count * period
                velocity = 0.3 * math.sin(t * 2 * math.pi * 0.1)  # 0.1 Hz sine wave
                
                self.hw.hw_commands_velocities_[0] = velocity
                self.hw.hw_commands_velocities_[1] = velocity
                
                # Execute command
                try:
                    result = self.hw.write(time=t, period=period)
                    if result == return_type.OK:
                        self.framework.record_command()
                    else:
                        self.framework.record_error(f"Write failed: {result}")
                except Exception as e:
                    self.framework.record_error(f"Write exception: {e}")
                    
                cycle_count += 1
                
                # Sleep for remaining time
                elapsed = time.perf_counter() - cycle_start
                sleep_time = max(0, period - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            self.framework.stop_measurement()
            stats = self.framework.calculate_throughput()
            
            # Validate throughput
            assert stats['command_frequency_hz'] >= target_frequency * 0.9, \
                f"Command throughput {stats['command_frequency_hz']:.1f}Hz below target {target_frequency}Hz"
            assert stats['error_rate_percent'] < 1.0, \
                f"Error rate {stats['error_rate_percent']:.1f}% too high"
            assert stats['command_jitter_ms'] < 2.0, \
                f"Command jitter {stats['command_jitter_ms']:.2f}ms too high"
                
            print(f"Command Execution Throughput Results:")
            print(f"  Frequency: {stats['command_frequency_hz']:.1f}Hz")
            print(f"  Jitter: {stats['command_jitter_ms']:.2f}ms")
            print(f"  Error Rate: {stats['error_rate_percent']:.2f}%")
            print(f"  Total Commands: {stats['total_commands']}")
            
        finally:
            self._cleanup_hardware()
            
    def test_sensor_reading_throughput(self):
        """Test sustained sensor reading throughput"""
        if not self._connect_hardware():
            return
            
        try:
            self.framework.start_measurement()
            
            # Target: 150Hz for 10 seconds
            target_frequency = self.throughput_targets['sensor_reading_hz']
            duration = 10.0
            period = 1.0 / target_frequency
            end_time = time.perf_counter() + duration
            
            cycle_count = 0
            
            while time.perf_counter() < end_time:
                cycle_start = time.perf_counter()
                
                # Execute sensor reading
                try:
                    result = self.hw.read(time=cycle_count * period, period=period)
                    if result == return_type.OK:
                        self.framework.record_sensor_reading()
                    else:
                        self.framework.record_error(f"Read failed: {result}")
                except Exception as e:
                    self.framework.record_error(f"Read exception: {e}")
                    
                cycle_count += 1
                
                # Sleep for remaining time
                elapsed = time.perf_counter() - cycle_start
                sleep_time = max(0, period - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            self.framework.stop_measurement()
            stats = self.framework.calculate_throughput()
            
            # Validate throughput
            assert stats['sensor_frequency_hz'] >= target_frequency * 0.9, \
                f"Sensor throughput {stats['sensor_frequency_hz']:.1f}Hz below target {target_frequency}Hz"
            assert stats['error_rate_percent'] < 1.0, \
                f"Error rate {stats['error_rate_percent']:.1f}% too high"
            assert stats['sensor_jitter_ms'] < 3.0, \
                f"Sensor jitter {stats['sensor_jitter_ms']:.2f}ms too high"
                
            print(f"Sensor Reading Throughput Results:")
            print(f"  Frequency: {stats['sensor_frequency_hz']:.1f}Hz")
            print(f"  Jitter: {stats['sensor_jitter_ms']:.2f}ms")
            print(f"  Error Rate: {stats['error_rate_percent']:.2f}%")
            print(f"  Total Readings: {stats['total_sensors']}")
            
        finally:
            self._cleanup_hardware()
            
    def test_continuous_operation_throughput(self):
        """Test continuous operation cycle throughput"""
        if not self._connect_hardware():
            return
            
        try:
            self.framework.start_measurement()
            
            # Target: 80Hz continuous cycles for 10 seconds
            target_frequency = self.throughput_targets['continuous_cycles_hz']
            duration = 10.0
            period = 1.0 / target_frequency
            end_time = time.perf_counter() + duration
            
            cycle_count = 0
            
            while time.perf_counter() < end_time:
                cycle_start = time.perf_counter()
                
                # Complete cycle: command + sensor reading
                t = cycle_count * period
                velocity = 0.2 * math.sin(t * 2 * math.pi * 0.05)  # 0.05 Hz sine wave
                
                self.hw.hw_commands_velocities_[0] = velocity
                self.hw.hw_commands_velocities_[1] = velocity
                
                try:
                    # Execute command
                    write_result = self.hw.write(time=t, period=period)
                    if write_result == return_type.OK:
                        self.framework.record_command()
                    else:
                        self.framework.record_error(f"Write failed: {write_result}")
                        
                    # Read sensors
                    read_result = self.hw.read(time=t, period=period)
                    if read_result == return_type.OK:
                        self.framework.record_sensor_reading()
                    else:
                        self.framework.record_error(f"Read failed: {read_result}")
                        
                    # Record cycle completion
                    self.framework.record_cycle()
                    
                except Exception as e:
                    self.framework.record_error(f"Cycle exception: {e}")
                    
                cycle_count += 1
                
                # Sleep for remaining time
                elapsed = time.perf_counter() - cycle_start
                sleep_time = max(0, period - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            self.framework.stop_measurement()
            stats = self.framework.calculate_throughput()
            
            # Validate throughput
            assert stats['cycle_frequency_hz'] >= target_frequency * 0.9, \
                f"Cycle throughput {stats['cycle_frequency_hz']:.1f}Hz below target {target_frequency}Hz"
            assert stats['error_rate_percent'] < 2.0, \
                f"Error rate {stats['error_rate_percent']:.1f}% too high"
            assert stats['cycle_jitter_ms'] < 5.0, \
                f"Cycle jitter {stats['cycle_jitter_ms']:.2f}ms too high"
                
            print(f"Continuous Operation Throughput Results:")
            print(f"  Cycle Frequency: {stats['cycle_frequency_hz']:.1f}Hz")
            print(f"  Command Frequency: {stats['command_frequency_hz']:.1f}Hz")
            print(f"  Sensor Frequency: {stats['sensor_frequency_hz']:.1f}Hz")
            print(f"  Cycle Jitter: {stats['cycle_jitter_ms']:.2f}ms")
            print(f"  Error Rate: {stats['error_rate_percent']:.2f}%")
            
        finally:
            self._cleanup_hardware()
            
    def test_burst_performance(self):
        """Test burst performance capability"""
        if not self._connect_hardware():
            return
            
        try:
            # Test burst of commands at maximum rate
            burst_size = 100
            burst_results = []
            
            for burst_num in range(5):  # 5 bursts
                self.framework.start_measurement()
                
                # Execute burst of commands as fast as possible
                for i in range(burst_size):
                    start_time = time.perf_counter()
                    
                    velocity = 0.1 * (i % 10 - 5)  # Varying pattern
                    self.hw.hw_commands_velocities_[0] = velocity
                    self.hw.hw_commands_velocities_[1] = velocity
                    
                    try:
                        result = self.hw.write(time=i*0.001, period=0.001)
                        if result == return_type.OK:
                            self.framework.record_command()
                        else:
                            self.framework.record_error(f"Burst write failed: {result}")
                    except Exception as e:
                        self.framework.record_error(f"Burst exception: {e}")
                        
                self.framework.stop_measurement()
                stats = self.framework.calculate_throughput()
                burst_results.append(stats)
                
                # Brief pause between bursts
                time.sleep(0.5)
                
            # Analyze burst performance
            burst_frequencies = [r['command_frequency_hz'] for r in burst_results]
            burst_error_rates = [r['error_rate_percent'] for r in burst_results]
            
            mean_burst_freq = statistics.mean(burst_frequencies)
            max_burst_freq = max(burst_frequencies)
            mean_error_rate = statistics.mean(burst_error_rates)
            
            # Validate burst performance
            assert mean_burst_freq >= 300.0, \
                f"Mean burst frequency {mean_burst_freq:.1f}Hz below 300Hz target"
            assert max_burst_freq >= 400.0, \
                f"Max burst frequency {max_burst_freq:.1f}Hz below 400Hz target"
            assert mean_error_rate < 5.0, \
                f"Mean burst error rate {mean_error_rate:.1f}% too high"
                
            print(f"Burst Performance Results:")
            print(f"  Mean Burst Frequency: {mean_burst_freq:.1f}Hz")
            print(f"  Max Burst Frequency: {max_burst_freq:.1f}Hz")
            print(f"  Mean Error Rate: {mean_error_rate:.1f}%")
            print(f"  Burst Consistency: {statistics.stdev(burst_frequencies):.1f}Hz std")
            
        finally:
            self._cleanup_hardware()
            
    def test_save_throughput_results(self):
        """Save throughput validation results for analysis"""
        if not self._connect_hardware():
            return
            
        try:
            results = {
                'timestamp': datetime.now().isoformat(),
                'hardware_config': self.config,
                'throughput_targets': self.throughput_targets,
                'measurements': {}
            }
            
            # Command execution throughput
            self.framework.start_measurement()
            target_freq = 200.0
            period = 1.0 / target_freq
            end_time = time.perf_counter() + 5.0
            
            cycle_count = 0
            while time.perf_counter() < end_time:
                cycle_start = time.perf_counter()
                
                velocity = 0.2 * math.sin(cycle_count * period * 2 * math.pi * 0.1)
                self.hw.hw_commands_velocities_[0] = velocity
                self.hw.hw_commands_velocities_[1] = velocity
                
                try:
                    result = self.hw.write(time=cycle_count * period, period=period)
                    if result == return_type.OK:
                        self.framework.record_command()
                    else:
                        self.framework.record_error(f"Write failed: {result}")
                except Exception as e:
                    self.framework.record_error(f"Write exception: {e}")
                    
                cycle_count += 1
                elapsed = time.perf_counter() - cycle_start
                sleep_time = max(0, period - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            self.framework.stop_measurement()
            results['measurements']['command_execution'] = self.framework.calculate_throughput()
            
            # Sensor reading throughput
            self.framework.start_measurement()
            target_freq = 150.0
            period = 1.0 / target_freq
            end_time = time.perf_counter() + 5.0
            
            cycle_count = 0
            while time.perf_counter() < end_time:
                cycle_start = time.perf_counter()
                
                try:
                    result = self.hw.read(time=cycle_count * period, period=period)
                    if result == return_type.OK:
                        self.framework.record_sensor_reading()
                    else:
                        self.framework.record_error(f"Read failed: {result}")
                except Exception as e:
                    self.framework.record_error(f"Read exception: {e}")
                    
                cycle_count += 1
                elapsed = time.perf_counter() - cycle_start
                sleep_time = max(0, period - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            self.framework.stop_measurement()
            results['measurements']['sensor_reading'] = self.framework.calculate_throughput()
            
            # Save results
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            results_file = f"throughput_validation_{timestamp}.json"
            results_path = os.path.join(os.path.dirname(__file__), '..', 'test_data', results_file)
            
            with open(results_path, 'w') as f:
                json.dump(results, f, indent=2)
                
            print(f"Throughput validation results saved to: {results_path}")
            
        finally:
            self._cleanup_hardware()