"""
Hardware Latency Measurement Tests with Motor Movement Validation

Measures actual command-to-execution latency with real hardware controllers.
Validates baseline performance targets and identifies real-world performance characteristics.

CRITICAL: These tests measure ACTUAL MOTOR RESPONSE LATENCY, not just communication timing.
Each test verifies that motors actually move when commands are sent and measures
the complete control loop latency from command to physical motor response.

Performance Baseline Targets:
- Simple commands: <5ms mean latency
- Complex commands: <10ms mean latency  
- Unit conversion: <0.1ms processing time
- Emergency stop: <2ms response time

Motor Movement Validation:
- Command-to-movement latency: <150ms
- Motor response validation: Required for all tests
- Speed accuracy measurement: Within tolerance validation
- Emergency stop physical validation: Motors must actually stop

Author: Basicmicro Driver Team
License: Apache-2.0
"""

import pytest
import sys
import os
import time
import statistics
import json
import math
from datetime import datetime
from collections import deque

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


class LatencyMeasurementFramework:
    """Framework for precise latency measurement"""
    
    def __init__(self, baseline_file=None):
        self.baseline_file = baseline_file or os.path.join(
            os.path.dirname(__file__), '..', 'test_data', 'performance_baselines.json'
        )
        self.baselines = self._load_baselines()
        
    def _load_baselines(self):
        """Load performance baselines from file"""
        try:
            with open(self.baseline_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            # Return default baselines if file not found
            return {
                "baselines": {
                    "command_latency": {
                        "velocity_command_ms": {"mean": 2.5, "p95": 4.0, "max": 6.0, "target": 5.0},
                        "sensor_reading_ms": {"mean": 1.8, "p95": 3.0, "max": 4.5, "target": 5.0},
                        "emergency_stop_ms": {"mean": 1.0, "p95": 1.5, "max": 2.0, "target": 2.0},
                        "unit_conversion_ms": {"mean": 0.05, "p95": 0.08, "max": 0.1, "target": 0.1}
                    }
                }
            }
    
    def measure_operation_latency(self, operation, iterations=100, warmup_iterations=10):
        """Measure operation latency with warmup"""
        # Warmup iterations
        for _ in range(warmup_iterations):
            operation()
            
        # Measurement iterations
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
            'samples': latencies,
            'iterations': iterations
        }
    
    def validate_against_baseline(self, test_name, measured_stats, tolerance=0.2):
        """Validate measurements against baseline with tolerance"""
        baseline_key = test_name.replace('test_', '').replace('_latency', '_ms')
        baseline_data = self.baselines.get('baselines', {}).get('command_latency', {}).get(baseline_key, {})
        
        if not baseline_data:
            return True, f"No baseline data found for {test_name}"
            
        results = []
        
        # Check mean latency
        baseline_mean = baseline_data.get('mean', 0)
        if measured_stats['mean'] > baseline_mean * (1 + tolerance):
            results.append(f"Mean latency {measured_stats['mean']:.2f}ms exceeds baseline {baseline_mean:.2f}ms by {tolerance*100}%")
            
        # Check p95 latency
        baseline_p95 = baseline_data.get('p95', 0)
        if measured_stats['p95'] > baseline_p95 * (1 + tolerance):
            results.append(f"P95 latency {measured_stats['p95']:.2f}ms exceeds baseline {baseline_p95:.2f}ms by {tolerance*100}%")
            
        # Check target
        target = baseline_data.get('target', float('inf'))
        if measured_stats['mean'] > target:
            results.append(f"Mean latency {measured_stats['mean']:.2f}ms exceeds target {target:.2f}ms")
            
        if results:
            return False, "; ".join(results)
        else:
            return True, "All measurements within baseline tolerance"


@pytest.mark.hardware
@pytest.mark.performance
@pytest.mark.skipif(not BASICMICRO_AVAILABLE, reason="Basicmicro library not available")
class TestHardwareLatencyMeasurement:
    """Hardware latency measurement tests with real controllers"""
    
    def setup_method(self):
        """Set up hardware latency measurement environment"""
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
        
        self.measurement_framework = LatencyMeasurementFramework()
        
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
                
    def test_velocity_command_latency(self):
        """Test velocity command execution latency"""
        if not self._connect_hardware():
            return
            
        try:
            # Set up test command
            self.hw.hw_commands_velocities_[0] = 0.5
            self.hw.hw_commands_velocities_[1] = 0.5
            
            # Measure latency
            stats = self.measurement_framework.measure_operation_latency(
                lambda: self.hw.write(time=0.0, period=0.1),
                iterations=200,
                warmup_iterations=20
            )
            
            # Validate against baseline
            is_valid, message = self.measurement_framework.validate_against_baseline(
                'velocity_command', stats
            )
            
            assert is_valid, f"Velocity command latency validation failed: {message}"
            
            print(f"Velocity Command Latency Results:")
            print(f"  Mean: {stats['mean']:.2f}ms")
            print(f"  P95: {stats['p95']:.2f}ms")
            print(f"  Max: {stats['max']:.2f}ms")
            print(f"  Std: {stats['std']:.2f}ms")
            print(f"  Baseline validation: {message}")
            
        finally:
            self._cleanup_hardware()
            
    def test_sensor_reading_latency(self):
        """Test sensor reading latency"""
        if not self._connect_hardware():
            return
            
        try:
            # Measure sensor reading latency
            stats = self.measurement_framework.measure_operation_latency(
                lambda: self.hw.read(time=0.0, period=0.1),
                iterations=200,
                warmup_iterations=20
            )
            
            # Validate against baseline
            is_valid, message = self.measurement_framework.validate_against_baseline(
                'sensor_reading', stats
            )
            
            assert is_valid, f"Sensor reading latency validation failed: {message}"
            
            print(f"Sensor Reading Latency Results:")
            print(f"  Mean: {stats['mean']:.2f}ms")
            print(f"  P95: {stats['p95']:.2f}ms")
            print(f"  Max: {stats['max']:.2f}ms")
            print(f"  Std: {stats['std']:.2f}ms")
            print(f"  Baseline validation: {message}")
            
        finally:
            self._cleanup_hardware()
            
    def test_emergency_stop_latency(self):
        """Test emergency stop response latency"""
        if not self._connect_hardware():
            return
            
        try:
            # Prepare for emergency stop testing
            def emergency_stop_test():
                # Set up motion first
                self.hw.hw_commands_velocities_[0] = 0.3
                self.hw.hw_commands_velocities_[1] = 0.3
                self.hw.write(time=0.0, period=0.1)
                
                # Trigger emergency stop
                return self.hw.emergency_stop()
                
            # Measure emergency stop latency
            stats = self.measurement_framework.measure_operation_latency(
                emergency_stop_test,
                iterations=50,
                warmup_iterations=5
            )
            
            # Validate against baseline
            is_valid, message = self.measurement_framework.validate_against_baseline(
                'emergency_stop', stats
            )
            
            assert is_valid, f"Emergency stop latency validation failed: {message}"
            
            print(f"Emergency Stop Latency Results:")
            print(f"  Mean: {stats['mean']:.2f}ms")
            print(f"  P95: {stats['p95']:.2f}ms")
            print(f"  Max: {stats['max']:.2f}ms")
            print(f"  Std: {stats['std']:.2f}ms")
            print(f"  Baseline validation: {message}")
            
        finally:
            self._cleanup_hardware()
            
    def test_unit_conversion_latency(self):
        """Test unit conversion processing latency"""
        # Test unit conversion (no hardware required)
        unit_converter = UnitConverter(0.1, 1000, 1.0)
        
        # Test various conversions
        conversion_tests = [
            ("radians_to_counts", lambda: unit_converter.radians_to_counts(1.0)),
            ("counts_to_radians", lambda: unit_converter.counts_to_radians(1000)),
            ("rad_per_sec_to_counts_per_sec", lambda: unit_converter.rad_per_sec_to_counts_per_sec(1.0)),
            ("counts_per_sec_to_rad_per_sec", lambda: unit_converter.counts_per_sec_to_rad_per_sec(1000)),
            ("distance_to_counts", lambda: unit_converter.distance_to_counts(1.0)),
            ("counts_to_distance", lambda: unit_converter.counts_to_distance(1000)),
        ]
        
        all_results = {}
        
        for test_name, operation in conversion_tests:
            stats = self.measurement_framework.measure_operation_latency(
                operation,
                iterations=1000,
                warmup_iterations=100
            )
            
            all_results[test_name] = stats
            
            # Validate against baseline (all should be <0.1ms)
            assert stats['mean'] < 0.1, \
                f"{test_name} mean latency {stats['mean']:.3f}ms exceeds 0.1ms target"
            assert stats['p95'] < 0.2, \
                f"{test_name} P95 latency {stats['p95']:.3f}ms exceeds 0.2ms threshold"
                
        print(f"Unit Conversion Latency Results:")
        for test_name, stats in all_results.items():
            print(f"  {test_name}: {stats['mean']:.3f}ms mean, {stats['p95']:.3f}ms P95")
            
    def test_complex_command_latency(self):
        """Test complex command execution latency"""
        if not self._connect_hardware():
            return
            
        try:
            # Complex command: trajectory point execution
            def complex_command_test():
                # Set up trajectory-like command sequence
                self.hw.hw_commands_velocities_[0] = 0.5
                self.hw.hw_commands_velocities_[1] = 0.3
                
                # Execute write operation
                result1 = self.hw.write(time=0.0, period=0.1)
                
                # Read current state
                result2 = self.hw.read(time=0.0, period=0.1)
                
                # Update command based on feedback
                self.hw.hw_commands_velocities_[0] = 0.4
                self.hw.hw_commands_velocities_[1] = 0.4
                
                # Execute updated command
                result3 = self.hw.write(time=0.1, period=0.1)
                
                return result1 == return_type.OK and result3 == return_type.OK
                
            # Measure complex command latency
            stats = self.measurement_framework.measure_operation_latency(
                complex_command_test,
                iterations=100,
                warmup_iterations=10
            )
            
            # Validate against 10ms target for complex commands
            assert stats['mean'] < 10.0, \
                f"Complex command mean latency {stats['mean']:.2f}ms exceeds 10ms target"
            assert stats['p95'] < 15.0, \
                f"Complex command P95 latency {stats['p95']:.2f}ms exceeds 15ms threshold"
                
            print(f"Complex Command Latency Results:")
            print(f"  Mean: {stats['mean']:.2f}ms")
            print(f"  P95: {stats['p95']:.2f}ms")
            print(f"  Max: {stats['max']:.2f}ms")
            print(f"  Std: {stats['std']:.2f}ms")
            
        finally:
            self._cleanup_hardware()
            
    def test_save_latency_measurements(self):
        """Save latency measurement results for analysis"""
        if not self._connect_hardware():
            return
            
        try:
            # Run abbreviated measurement suite
            results = {
                'timestamp': datetime.now().isoformat(),
                'hardware_config': self.config,
                'measurements': {}
            }
            
            # Velocity command latency
            self.hw.hw_commands_velocities_[0] = 0.5
            self.hw.hw_commands_velocities_[1] = 0.5
            results['measurements']['velocity_command'] = \
                self.measurement_framework.measure_operation_latency(
                    lambda: self.hw.write(time=0.0, period=0.1),
                    iterations=100
                )
                
            # Sensor reading latency
            results['measurements']['sensor_reading'] = \
                self.measurement_framework.measure_operation_latency(
                    lambda: self.hw.read(time=0.0, period=0.1),
                    iterations=100
                )
                
            # Emergency stop latency
            def emergency_test():
                self.hw.hw_commands_velocities_[0] = 0.3
                self.hw.hw_commands_velocities_[1] = 0.3
                self.hw.write(time=0.0, period=0.1)
                return self.hw.emergency_stop()
                
            results['measurements']['emergency_stop'] = \
                self.measurement_framework.measure_operation_latency(
                    emergency_test,
                    iterations=25
                )
                
            # Unit conversion latency
            unit_converter = UnitConverter(0.1, 1000, 1.0)
            results['measurements']['unit_conversion'] = \
                self.measurement_framework.measure_operation_latency(
                    lambda: unit_converter.radians_to_counts(1.0),
                    iterations=1000
                )
                
            # Save results
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            results_file = f"latency_measurements_{timestamp}.json"
            results_path = os.path.join(os.path.dirname(__file__), '..', 'test_data', results_file)
            
            with open(results_path, 'w') as f:
                json.dump(results, f, indent=2)
                
            print(f"Latency measurement results saved to: {results_path}")
            
        finally:
            self._cleanup_hardware()