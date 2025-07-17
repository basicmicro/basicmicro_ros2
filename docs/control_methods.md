# Motor Control Methods Guide

This guide explains the different methods for controlling motors with the Basicmicro ROS2 driver and when to use each approach.

## Understanding Control Methods

### 1. Testing Commands (`--once`)
**Purpose**: Quick testing and verification  
**Behavior**: Sends single command and exits immediately  
**Delay**: Normal - includes ROS2 node startup time (~1-2 seconds)

```bash
# Testing command - has startup delay
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

**Why there's a delay:**
- ROS2 node startup time
- Topic discovery and subscription matching
- Single command execution then shutdown
- **This is normal and expected behavior**

### 2. Continuous Control (Real-time)
**Purpose**: Real-time robot control  
**Behavior**: Maintains persistent connection, no startup delays  
**Delay**: Minimal - direct communication with running node

```bash
# Continuous control - no startup delay
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```

### 3. Programmatic Control (Recommended)
**Purpose**: Application integration and autonomous control  
**Behavior**: Direct ROS2 node communication  
**Delay**: Minimal - optimized for real-time performance

## When to Use Each Method

### ✅ Use `--once` for:
- **Testing hardware connectivity**
- **Verifying motor response**
- **Debugging motor configuration**
- **One-time movements**

### ✅ Use continuous control for:
- **Real-time teleoperation**
- **Joystick/gamepad control**
- **Sustained movement commands**
- **Performance testing**

### ✅ Use programmatic control for:
- **Autonomous navigation**
- **Application integration**
- **Custom control algorithms**
- **Production robotics**

## Command Delay Analysis

### Expected Startup Times:
- `--once` command: **1-2 seconds** (normal)
- Continuous control: **Initial 1-2 seconds, then immediate**
- Programmatic control: **Immediate after initialization**

### Why `--once` Has Delays:
1. **ROS2 Node Startup**: ~500ms
2. **Topic Discovery**: ~500ms  
3. **Subscription Matching**: ~200ms
4. **Command Execution**: ~10ms
5. **Node Shutdown**: ~200ms

**Total**: ~1.4 seconds (this is normal ROS2 behavior)

### Eliminating Delays:
- **Keep node running**: Use continuous or programmatic control
- **Persistent connections**: Avoid repeated `--once` commands
- **Proper real-time methods**: Follow patterns below

## Real-Time Control Examples

### Continuous Movement Control
```bash
# Start continuous control (no delay after startup)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10

# Stop motors
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --rate 1
```

### Programmatic Control (Python)
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Wait for publisher to be ready
        time.sleep(1.0)
        
    def move_forward(self, speed=0.1, duration=2.0):
        """Move forward for specified duration - NO DELAY"""
        msg = Twist()
        msg.linear.x = speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(msg)
            time.sleep(0.1)  # 10Hz control rate
            
        # Stop motors
        self.stop_motors()
        
    def stop_motors(self):
        """Stop motors immediately - NO DELAY"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    controller = MotorController()
    
    try:
        # Real-time control with no command delays
        controller.move_forward(0.1, 2.0)  # Move forward 0.1 m/s for 2 seconds
        time.sleep(1.0)
        controller.move_forward(-0.1, 1.0)  # Move backward 0.1 m/s for 1 second
        
    except KeyboardInterrupt:
        controller.stop_motors()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### For Real-Time Applications:
1. **Use programmatic control** (Python/C++ nodes)
2. **Maintain persistent connections** (avoid `--once`)
3. **Optimize control rates** (10-50Hz typical)
4. **Use proper ROS2 QoS settings**

### For Testing:
1. **Accept `--once` delays** (normal behavior)
2. **Use continuous commands** for extended testing
3. **Pre-warm the system** with initial commands

## Connection Health Monitoring

The driver includes CAN-like error rate monitoring:
- **Connection States**: "good", "degraded", "failed"
- **Error Recovery**: Automatic healing over time
- **Health Checks**: Periodic controller verification
- **User Notifications**: Clear status messages

Monitor connection health:
```bash
# Check connection status
ros2 topic echo /basicmicro/status

# Monitor diagnostics
ros2 topic echo /diagnostics
```

## Common Issues and Solutions

### "Waiting for at least 1 matching subscription(s)..."
**Cause**: Normal ROS2 topic discovery  
**Solution**: Wait 1-2 seconds or use continuous control

### Intermittent Command Delays
**Cause**: Connection instability  
**Solution**: Monitor `/diagnostics` for connection health

### Motors Don't Respond
**Cause**: Driver not running or connection failed  
**Solution**: Check driver status and hardware connection

## Best Practices

1. **Choose the right method** for your use case
2. **Monitor connection health** during operation
3. **Use safety timeouts** in programmatic control
4. **Test with `--once`**, deploy with programmatic control
5. **Implement proper error handling** in applications

---

**Remember**: Command delays with `--once` are normal ROS2 behavior. Use continuous or programmatic control for real-time applications.