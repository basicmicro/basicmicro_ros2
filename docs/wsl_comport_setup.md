# WSL Comport Setup Guide

Complete guide for setting up USB serial communication between Windows and WSL for Basicmicro controllers.

## Overview

Windows Subsystem for Linux (WSL) doesn't automatically forward USB serial devices. This guide shows how to properly configure USB serial forwarding for Basicmicro motor controllers.

## Prerequisites

- **Windows 10/11** with WSL2 installed
- **WSL2 Ubuntu 22.04** (recommended)
- **Administrative access** on Windows
- **Basicmicro motor controller** connected via USB

## Method 1: USB/IP Forwarding (Recommended)

### Windows Setup

#### 1. Install usbipd-win
```powershell
# Run PowerShell as Administrator
winget install --interactive --exact dorssel.usbipd-win
```

Or download from: https://github.com/dorssel/usbipd-win/releases

#### 2. List USB Devices
```powershell
# List all USB devices
usbipd list
```

Look for your Basicmicro controller, typically showing as:
```
BUSID  VID:PID    DEVICE                                                        STATE
2-3    03eb:2404  USB Serial Device (COM3)                                     Not shared
```

#### 3. Share the USB Device
```powershell
# Share the device (replace 2-3 with your BUSID)
usbipd bind --busid 2-3

# Verify it's shared
usbipd list
```

Should now show `Shared` state.

### WSL Setup

#### 1. Install USB/IP Tools in WSL
```bash
# Update package list
sudo apt update

# Install USB/IP tools
sudo apt install linux-tools-virtual hwdata -y

# Update udev rules
sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20
```

#### 2. Attach USB Device
```powershell
# From Windows PowerShell (as Administrator)
# Attach to default WSL distribution
usbipd attach --wsl --busid 2-3

# Or attach to specific distribution
usbipd attach --wsl --busid 2-3 --distribution Ubuntu-22.04
```

#### 3. Verify Device in WSL
```bash
# Check for the device
lsusb

# Look for serial devices
ls /dev/ttyACM* /dev/ttyUSB*
```

Should show device like `/dev/ttyACM0`.

## Method 2: COM Port Forwarding (Alternative)

### Using socat (Network Serial Bridge)

#### Windows Setup
```powershell
# Install socat for Windows
# Download from: https://sourceforge.net/projects/unix-utils/files/socat/

# Forward COM port to TCP (run in PowerShell)
socat TCP-LISTEN:9999,reuseaddr,fork OPEN:COM3,raw,echo=0
```

#### WSL Setup
```bash
# Install socat in WSL
sudo apt install socat -y

# Create virtual serial port from TCP connection
sudo socat PTY,link=/dev/ttyVirtualCOM,raw,echo=0 TCP:localhost:9999 &

# Use /dev/ttyVirtualCOM as your serial port
```

## Configuration and Testing

### 1. Set Device Permissions

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Set temporary permissions
sudo chmod 666 /dev/ttyACM0

# Create udev rule for persistent permissions
sudo tee /etc/udev/rules.d/99-basicmicro-wsl.rules << 'EOF'
# Basicmicro controllers in WSL
SUBSYSTEM=="tty", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2404", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2. Test Serial Communication

```bash
# Basic connectivity test
python3 -c "
import serial
try:
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
    print('✅ Serial port opened successfully')
    ser.close()
except Exception as e:
    print(f'❌ Error: {e}')
"
```

### 3. Test Basicmicro Controller

```bash
# Test controller communication
python3 -c "
from basicmicro import Basicmicro
controller = Basicmicro('/dev/ttyACM0', 38400)
success = controller.Open()
if success:
    version = controller.ReadVersion(0x80)
    if version[0]:
        print(f'✅ Controller connected: {version[1].strip()}')
    else:
        print('❌ Version read failed')
    controller.close()
else:
    print('❌ Failed to open controller')
"
```

## Automation Scripts

### Auto-Attach Script (PowerShell)

Create `attach-basicmicro.ps1`:

```powershell
# Auto-attach Basicmicro controller to WSL
param(
    [string]$Distribution = "Ubuntu-22.04"
)

# Find Basicmicro device
$devices = usbipd list | Select-String "03eb:2404|Roboclaw"
if ($devices) {
    $busid = ($devices -split '\s+')[0]
    Write-Host "Found Basicmicro device at BUSID: $busid"
    
    # Bind if not already bound
    usbipd bind --busid $busid
    
    # Attach to WSL
    usbipd attach --wsl --busid $busid --distribution $Distribution
    
    Write-Host "Device attached to $Distribution"
} else {
    Write-Host "No Basicmicro device found"
}
```

### Auto-Detach Script (PowerShell)

Create `detach-basicmicro.ps1`:

```powershell
# Detach all USB devices from WSL
usbipd detach --all
Write-Host "All USB devices detached from WSL"
```

### WSL Check Script (Bash)

Create `check-basicmicro.sh`:

```bash
#!/bin/bash
# Check Basicmicro controller status in WSL

echo "=== USB Devices ==="
lsusb | grep -i "03eb\|roboclaw\|basicmicro" || echo "No Basicmicro USB devices found"

echo -e "\n=== Serial Devices ==="
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial devices found"

echo -e "\n=== Permission Check ==="
if [ -e "/dev/ttyACM0" ]; then
    ls -la /dev/ttyACM0
    groups | grep -q dialout && echo "✅ User in dialout group" || echo "❌ User not in dialout group"
else
    echo "No /dev/ttyACM0 device"
fi

echo -e "\n=== Controller Test ==="
python3 -c "
from basicmicro import Basicmicro
try:
    controller = Basicmicro('/dev/ttyACM0', 38400)
    if controller.Open():
        version = controller.ReadVersion(0x80)
        if version[0]:
            print(f'✅ Controller: {version[1].strip()}')
        else:
            print('❌ Version read failed')
        controller.close()
    else:
        print('❌ Connection failed')
except Exception as e:
    print(f'❌ Error: {e}')
"
```

Make executable:
```bash
chmod +x check-basicmicro.sh
```

## Troubleshooting

### Device Not Visible in WSL

#### Check USB/IP Status
```powershell
# Windows: Check attachment status
usbipd list
```

#### Re-attach Device
```powershell
# Detach first
usbipd detach --busid 2-3

# Re-attach
usbipd attach --wsl --busid 2-3
```

### Permission Denied Errors

```bash
# Check current permissions
ls -la /dev/ttyACM0

# Temporary fix
sudo chmod 666 /dev/ttyACM0

# Check group membership
groups $USER

# Add to dialout group if needed
sudo usermod -a -G dialout $USER
# Logout and login required
```

### Device Disappears After Restart

#### Create Startup Script
Create `startup-usb.ps1` (run at Windows startup):

```powershell
# Wait for system to fully boot
Start-Sleep 30

# Auto-attach Basicmicro devices
& "C:\path\to\attach-basicmicro.ps1"
```

Add to Windows startup via Task Scheduler.

### Communication Errors

#### Check Windows Device Manager
1. Open Device Manager
2. Look for "Ports (COM & LPT)"
3. Verify Basicmicro device is listed
4. Check COM port number
5. Update driver if needed

#### WSL Network Issues
```bash
# Test TCP connectivity (if using socat method)
nc -zv localhost 9999

# Check for port conflicts
netstat -an | grep 9999
```

## Performance Optimization

### USB/IP Performance
```bash
# Check USB version and speed
lsusb -v | grep -A5 "03eb:2404"

# Monitor USB traffic
sudo usbmon
```

### Serial Port Optimization
```bash
# Set optimal serial parameters
stty -F /dev/ttyACM0 speed 38400 cs8 -cstopb -parenb raw
```

## Integration with ROS2

### Environment Variables
Add to `~/.bashrc`:

```bash
# Auto-detect Basicmicro device
export BASICMICRO_PORT=$(ls /dev/ttyACM* 2>/dev/null | head -1)
export BASICMICRO_BAUD=38400

# Function to check device status
check_basicmicro() {
    if [ -e "$BASICMICRO_PORT" ]; then
        echo "✅ Basicmicro device: $BASICMICRO_PORT"
    else
        echo "❌ No Basicmicro device found"
        echo "Run: ./check-basicmicro.sh"
    fi
}
```

### Launch File Parameters
```xml
<!-- In your launch file -->
<node pkg="basicmicro_driver" exec="basicmicro_node.py">
    <param name="port" value="$(env BASICMICRO_PORT /dev/ttyACM0)"/>
    <param name="baud" value="$(env BASICMICRO_BAUD 38400)"/>
</node>
```

## WSL-Specific Best Practices

1. **Always detach USB devices** before shutting down WSL
2. **Use automation scripts** for consistent setup
3. **Monitor device status** with check scripts
4. **Set appropriate udev rules** for permissions
5. **Keep Windows drivers updated** for better compatibility

## Success Checklist

- [ ] usbipd-win installed on Windows
- [ ] USB device shared and attached to WSL
- [ ] Serial device visible in WSL (`/dev/ttyACM0`)
- [ ] Permissions configured (dialout group)
- [ ] Controller communication test passed
- [ ] Automation scripts created and tested
- [ ] ROS2 integration working

**WSL setup typically adds 10-15 minutes to initial configuration.**

---
*For general installation instructions, see the [Installation Guide](installation.md). For controller configuration, see [Motor Controller Setup](motor_controller_setup.md).*