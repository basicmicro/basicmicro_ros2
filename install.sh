#!/bin/bash

# Basicmicro ROS2 Driver Installation Script
# This script automates the installation process for the Basicmicro ROS2 driver

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
DEFAULT_ROS_DISTRO="jazzy"
DEFAULT_WORKSPACE_PATH="$HOME/ros2_ws"
DEFAULT_SYSTEM_PATH="/opt/ros/$DEFAULT_ROS_DISTRO"
USE_VIRTUAL_ENV=true
INSTALL_MODE="development"  # development or system-wide

# Package configuration
BASICMICRO_PACKAGE="basicmicro"

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to install ROS2
install_ros2() {
    print_status "Installing ROS2 $DEFAULT_ROS_DISTRO..."
    
    # Step 1: Install prerequisites and add ROS2 repository
    print_status "Installing prerequisites..."
    sudo apt update
    sudo apt install -y software-properties-common curl
    
    print_status "Adding ROS2 GPG key..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    print_status "Adding ROS2 repository..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Step 2: Install ROS2
    print_status "Installing ROS2 $DEFAULT_ROS_DISTRO desktop..."
    sudo apt update
    sudo apt install -y ros-$DEFAULT_ROS_DISTRO-desktop python3-rosdep python3-colcon-common-extensions
    
    # Step 3: Initialize rosdep
    print_status "Initializing rosdep..."
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # Step 4: Set up environment
    print_status "Setting up ROS2 environment..."
    if ! grep -q "source /opt/ros/$DEFAULT_ROS_DISTRO/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/$DEFAULT_ROS_DISTRO/setup.bash" >> ~/.bashrc
        print_success "Added ROS2 source to ~/.bashrc"
    fi
    
    # Source for current session
    source "/opt/ros/$DEFAULT_ROS_DISTRO/setup.bash"
    
    print_success "ROS2 $DEFAULT_ROS_DISTRO installation complete!"
}

# Function to check ROS2 installation
check_ros2_installation() {
    print_status "Checking ROS2 installation..."
    
    if [ -z "$ROS_DISTRO" ]; then
        if [ -f "/opt/ros/$DEFAULT_ROS_DISTRO/setup.bash" ]; then
            source "/opt/ros/$DEFAULT_ROS_DISTRO/setup.bash"
            print_success "ROS2 $DEFAULT_ROS_DISTRO found and sourced"
        else
            print_warning "ROS2 not found on this system."
            echo ""
            echo -e "${YELLOW}Would you like to install ROS2 $DEFAULT_ROS_DISTRO now?${NC}"
            echo -e "${BLUE}This will:${NC}"
            echo -e "  - Install ROS2 $DEFAULT_ROS_DISTRO Desktop"
            echo -e "  - Set up development tools (colcon, rosdep)"
            echo -e "  - Configure environment variables"
            echo -e "  - Add ROS2 sourcing to ~/.bashrc"
            echo ""
            read -p "Install ROS2 $DEFAULT_ROS_DISTRO? (y/n): " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                install_ros2
            else
                print_error "ROS2 installation declined. Cannot proceed without ROS2."
                print_error "To install ROS2 manually, visit: https://docs.ros.org/en/jazzy/Installation.html"
                exit 1
            fi
        fi
    else
        print_success "ROS2 $ROS_DISTRO already sourced"
    fi
}

# Function to setup dependencies
setup_dependencies() {
    print_status "Setting up dependencies..."
    
    # Set paths for later use
    PROJECT_ROOT="$(pwd)"
    
    print_success "Dependencies setup complete"
}

# Function to check workspace setup
setup_workspace() {
    print_status "Setting up workspace..."
    
    if [ ! -d "$DEFAULT_WORKSPACE_PATH" ]; then
        mkdir -p "$DEFAULT_WORKSPACE_PATH/src"
        print_success "Created workspace directory: $DEFAULT_WORKSPACE_PATH"
    else
        print_success "Workspace already exists: $DEFAULT_WORKSPACE_PATH"
    fi
    
    # Setup dependencies (clone basicmicro_python)
    setup_dependencies
    
    # Copy current driver directory to workspace
    print_status "Copying driver to workspace..."
    if [ -d "$DEFAULT_WORKSPACE_PATH/src/basicmicro_driver" ]; then
        print_warning "Driver already exists in workspace, removing old version..."
        rm -rf "$DEFAULT_WORKSPACE_PATH/src/basicmicro_driver"
    fi
    
    cp -r "$PROJECT_ROOT" "$DEFAULT_WORKSPACE_PATH/src/basicmicro_driver"
    print_success "Driver copied to workspace"
}

# Function to setup Python environment
setup_python_environment() {
    if [ "$INSTALL_MODE" = "system-wide" ]; then
        print_status "Using system Python environment for system-wide installation..."
        print_success "System Python environment selected"
        return
    fi
    
    print_status "Setting up Python environment..."
    
    cd "$DEFAULT_WORKSPACE_PATH"
    
    if [ "$USE_VIRTUAL_ENV" = true ]; then
        if [ ! -d "venv" ]; then
            print_status "Creating Python virtual environment..."
            python3 -m venv venv --system-site-packages
            print_success "Virtual environment created with system packages access"
        else
            print_success "Virtual environment already exists"
        fi
        
        print_status "Activating virtual environment..."
        source venv/bin/activate
        print_success "Virtual environment activated"
    else
        print_warning "Using system Python environment"
    fi
}

# Function to install Python dependencies
install_python_dependencies() {
    if [ "$INSTALL_MODE" = "system-wide" ]; then
        print_status "Installing Python dependencies system-wide..."
        # Use apt for system-wide installation to avoid PEP 668 issues
        sudo apt update
        sudo apt install -y python3-yaml python3-numpy python3-serial python3-empy python3-lark python3-catkin-pkg python3-pip python3-setuptools python3-wheel ros-$DEFAULT_ROS_DISTRO-ament-package ros-$DEFAULT_ROS_DISTRO-ament-cmake ros-$DEFAULT_ROS_DISTRO-ament-cmake-core
        
        # Install basicmicro package system-wide
        print_status "Installing Basicmicro Python library system-wide..."
        sudo pip3 install "$BASICMICRO_PACKAGE" --break-system-packages
        print_success "System Python dependencies and Basicmicro library installed"
        return
    fi
    
    print_status "Installing Python dependencies..."
    
    # Install required packages
    pip install -U pip setuptools wheel
    pip install PyYAML numpy pyserial empy lark-parser catkin_pkg
    
    # Install basicmicro package
    print_status "Installing Basicmicro Python library..."
    pip install "$BASICMICRO_PACKAGE"
    
    print_success "Python dependencies and Basicmicro library installed"
}


# Function to install ROS2 dependencies
install_ros2_dependencies() {
    print_status "Installing ROS2 dependencies..."
    
    # Install rosdep if not present
    if ! command_exists rosdep; then
        # rosdep is part of ROS2 installation, should already be available
        print_warning "rosdep not found - this should have been installed with ROS2"
    fi
    
    # Update rosdep
    rosdep update
    
    # Install dependencies
    cd "$DEFAULT_WORKSPACE_PATH"
    rosdep install --from-paths src/basicmicro_driver --ignore-src -r -y
    
    print_success "ROS2 dependencies installed"
}

# Function to build the package
build_package() {
    print_status "Building ROS2 package..."
    
    if [ "$INSTALL_MODE" = "system-wide" ]; then
        # Create temporary build directory
        TEMP_WS="/tmp/basicmicro_build_ws"
        mkdir -p "$TEMP_WS/src"
        
        # Copy the cloned ROS2 driver to temporary workspace
        ls -la "$PROJECT_ROOT" || print_error "Project root does not exist!"
        
        # Copy the driver package to temporary workspace
        cp -r "$PROJECT_ROOT" "$TEMP_WS/src/basicmicro_driver"
        cd "$TEMP_WS"
        
        # Verify workspace structure
        find src -name "package.xml" -type f | head -5
        colcon list || print_warning "colcon list failed"
        
        # Source ROS2 environment
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        
        # Verify Python environment
        python3 -c "import ament_package.templates; print('✅ ament_package import successful')" || print_warning "❌ ament_package import failed"
        
        # Build and install system-wide
        print_status "Building package..."
        if colcon build --packages-select basicmicro_driver --cmake-args -DCMAKE_BUILD_TYPE=Release; then
            print_status "Build succeeded, installing to system location..."
            
            # Copy built package to system location
            if [ -d "install/basicmicro_driver" ]; then
                print_status "Installing package to system location..."
                sudo cp -r install/basicmicro_driver/* "$DEFAULT_SYSTEM_PATH/" || {
                    print_error "Failed to copy package to system location"
                    exit 1
                }
                print_success "Package built and installed system-wide"
                # Clean up (ignore permission errors)
                rm -rf "$TEMP_WS" 2>/dev/null || sudo rm -rf "$TEMP_WS"
            else
                print_error "Build directory not found"
                exit 1
            fi
        else
            print_error "Package build failed"
            exit 1
        fi
    else
        cd "$DEFAULT_WORKSPACE_PATH"
        
        # Source ROS2 environment
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        
        # Build the package
        colcon build --packages-select basicmicro_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
        
        if [ $? -eq 0 ]; then
            print_success "Package built successfully"
        else
            print_error "Package build failed"
            exit 1
        fi
    fi
}

# Function to setup serial port permissions
setup_serial_permissions() {
    print_status "Setting up serial port permissions..."
    
    # Add user to dialout group for serial port access
    if ! groups $USER | grep -q "dialout"; then
        sudo usermod -a -G dialout $USER
        print_warning "Added user to dialout group. You may need to log out and log back in for changes to take effect."
    else
        print_success "User already in dialout group"
    fi
}

# Function to run hardware test
run_hardware_test() {
    print_status "Running hardware connection test..."
    
    # Look for available serial ports
    SERIAL_PORTS=$(ls /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* 2>/dev/null || echo "")
    
    if [ -z "$SERIAL_PORTS" ]; then
        print_warning "No serial ports found. Please connect your Basicmicro controller and ensure it's recognized by the system."
        return 1
    fi
    
    print_success "Found serial ports: $SERIAL_PORTS"
    
    # Test with the first available port
    FIRST_PORT=$(echo $SERIAL_PORTS | awk '{print $1}')
    
    print_status "Testing connection to $FIRST_PORT..."
    
    python3 -c "
import sys
try:
    from basicmicro import Basicmicro
    controller = Basicmicro('$FIRST_PORT', 38400)
    success = controller.Open()
    if success:
        version = controller.ReadVersion(0x80)
        if version[0]:
            print('✅ Controller connected: ' + version[1].strip())
            controller.close()
            sys.exit(0)
        else:
            print('❌ Failed to read controller version')
            controller.close()
            sys.exit(1)
    else:
        print('❌ Failed to open controller connection')
        sys.exit(1)
except Exception as e:
    print('❌ Hardware test failed: ' + str(e))
    sys.exit(1)
"
    
    if [ $? -eq 0 ]; then
        print_success "Hardware test passed"
        return 0
    else
        print_warning "Hardware test failed. Please check your controller connection and port settings."
        return 1
    fi
}

# Function to create launch script
create_launch_script() {
    print_status "Creating launch script..."
    
    if [ "$INSTALL_MODE" = "system-wide" ]; then
        # System-wide installation - simple launch script that works from anywhere
        cat > "$HOME/launch_basicmicro.sh" << 'EOF'
#!/bin/bash

# Basicmicro ROS2 Driver Launch Script (System-wide)
# This script launches the driver from any directory

# Configuration
DEFAULT_PORT="/dev/ttyACM0"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Check for port argument
if [ "$#" -gt 0 ]; then
    PORT="$1"
else
    PORT="$DEFAULT_PORT"
fi

echo -e "${GREEN}[INFO]${NC} Launching Basicmicro ROS2 driver on port: $PORT"

# Launch the driver (works from any directory)
echo -e "${GREEN}[INFO]${NC} Starting Basicmicro ROS2 driver..."
ros2 run basicmicro_driver basicmicro_node.py --ros-args -p port:=$PORT

EOF
        chmod +x "$HOME/launch_basicmicro.sh"
        print_success "Launch script created: $HOME/launch_basicmicro.sh"
    else
        # Development installation - workspace-based launch script
        cat > "$DEFAULT_WORKSPACE_PATH/launch_basicmicro.sh" << 'EOF'
#!/bin/bash

# Basicmicro ROS2 Driver Launch Script (Development)
# This script sets up the development environment and launches the driver

# Configuration
WORKSPACE_PATH="$HOME/ros2_ws"
USE_VIRTUAL_ENV=true
DEFAULT_PORT="/dev/ttyACM0"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Check for port argument
if [ "$#" -gt 0 ]; then
    PORT="$1"
else
    PORT="$DEFAULT_PORT"
fi

echo -e "${GREEN}[INFO]${NC} Launching Basicmicro ROS2 driver on port: $PORT"

# Setup environment
cd "$WORKSPACE_PATH"

# Source ROS2 environment
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
else
    echo -e "${RED}[ERROR]${NC} ROS2 not found"
    exit 1
fi

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${RED}[ERROR]${NC} Workspace not built. Please run: colcon build --packages-select basicmicro_driver"
    exit 1
fi

# Activate virtual environment if used
if [ "$USE_VIRTUAL_ENV" = true ] && [ -d "venv" ]; then
    source venv/bin/activate
    echo -e "${GREEN}[INFO]${NC} Virtual environment activated"
fi

# Launch the driver
echo -e "${GREEN}[INFO]${NC} Starting Basicmicro ROS2 driver..."
ros2 run basicmicro_driver basicmicro_node.py --ros-args -p port:=$PORT

EOF
        chmod +x "$DEFAULT_WORKSPACE_PATH/launch_basicmicro.sh"
        print_success "Launch script created: $DEFAULT_WORKSPACE_PATH/launch_basicmicro.sh"
    fi
}

# Function to display final instructions
display_final_instructions() {
    echo ""
    echo -e "${GREEN}================================="
    echo -e "  INSTALLATION COMPLETE!"
    echo -e "=================================${NC}"
    echo ""
    echo -e "${BLUE}Quick Start:${NC}"
    echo -e "1. Connect your Basicmicro controller via USB"
    echo -e "2. Run the launch script:"
    if [ "$INSTALL_MODE" = "system-wide" ]; then
        echo -e "   ${YELLOW}~/launch_basicmicro.sh /dev/ttyACM0${NC}"
        echo -e "   ${YELLOW}# Works from any directory${NC}"
    else
        echo -e "   ${YELLOW}cd $DEFAULT_WORKSPACE_PATH${NC}"
        echo -e "   ${YELLOW}./launch_basicmicro.sh /dev/ttyACM0${NC}"
    fi
    echo -e "   ${YELLOW}# Replace /dev/ttyACM0 with your actual port${NC}"
    echo ""
    echo -e "${BLUE}Manual Launch:${NC}"
    if [ "$INSTALL_MODE" = "system-wide" ]; then
        echo -e "   ${YELLOW}ros2 run basicmicro_driver basicmicro_node.py --ros-args -p port:=/dev/ttyACM0${NC}"
        echo -e "   ${YELLOW}# Works from any directory${NC}"
    else
        echo -e "   ${YELLOW}cd $DEFAULT_WORKSPACE_PATH${NC}"
        echo -e "   ${YELLOW}source /opt/ros/$ROS_DISTRO/setup.bash${NC}"
        echo -e "   ${YELLOW}source install/setup.bash${NC}"
        if [ "$USE_VIRTUAL_ENV" = true ]; then
            echo -e "   ${YELLOW}source venv/bin/activate${NC}"
        fi
        echo -e "   ${YELLOW}ros2 run basicmicro_driver basicmicro_node.py --ros-args -p port:=/dev/ttyACM0${NC}"
    fi
    echo -e "   ${YELLOW}# Replace /dev/ttyACM0 with your actual port${NC}"
    echo ""
    echo -e "${BLUE}Test Motor Movement:${NC}"
    echo -e "   ${YELLOW}ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.1}, angular: {z: 0.0}}\" --once${NC}"
    echo ""
    echo -e "${BLUE}Documentation:${NC}"
    echo -e "   See docs/ directory for detailed guides and examples"
    echo ""
    
    if [ "$USE_VIRTUAL_ENV" = true ]; then
        echo -e "${YELLOW}Note: Virtual environment is used for Python dependencies${NC}"
    fi
}

# Main installation function
main() {
    echo -e "${GREEN}================================="
    echo -e "  Basicmicro ROS2 Driver Installer"
    echo -e "=================================${NC}"
    echo ""
    
    # Store absolute script path before any directory changes
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
    
    # Check if git is available
    if ! command_exists git; then
        print_error "Git is required for this installation. Please install git first:"
        print_error "sudo apt install git"
        exit 1
    fi
    
    # Check if script is run from correct directory
    if [ ! -f "package.xml" ] || [ ! -f "setup.py" ]; then
        print_error "This script must be run from the basicmicro_driver directory"
        print_error "Please run: git clone https://github.com/basicmicro/basicmicro_ros2.git"
        print_error "Then: cd basicmicro_ros2/basicmicro_driver && ./install.sh"
        exit 1
    fi
    
    print_status "Starting installation process..."
    print_status "Installation mode: $INSTALL_MODE"
    
    # Installation steps
    check_ros2_installation
    setup_workspace
    setup_python_environment
    install_python_dependencies
    install_ros2_dependencies
    build_package
    setup_serial_permissions
    create_launch_script
    
    # Optional hardware test
    echo ""
    read -p "Would you like to run a hardware connection test? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        run_hardware_test
    fi
    
    display_final_instructions
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --system-wide)
            INSTALL_MODE="system-wide"
            USE_VIRTUAL_ENV=false
            shift
            ;;
        --dev|--development)
            INSTALL_MODE="development"
            shift
            ;;
        --no-venv)
            USE_VIRTUAL_ENV=false
            shift
            ;;
        --workspace)
            DEFAULT_WORKSPACE_PATH="$2"
            shift 2
            ;;
        --ros-distro)
            DEFAULT_ROS_DISTRO="$2"
            DEFAULT_SYSTEM_PATH="/opt/ros/$DEFAULT_ROS_DISTRO"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Installation Modes:"
            echo "  (default)           Development mode - installs to workspace"
            echo "  --system-wide       System-wide installation (requires sudo)"
            echo "  --dev               Explicitly use development mode"
            echo ""
            echo "Options:"
            echo "  --no-venv           Use system Python instead of virtual environment"
            echo "  --workspace PATH    Set workspace path (default: $DEFAULT_WORKSPACE_PATH)"
            echo "  --ros-distro DISTRO Set ROS2 distribution (default: $DEFAULT_ROS_DISTRO)"
            echo "  -h, --help          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                  # Development installation"
            echo "  $0 --system-wide    # System-wide installation"
            echo "  $0 --no-venv        # Development without virtual environment"
            echo ""
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Run main installation
main