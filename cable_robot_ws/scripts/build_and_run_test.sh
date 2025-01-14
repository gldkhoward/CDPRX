#!/bin/bash

# Function to print colored output
print_status() {
    echo -e "\e[1;34m>>> $1\e[0m"
}

print_error() {
    echo -e "\e[1;31m>>> Error: $1\e[0m"
}

print_success() {
    echo -e "\e[1;32m>>> Success: $1\e[0m"
}

# Navigate to workspace root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"

print_status "Building cable_robot workspace..."

# Source ROS 2 environment if not already sourced
if [[ -z "$ROS_DISTRO" ]]; then
    print_status "Sourcing ROS 2 environment..."
    source /opt/ros/humble/setup.bash
fi

# Create build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir build
fi

# Build the workspace
print_status "Running colcon build..."
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --symlink-install

# Check if build was successful
if [ $? -eq 0 ]; then
    print_success "Build completed successfully!"
else
    print_error "Build failed!"
    exit 1
fi

# Source the workspace
print_status "Sourcing workspace..."
source install/setup.bash

# Launch the simulation
print_status "Launching cable robot simulation..."
ros2 launch cable_robot_entry simulation.launch.py