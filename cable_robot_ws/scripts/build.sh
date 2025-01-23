#!/bin/bash

# Script location and workspace detection
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_ROOT="$(cd $SCRIPT_DIR/.. && pwd)"  # Assumes scripts is one level below workspace root

# Default values
PACKAGE="all"  # Build all packages by default
CLEAN=false

# Function to display usage
usage() {
    echo "Usage: $0 [-p package] [-c]"
    echo "Options:"
    echo "  -p    Package to build (core, hardware, gui, all)"
    echo "  -c    Clean build (removes build and install directories)"
    exit 1
}

# Parse command line arguments
while getopts "p:ch" opt; do
    case $opt in
        p)
            PACKAGE=$OPTARG
            ;;
        c)
            CLEAN=true
            ;;
        h)
            usage
            ;;
        \?)
            usage
            ;;
    esac
done

# Validate package argument
case $PACKAGE in
    core|hardware|gui|all)
        ;;
    *)
        echo "Error: Invalid package specified"
        usage
        ;;
esac

# Change to workspace root
cd $WORKSPACE_ROOT

# Clean if requested
if [ "$CLEAN" = true ]; then
    echo "Cleaning build directories..."
    rm -rf build/ install/
fi

# Build based on package selection
case $PACKAGE in
    all)
        echo "Building all packages..."
        colcon build
        ;;
    core)
        echo "Building cable_robot_core..."
        colcon build --packages-select cable_robot_core
        ;;
    hardware)
        echo "Building cable_robot_hardware..."
        colcon build --packages-select cable_robot_hardware
        ;;
    gui)
        echo "Building cable_robot_gui..."
        colcon build --packages-select cable_robot_gui
        ;;
esac