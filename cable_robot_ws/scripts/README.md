# Scripts Directory

This directory contains utility scripts for managing and running the cable-driven robot project.

## Available Scripts

### setup_workspace.sh
Sets up the ROS2 workspace and builds the project.

Usage:
```bash
./setup_workspace.sh
```

### hello_world.py
A simple example script showing how to create a basic ROS2 publisher.

Usage:
```bash
# Make sure the script is executable
chmod +x hello_world.py

# Run the script
./hello_world.py
```

## Making Scripts Executable

All scripts in this directory should be made executable before use. To make a script executable:

```bash
chmod +x script_name.sh
```

or for Python scripts:

```bash
chmod +x script_name.py
```

## General Usage Notes

1. All scripts should be run from the workspace root directory
2. Make sure ROS2 environment is sourced before running scripts:
   ```bash
   source /opt/ros/humble/setup.bash
   source ../install/setup.bash
   ```
3. Python scripts require Python 3.6 or newer

## Adding New Scripts

When adding new scripts:
1. Make them executable using chmod
2. Add documentation to this README
3. Follow the existing naming conventions
4. Include usage examples in script comments