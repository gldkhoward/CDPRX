# ROS2 Launch Command Missing: Troubleshooting Guide

## The Issue

When setting up ROS2 Humble on Ubuntu, users may encounter an issue where the `ros2 launch` command is not available, even though basic ROS2 functionality works. This typically manifests as:

```bash
ros2 launch my_package my_launch_file.py
# Error: argument Call `ros2 <command> -h` for more detailed usage.: invalid choice: 'launch'
```

## Root Cause

This issue typically occurs when:

1. ROS2 is installed without all the necessary packages for launch functionality
2. Only the base ROS2 system is installed instead of the full desktop installation
3. The launch-related packages were not installed automatically due to dependency resolution issues

The key missing packages that cause this issue are:
- ros-humble-ros2launch
- ros-humble-launch-testing
- ros-humble-launch-testing-ament-cmake
- ros-humble-launch-testing-ros

## Solution

### Quick Fix
If you encounter this issue, you can resolve it by installing the missing packages:

```bash
sudo apt install ros-humble-ros2launch ros-humble-launch-testing ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros
```

### Comprehensive Fix
For a complete ROS2 development environment, install the full desktop version:

```bash
sudo apt install ros-humble-desktop ros-humble-ros2cli-common-extensions
```

## Prevention

To avoid this issue in future installations:

1. Always install the full desktop version of ROS2:
```bash
sudo apt install ros-humble-desktop
```

2. After installation, verify all essential commands are available:
```bash
ros2 --help
```

The output should include these essential commands:
- launch
- topic
- action
- multicast
- lifecycle
- doctor/wtf

3. If setting up a development environment, also install development tools:
```bash
sudo apt install ros-humble-ros2cli-common-extensions
```

## Verifying Installation

To verify your ROS2 installation is complete:

1. Check available commands:
```bash
ros2 --help
```

2. Test launch functionality:
```bash
ros2 launch --help
```

3. Check if key packages are installed:
```bash
dpkg -l | grep ros-humble-launch
```

## Common Signs of Incomplete Installation

Watch out for these signs that might indicate an incomplete ROS2 installation:

1. Missing commands in `ros2 --help` output
2. Error messages about "invalid choice: 'launch'"
3. Fewer installed packages compared to a working system
4. Missing ROS2 environment variables

## Additional Resources

- [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [ROS2 Launch Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS2 CLI Tools](https://docs.ros.org/en/humble/Concepts/About-Command-Line-Tools.html)
