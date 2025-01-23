# CDPRX Build & Utility Scripts

This directory contains utility scripts for building, testing, and managing the CDPRX cable robot project.

## Contents

1. [Build Scripts](#build-scripts)
2. [Usage Examples](#usage-examples)
3. [Adding New Scripts](#adding-new-scripts)
4. [Common Issues](#common-issues)

## Build Scripts

### build.sh

Manages the build process for CDPRX packages. Can be run from any directory within the project.

**Options:**

- `-p <package>`: Select package to build (core, hardware, gui, all)
- `-c`: Clean build (removes build and install directories)
- `-h`: Display help message

**Supported Packages:**

- `core`: cable_robot_core
- `hardware`: cable_robot_hardware
- `gui`: cable_robot_gui
- `all`: Builds all packages (default)

## Usage Examples

Build all packages:

```bash
./scripts/build.sh
```

Clean build of core package:

```bash
./scripts/build.sh -p core -c
```

Build GUI package:

```bash
./scripts/build.sh -p gui
```

## Adding New Scripts

When adding new scripts to this directory:

1. **Naming Convention:**

   - Use descriptive, lowercase names
   - Use hyphens for spaces (e.g., `run-tests.sh`)
   - Add the appropriate file extension (`.sh` for bash scripts)
2. **Documentation Requirements:**

   - Add a section in this README under the appropriate category
   - Include usage examples
   - Document all command-line options
   - List any dependencies
3. **Script Guidelines:**

   - Include a usage function
   - Add error handling
   - Make script executable (`chmod +x script-name.sh`)
   - Include comments for complex operations
   - Use the `SCRIPT_DIR` variable for relative paths

## Common Issues

### Build Script Issues

- **Error: Invalid package specified**

  - Ensure you're using one of the supported package names
  - Check for typos in the package parameter
- **Build fails after clean**

  - Ensure all dependencies are installed
  - Check package.xml for missing dependencies

### Troubleshooting

If you encounter issues:

1. Use `-h` flag to verify correct usage
2. Ensure you're in the correct directory
3. Check script permissions (`chmod +x`)
4. Verify all dependencies are installed

## Contributing

When contributing new scripts:

1. Follow the guidelines in [Adding New Scripts](#adding-new-scripts)
2. Test the script thoroughly
3. Update this README
4. Submit a pull request with your changes

## License

This script collection is part of the CDPRX project and follows the project's licensing terms.
